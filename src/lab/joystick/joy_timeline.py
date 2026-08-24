#!/usr/bin/env python3
"""joy_timeline.py — разбор РУЧНОГО freefly-полёта по bag: что и когда пилот
делал стиками/тумблером. Выход — три артефакта в OUT (дефолт
/root/sim_ws/output/joystick/):

  report.txt          — человекочитаемая лента: щелчки CH6, жесты арм/дизарм,
                        отрыв/макс.высота/касание, сегменты стиков (+stdout);
  scenario_draft.json — ЧЕРНОВИК сценария для joy_replay.py: жесты заменены
                        замкнутыми якорями arm/disarm, тайминги — hold'ами.
                        Черновик правится руками (заменить тайминги набора
                        высоты на wait_alt и т.п.) и кладётся в
                        src/lab/joystick/scenarios/;
  raw.jsonl           — сырой таймлайн /joy в sim-времени для joy_replay --raw
                        (валидация канала реплея, не для воспроизводимости).

ВРЕМЯ. Штампы /joy — wall (joy_linux_node без use_sim_time), а реплей живёт в
sim-времени. Мост: bag-receive-время (wall) реперного sim-штампованного топика
(--ref-topic, дефолт ground-truth /model/iris_cam/odometry — есть в каждом
freefly-bag через TOPICS_EXTRA) даёт кусочно-линейное отображение wall→sim,
через него /joy пересчитывается в sim. Заодно ref даёт высоту (gt z − базлайн).

СЕМАНТИКА значений в отчёте/сценарии — PWM-конвенция (v=(PWM−1500)/400), знаки
JOY_SIGNS уже сняты: thr −1 = газ min, yaw +1 = вправо, pitch −1 = вперёд.
См. докстринг joy_replay.py.

Запуск внутри p1317_nav (или через analyze.sh с хоста):
  python3 /lab/joystick/joy_timeline.py [bag] [--out DIR] [--eps 0.1] ...
"""
import argparse
import json
import os
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

AXES = ('roll', 'pitch', 'thr', 'yaw')
SW_NAMES = {-1: 'ВВЕРХ(-1, наш стек)', 0: 'ЦЕНТР(0, ALT_HOLD/LOITER)',
            1: 'ВНИЗ(+1, MANUAL)'}
# Пороги жеста ОТКАЛИБРОВАНЫ 2026-08-24 по 13 реплейным bag'ам против истины
# /mavros/state: recall 13/13 и НОЛЬ ложных на всей сетке LVL 0.5-0.9 (фоновый
# max|yaw| при газе<−0.5 вне жестов ≈ 0.00 — предикат «газ в полу И yaw в
# упоре» самоселективен). 0.85 подтверждён и единственным ручным полётом
# (182409, жесты пилота детектились). Двигать не по чему — живые-руки-данные
# соберёт следующий ручной полёт (старые ручные bag'и удалены при чистках).
GESTURE_LVL = 0.85       # |thr|,|yaw| для распознавания руддер-жеста
GESTURE_MIN = 0.5        # мин. длительность жеста, sim-с
GESTURE_GAP = 3.0        # слияние импульсов жеста с паузой ≤gap в ОДНО событие:
                         # реплей давит руддер-дизарм импульсами 4с/2с
                         # (joy_replay), и отказ FCU выглядел бы десятком
                         # жестов в ленте. Та же семантика «отпустил = пауза
                         # >3 с», что у страховки дизарма Freefly.
LIFTOFF_ALT = 0.3        # м: отрыв
TOUCH_ALT = 0.15         # м: касание после апекса


def default_signs():
    try:
        from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT
        return JOY_SIGNS_DEFAULT
    except ImportError:
        sys.path.insert(0, '/root/sim_ws/src/control')
        from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT
        return JOY_SIGNS_DEFAULT


def read_bag(bag, joy_topic, ref_topic, state_topic='/mavros/state',
             hud_topic='/mission/status'):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))
    joy_ns, joy_axes, ref, fcu, hud = [], [], [], [], []
    while r.has_next():
        topic, raw, t_ns = r.read_next()
        if topic == joy_topic:
            m = deserialize_message(raw, Joy)
            a = list(m.axes) + [0.0] * 6
            joy_ns.append(t_ns)
            joy_axes.append(a[:6])
        elif topic == ref_topic:
            m = deserialize_message(raw, Odometry)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            ref.append((t_ns, st, m.pose.pose.position.z))
        elif topic == state_topic:
            # ленивый импорт: в старых bag топика нет, mavros_msgs не обязателен
            from mavros_msgs.msg import State
            m = deserialize_message(raw, State)
            fcu.append((t_ns, bool(m.armed), m.mode))
        elif topic == hud_topic:
            # гейт LOITER-на-VINS от лётной ноды (debug-HUD, "k=v k=v ...")
            from std_msgs.msg import String
            m = deserialize_message(raw, String)
            kv = dict(p.split('=', 1) for p in m.data.split() if '=' in p)
            hud.append((t_ns, kv.get('st', '?'), kv.get('why', '-'),
                        kv.get('odom', '?')))
    return (np.array(joy_ns, dtype=np.int64), np.array(joy_axes),
            np.array(ref), fcu, hud)


def zoh(t_src, v_src, grid):
    idx = np.clip(np.searchsorted(t_src, grid, side='right') - 1,
                  0, len(t_src) - 1)
    return v_src[idx]


def rle(vals):
    """[(i0, i1, val)] по массиву (i1 эксклюзивно)."""
    out, i0 = [], 0
    for i in range(1, len(vals) + 1):
        if i == len(vals) or vals[i] != vals[i0]:
            out.append((i0, i, vals[i0]))
            i0 = i
    return out


def segments(vals, dt, min_dur):
    """RLE с поглощением коротких прогонов в предыдущий + слияние равных."""
    segs = rle(list(vals))
    out = []
    for i0, i1, v in segs:
        if out and (i1 - i0) * dt < min_dur:
            out[-1] = (out[-1][0], i1, out[-1][2])
        elif out and v == out[-1][2]:
            out[-1] = (out[-1][0], i1, v)
        else:
            out.append((i0, i1, v))
    # после поглощений соседи могли сравняться
    merged = []
    for s in out:
        if merged and s[2] == merged[-1][2]:
            merged[-1] = (merged[-1][0], s[1], s[2])
        else:
            merged.append(s)
    return merged


def sustained_runs(mask, dt, min_dur):
    return [(i0, i1) for i0, i1, v in rle(list(mask))
            if v and (i1 - i0) * dt >= min_dur]


def merge_runs(runs, dt, gap):
    """Слить прогоны с паузой ≤ gap (сек) в один: импульсный жест = одно событие."""
    out = []
    for i0, i1 in runs:
        if out and (i0 - out[-1][1]) * dt <= gap:
            out[-1][1] = i1
        else:
            out.append([i0, i1])
    return [(a, b) for a, b in out]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', nargs='?', default='/root/sim_ws/output/scene_bag')
    ap.add_argument('--out', default='/root/sim_ws/output/joystick')
    ap.add_argument('--joy-topic', default='/joy')
    ap.add_argument('--ref-topic', default='/model/iris_cam/odometry',
                    help='sim-штампованный топик: мост wall→sim + высота (gt)')
    ap.add_argument('--signs', default='',
                    help='знаки осей "r,p,t,y" (дефолт — из control_pkg)')
    ap.add_argument('--eps', type=float, default=0.1,
                    help='шаг квантования стиков для сегментации')
    ap.add_argument('--min-dur', type=float, default=0.3,
                    help='мин. длительность сегмента, sim-с')
    ap.add_argument('--grid-hz', type=float, default=20.0)
    args = ap.parse_args()
    signs = (tuple(float(x) for x in args.signs.split(','))
             if args.signs else default_signs())

    joy_ns, joy_axes, ref, fcu, hud = read_bag(args.bag, args.joy_topic,
                                               args.ref_topic)
    if len(joy_ns) == 0:
        sys.exit(f"ОШИБКА: в {args.bag} нет {args.joy_topic}. Полёт был с "
                 f"BS_PILOT=joy и /joy в TOPICS_EXTRA?")
    rep = []                                   # строки отчёта

    # --- мост wall→sim + высота ---
    if len(ref) >= 2:
        sim_joy = np.interp(joy_ns, ref[:, 0], ref[:, 1])
        z = ref[:, 2]
        z0 = np.median(z[:max(1, min(20, len(z)))])
        alt_t, alt_v = ref[:, 1], z - z0
        wall_span = (joy_ns[-1] - joy_ns[0]) / 1e9
        rtf = (sim_joy[-1] - sim_joy[0]) / wall_span if wall_span > 0 else 0
    else:
        rep.append(f"⚠️ {args.ref_topic} в bag нет — беру wall-время как sim "
                   f"(верно только при RTF≈1)")
        sim_joy = (joy_ns - joy_ns[0]) / 1e9
        alt_t = alt_v = np.array([])
        rtf = 1.0

    t0 = sim_joy[0]
    span = sim_joy[-1] - t0
    rep.insert(0, f"=== joy_timeline: {args.bag} ===")
    rep.append(f"/joy: {len(joy_ns)} сообщ., {span:.1f} sim-с "
               f"(~{len(joy_ns) / max(span, 1e-6):.0f} sim-Гц), RTF≈{rtf:.2f}, "
               f"signs={signs}")

    # --- семантические серии на сетке (ZOH) ---
    dt = 1.0 / args.grid_hz
    grid = np.arange(t0, sim_joy[-1], dt)
    sem = {a: zoh(sim_joy, signs[i] * joy_axes[:, i], grid)
           for i, a in enumerate(AXES)}
    sw_raw = zoh(sim_joy, joy_axes[:, 5], grid)
    sw = np.where(sw_raw > 0.5, 1, np.where(sw_raw < -0.5, -1, 0))

    # квантование + сглаживание медианой (5 тиков ≈ 0.25 с) от дребезга
    def med5(x):
        if len(x) < 5:
            return x
        return np.median(np.lib.stride_tricks.sliding_window_view(
            np.pad(x, 2, mode='edge'), 5), axis=1)
    q = {a: np.round(med5(sem[a]) / args.eps) * args.eps for a in AXES}
    segs = {a: segments(q[a], dt, args.min_dur) for a in AXES}
    sw_segs = segments(sw, dt, args.min_dur)

    # --- события ---
    events = []                                # (t_rel, 'вид', данные)
    for i0, i1, v in sw_segs[1:]:
        events.append((grid[i0] - t0, 'sw', int(v)))
    arm_runs = merge_runs(sustained_runs((sem['thr'] < -GESTURE_LVL)
                                         & (sem['yaw'] > GESTURE_LVL),
                                         dt, GESTURE_MIN), dt, GESTURE_GAP)
    dis_runs = merge_runs(sustained_runs((sem['thr'] < -GESTURE_LVL)
                                         & (sem['yaw'] < -GESTURE_LVL),
                                         dt, GESTURE_MIN), dt, GESTURE_GAP)
    for i0, i1 in arm_runs:
        events.append((grid[i0] - t0, 'arm', (grid[i1 - 1] - grid[i0])))
    for i0, i1 in dis_runs:
        events.append((grid[i0] - t0, 'disarm', (grid[i1 - 1] - grid[i0])))
    if fcu:                                   # /mavros/state в bag (1 Гц): латчи
        fcu_ns = np.array([f[0] for f in fcu], dtype=np.int64)
        fcu_sim = (np.interp(fcu_ns, ref[:, 0], ref[:, 1]) if len(ref) >= 2
                   else (fcu_ns - joy_ns[0]) / 1e9)
        prev_a, prev_m = None, None
        for (t_ns, armed, mode), t_s in zip(fcu, fcu_sim):
            if prev_a is not None and armed != prev_a:
                events.append((t_s - t0, 'alt',
                               'FCU: ARMED' if armed else 'FCU: DISARMED'))
            if mode != prev_m and prev_m is not None:
                events.append((t_s - t0, 'alt', f'FCU: режим {prev_m} → {mode}'))
            prev_a, prev_m = armed, mode
    if hud:                                   # /mission/status в bag: гейт HUD
        hud_ns = np.array([h[0] for h in hud], dtype=np.int64)
        hud_sim = (np.interp(hud_ns, ref[:, 0], ref[:, 1]) if len(ref) >= 2
                   else (hud_ns - joy_ns[0]) / 1e9)
        prev_st = None
        for (t_ns, st, why, odom), t_s in zip(hud, hud_sim):
            if prev_st is not None and st != prev_st:
                label = {'READY': f'HUD: VINS READY (odom={odom})',
                         'WAIT': f'HUD: VINS WAIT ({why})'}.get(
                             st, f'HUD: NO VINS ({why})')
                events.append((t_s - t0, 'alt', label))
            prev_st = st
    if len(alt_t):
        up = np.where(alt_v > LIFTOFF_ALT)[0]
        if len(up):
            events.append((alt_t[up[0]] - t0, 'alt', f'отрыв (> {LIFTOFF_ALT} м)'))
            imax = int(np.argmax(alt_v))
            events.append((alt_t[imax] - t0, 'alt',
                           f'макс. высота {alt_v[imax]:.2f} м'))
            dn = np.where((alt_t > alt_t[imax]) & (alt_v < TOUCH_ALT))[0]
            if len(dn):
                events.append((alt_t[dn[0]] - t0, 'alt', 'касание'))
    events.sort(key=lambda e: e[0])

    rep.append("\nсобытия (t от первого /joy, sim-с):")
    for t, kind, d in events:
        if kind == 'sw':
            rep.append(f"  {t:7.1f}  CH6 → {SW_NAMES.get(d, d)}")
        elif kind == 'arm':
            rep.append(f"  {t:7.1f}  ЖЕСТ АРМ (газ min + yaw вправо), {d:.1f} с")
        elif kind == 'disarm':
            rep.append(f"  {t:7.1f}  ЖЕСТ ДИЗАРМ (газ min + yaw влево), {d:.1f} с")
        else:
            rep.append(f"  {t:7.1f}  {d}")

    rep.append(f"\nсегменты стиков (квант {args.eps}, мин {args.min_dur} с):")
    for a in AXES:
        parts = [f"{v:+.2f}×{(i1 - i0) * dt:.1f}с" for i0, i1, v in segs[a]]
        rep.append(f"  {a:5s}: " + " → ".join(parts))

    # --- черновик сценария ---
    arm_t = grid[arm_runs[0][0]] - t0 if arm_runs else None
    scn_t0 = max(0.0, arm_t - 2.0) if arm_t is not None else 0.0
    gi0 = int(np.searchsorted(grid - t0, scn_t0))
    init = {'sticks': {a: round(float(q[a][min(gi0, len(grid) - 1)]), 2)
                       for a in AXES},
            'sw': int(sw[min(gi0, len(grid) - 1)])}
    # маски жестов: их thr/yaw-сегменты — сам жест, в шаги не попадают
    gmask = [(grid[i0] - t0 - 0.3, grid[i1 - 1] - t0 + 0.3)
             for i0, i1 in arm_runs + dis_runs]

    def in_gesture(t):
        return any(a <= t <= b for a, b in gmask)

    ev2 = []                                    # (t_rel, dict-шаг)
    for a in AXES:
        for (p0, p1, pv), (i0, i1, v) in zip(segs[a], segs[a][1:]):
            t = grid[i0] - t0
            if t < scn_t0:
                continue
            # в окне жеста режем ТОЛЬКО сам жест (yaw ±1 / газ min): ввод
            # пилота сразу после жеста (напр. взлётный газ) — не терять
            if in_gesture(t) and ((a == 'yaw' and abs(v) > 0.5)
                                  or (a == 'thr' and v < -0.5)):
                continue
            ev2.append((t, {'sticks': {a: round(float(v), 2)}}))
    for i0, i1, v in sw_segs[1:]:
        t = grid[i0] - t0
        if t >= scn_t0:
            ev2.append((t, {'sw': int(v)}))
    if arm_runs:
        ev2.append((arm_t, {'arm': {'timeout': 120},
                            'note': 'руддер-арм (замкнуто по /mavros/state)'}))
    if dis_runs:
        ev2.append((grid[dis_runs[0][0]] - t0,
                    {'disarm': {'timeout': 30}, 'note': 'руддер-дизарм'}))
    ev2.sort(key=lambda e: e[0])

    steps, prev_t = [], scn_t0
    for t, step in ev2:
        if t - prev_t >= 0.25:
            steps.append({'hold': round(t - prev_t, 1)})
        # слить sticks-шаги одного момента (кластер < 0.15 с)
        if ('sticks' in step and steps and 'sticks' in steps[-1]
                and t - prev_t < 0.15):
            steps[-1]['sticks'].update(step['sticks'])
        else:
            steps.append(step)
        prev_t = t
        if 'disarm' in step:
            break
    scenario = {
        'version': 1,
        'name': os.path.basename(args.bag.rstrip('/')) + '_draft',
        'note': ('черновик joy_timeline.py — тайминги сырые; перед боевым '
                 'реплеем заменить наборы/снижения высоты на wait_alt, '
                 'латч режима — на wait_mode'),
        'init': init,
        'steps': steps,
    }

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'scenario_draft.json'), 'w') as f:
        json.dump(scenario, f, ensure_ascii=False, indent=2)
    with open(os.path.join(args.out, 'raw.jsonl'), 'w') as f:
        for t, a in zip(sim_joy, joy_axes):
            f.write(json.dumps({'t': round(float(t - t0), 4),
                                'axes': [round(float(x), 4) for x in a]}) + '\n')
    report = '\n'.join(rep) + '\n'
    with open(os.path.join(args.out, 'report.txt'), 'w') as f:
        f.write(report)
    print(report)
    print(f"→ {args.out}/report.txt, scenario_draft.json "
          f"({len(steps)} шагов), raw.jsonl ({len(joy_ns)} сэмплов)")


if __name__ == '__main__':
    main()
