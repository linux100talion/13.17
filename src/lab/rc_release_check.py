#!/usr/bin/env python3
"""rc_release_check.py — СЕМАНТИКА НУЛЯ в RC_CHANNELS_OVERRIDE. Прямой MAVLink.

Зачем. Сторож свежести пульта (control_pkg/domain/pilot_link.py) при тишине
источника шлёт ch1..4 = 0 (RC_RELEASE) вместо команды — в расчёте на то, что
ArduPilot трактует ноль как «ОТПУСТИТЬ канал обратно радио» (has_override()
false), а не как «не трогать» (это 65535, RC_NOCHANGE). До сих пор это было
утверждение из чужого кода, а не наш замер. Здесь оно меряется.

Почему НЕ через MAVROS: меряем поведение ПОЛЁТНИКА, и лишний слой (маппинг
OverrideRCIn, QoS, своя нумерация) только добавил бы подозреваемых. Тот же
скрипт годится на реальном борту — там mavlink-router тоже раздаёт UDP.

⚠️ sysid: ArduCopter молча дропает override, если msg.sysid != MAV_GCS_SYSID
(в 4.8 параметр переименован из SYSID_MYGCS; в симе выставлен 1 = как у MAVROS,
см. docker/sim/config/sitl-extra.parm). Скрипт СПРАШИВАЕТ параметр у борта и
представляется этим sysid — иначе замер показал бы ложное «ноль не работает».

Фазы (борт ДИЗАРМИРОВАН, на земле, лётная нода не запущена — проверяется):
  A (3 с): ch1..4 = 1600             → RC_CHANNELS обязан показать 1600
  B (4 с): ch1..4 = 0     (RELEASE)  → обязан СМЕНИТЬСЯ, и быстро
  C (3 с): ch1..4 = 1700             → канал берётся заново
  D (6 с): ch1..4 = 65535 (NOCHANGE) → 1700 держится / истекает по RC_OVERRIDE_TIME

Вердикт по задержке фазы B: < 0.5 с — ноль отпускает (спека подтверждена);
≈ RC_OVERRIDE_TIME — не release, а истечение; не сменилось — ноль игнорируется.
Фаза D — контраст: совпали задержки B и D → ноль неотличим от «не трогать».

Запуск:  bash src/lab/rc_release_check.sh          (обёртка, атомарно)
         python3 src/lab/rc_release_check.py [--url udpin:127.0.0.1:14541]
"""
import argparse
import sys
import time

from pymavlink import mavutil

RC_RELEASE = 0
RC_NOCHANGE = 65535
PHASES = [('A', 'override 1600', 1600, 3.0),
          ('B', 'RELEASE (0)', RC_RELEASE, 4.0),
          ('C', 'override 1700', 1700, 3.0),
          ('D', 'NOCHANGE (65535)', RC_NOCHANGE, 6.0)]


def rc_in(m, timeout=0.0):
    """Последний RC_CHANNELS → [ch1..ch4] или None."""
    msg = m.recv_match(type='RC_CHANNELS', blocking=timeout > 0, timeout=timeout or None)
    if msg is None:
        return None
    return [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]


def drain(m, sec, state):
    """Крутить приём sec секунд, обновляя state['rc'] и state['armed']."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        msg = m.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'RC_CHANNELS':
            state['rc'] = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]
        elif t == 'HEARTBEAT' and msg.get_srcComponent() == 1:
            state['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def get_param(m, names, timeout=3.0):
    """Первый существующий параметр из списка имён → (имя, значение) или (None, None)."""
    for name in names:
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg and msg.param_id.strip('\x00') == name:
                return name, msg.param_value
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='udpin:127.0.0.1:14541',
                    help='эндпоинт mavlink-router (сим: 14541 «кастомный узел»)')
    ap.add_argument('--sysid', type=int, default=None,
                    help='чем представляться (по умолчанию спрашиваем MAV_GCS_SYSID у борта)')
    args = ap.parse_args()

    print("== СЕМАНТИКА НУЛЯ В RC_CHANNELS_OVERRIDE ==")
    m = mavutil.mavlink_connection(args.url, source_system=255, source_component=190)
    print(f"  жду HEARTBEAT на {args.url} …")
    if m.wait_heartbeat(timeout=20) is None:
        print("ОШИБКА: борта на этом эндпоинте нет (mavlink-router поднят? SITL жив?)")
        return 2
    print(f"  борт: sysid={m.target_system} comp={m.target_component}")

    # sysid, от которого борт примет override
    name, val = get_param(m, ['MAV_GCS_SYSID', 'SYSID_MYGCS'])
    gcs = args.sysid if args.sysid is not None else (int(val) if val is not None else 255)
    print(f"  {name or 'MAV_GCS_SYSID/SYSID_MYGCS'} = "
          f"{'?' if val is None else int(val)} → представляюсь sysid={gcs}")
    m.mav.srcSystem = gcs

    # RC_CHANNELS с гарантированным темпом (20 Гц): иначе рискуем мерить стрим-рейт
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 50000,
                            0, 0, 0, 0, 0)

    state = {'rc': None, 'armed': None}
    drain(m, 3.0, state)
    if state['rc'] is None:
        print("ОШИБКА: RC_CHANNELS не идёт — мерить нечем (стрим-рейт/версия прошивки).")
        return 2
    if state['armed']:
        print("ОТКАЗ: борт ARMED. Проверка только на земле и дизармированным.")
        return 2
    print(f"  фон (без override): RC_CHANNELS ch1..4 = {state['rc']}, armed={state['armed']}")

    res = {}
    for nm, what, val_, dur in PHASES:
        before = list(state['rc'])
        t0 = time.monotonic()
        changed_at = None
        while time.monotonic() - t0 < dur:
            m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                            val_, val_, val_, val_,
                                            RC_NOCHANGE, RC_NOCHANGE,
                                            RC_NOCHANGE, RC_NOCHANGE)
            drain(m, 0.05, state)
            if changed_at is None and state['rc'] != before:
                changed_at = time.monotonic() - t0
        res[nm] = (before, list(state['rc']), changed_at)
        lat = f"{changed_at:.2f} с" if changed_at is not None else "НЕ СМЕНИЛОСЬ"
        print(f"  фаза {nm} [{what:<18}] {dur:.0f} с: {before} → {state['rc']}"
              f"   смена через {lat}")

    print("\n== ВЕРДИКТ ==")
    a_after = res['A'][1]
    took = all(v == 1600 for v in a_after)
    print(f"  A override берётся:     {'ДА' if took else 'НЕТ — sysid не тот?'} ({a_after})")
    b_after, b_lat = res['B'][1], res['B'][2]
    d_lat = res['D'][2]
    released = b_lat is not None and all(v != 1600 for v in b_after)
    if not released:
        b_verdict = "НОЛЬ НЕ ОТПУСКАЕТ — сторож не выводит ноду из цепочки"
    elif b_lat < 0.5:
        b_verdict = f"НОЛЬ = ОТПУСТИТЬ (задержка {b_lat:.2f} с) — спека подтверждена"
    else:
        b_verdict = (f"отпустило через {b_lat:.2f} с — похоже на ИСТЕЧЕНИЕ по "
                     f"RC_OVERRIDE_TIME, а не release")
    print(f"  B ноль (RC_RELEASE):    {b_verdict}")
    print(f"    после отпускания: {b_after} (это уже не наша команда)")
    c_after = res['C'][1]
    retook = all(v == 1700 for v in c_after)
    print(f"  C канал берётся снова:  {'ДА' if retook else 'НЕТ'} ({c_after})")
    if d_lat is None:
        print(f"  D 65535 (NOCHANGE):     держался всю фазу (> {PHASES[3][3]:.0f} с)")
    else:
        print(f"  D 65535 (NOCHANGE):     отпустило через {d_lat:.2f} с (истечение)")
    same = b_lat is not None and d_lat is not None and abs(b_lat - d_lat) < 0.3
    if same:
        print("  ⚠️ B и D совпали по задержке: ноль неотличим от «не трогать»")
    ok = took and released and retook and b_lat < 0.5 and not same
    print("\nИТОГ:", "✅ СТОРОЖ ЧЕСТЕН — ноль отпускает канал немедленно" if ok
          else "⚠️ НЕ ТО ПОВЕДЕНИЕ, на которое рассчитан сторож — смотреть выше")

    for _ in range(10):                 # не оставлять override за собой
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        *([RC_RELEASE] * 4 + [RC_NOCHANGE] * 4))
        time.sleep(0.05)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
