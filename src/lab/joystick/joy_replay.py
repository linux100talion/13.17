#!/usr/bin/env python3
"""joy_replay.py — «виртуальный пилот» freefly: публикует /joy по сценарию,
неотличимо от живого TX12 (joy_linux_node). Нода миссии запускается с --pilot joy
(BS_PILOT=replay в bootstrap_arch2.sh делает подмену сам) и видит обычный
JoyPilot-вход — весь стек ниже /joy идентичен ручному полёту.

Два режима:
  --scenario f.json  — СЕМАНТИЧЕСКИЙ сценарий (основной): шаги со смысловыми
                       значениями стиков + замкнутые якоря (arm/disarm по
                       /mavros/state, wait_alt по ground-truth Gazebo,
                       wait_mode по режиму FCU). Формат — README.md рядом.
  --raw f.jsonl      — сырой таймлайн /joy из joy_timeline.py (ZOH по
                       sim-времени) — только валидация канала реплея;
                       разомкнут, траекторию не повторит (ветер, тайминг EKF).

СЕМАНТИКА СТИКОВ сценария — PWM-конвенция, v = (PWM−1500)/400:
  roll: +1 = вправо (1900); pitch: −1 = вперёд/«от себя» (1300), +1 = назад;
  thr:  −1 = газ min (1100), +1 = max (1900);  yaw: +1 = вправо (1900).
  sw (CH6): −1 = наш стек (тумблер ВВЕРХ), 0 = центр (ALT_HOLD; при
  BS_FF_LOITER=1 — штатный LOITER-на-VINS), +1 = MANUAL (ВНИЗ).
В /joy уходят СЫРЫЕ оси: raw = v * sign. Знаки (--signs) обязаны совпадать с
BS_JOY_SIGNS ноды (дефолт — JOY_SIGNS_DEFAULT из control_pkg, единый источник).

Время — sim (/clock, use_sim_time): hold/таймауты RTF-независимы.
/mavros/state идёт 1 Гц → якоря arm/wait_mode дискретны ±1 с (для простых
полётов достаточно). Высота якорей — ground truth /model/iris_cam/odometry
(стендовый оракул, жив в GPS-denied; rel_alt без GPS замерзает — см. baro_alt.py).

Таймаут ЛЮБОГО якоря → аварийная посадка (thr=−0.5 до земли по gt → газ min →
disarm-жест) — стенд без человека не зависает в воздухе. Конец сценария —
держим последнее состояние (bootstrap_arch2.sh убьёт процесс по trap на выходе).

Запуск руками (обычно не нужен — BS_PILOT=replay делает всё сам):
  python3 /lab/joystick/joy_replay.py --scenario /lab/joystick/scenarios/x.json
"""
import argparse
import json
import sys

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

AXES = ('roll', 'pitch', 'thr', 'yaw')          # семантические оси = /joy 0..3
STEP_KEYS = {'note', 'sticks', 'sw', 'hold', 'arm', 'disarm',
             'wait_alt', 'wait_mode', 'ramp'}
ALT_BASELINE_N = 40      # сэмплов gt-z на базлайн «земли» (нода стартует до арма)
# Аварийная посадка при таймауте якоря: умеренное снижение → газ min → дизарм.
ABORT_STEPS = [
    {'note': 'АВАРИЙНАЯ ПОСАДКА (таймаут якоря)'},
    {'sticks': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'thr': -0.5}},
    {'wait_alt': {'lte': 0.2, 'timeout': 120}},
    {'sticks': {'thr': -1.0}},
    {'hold': 2.0},
    {'disarm': {'timeout': 30}},
]


def default_signs():
    """JOY_SIGNS_DEFAULT из control_pkg — единый источник правды о знаках."""
    try:
        from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT
        return JOY_SIGNS_DEFAULT
    except ImportError:
        sys.path.insert(0, '/root/sim_ws/src/control')
        from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT
        return JOY_SIGNS_DEFAULT


def load_scenario(path):
    with open(path) as f:
        scn = json.load(f)
    for i, s in enumerate(scn.get('steps', [])):
        bad = set(s) - STEP_KEYS
        if bad:
            sys.exit(f"ОШИБКА: шаг {i} — неизвестные ключи {sorted(bad)} "
                     f"(допустимы: {sorted(STEP_KEYS)})")
        if len(set(s) - {'note'}) > 1:
            sys.exit(f"ОШИБКА: шаг {i} — больше одного действия: {sorted(s)}")
    return scn


def load_raw(path):
    """jsonl из joy_timeline.py: {"t": сек от старта, "axes": [сырые /joy]}."""
    ts, axes = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            ts.append(float(d['t']))
            axes.append([float(a) for a in d['axes']])
    if not ts:
        sys.exit(f"ОШИБКА: {path} пуст")
    t0 = ts[0]
    return [t - t0 for t in ts], axes


class JoyReplay(Node):
    def __init__(self, args):
        super().__init__('joy_replay', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._signs = args.signs
        self._pub = self.create_publisher(Joy, '/joy', qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.create_subscription(Odometry, args.alt_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self._armed = None          # None = /mavros/state ещё не видели
        self._mode = ''
        self._alt = None            # м над точкой старта (gt)
        self._z_warm = []
        self._z0 = None

        self._raw = None
        if args.raw:
            self._raw = load_raw(args.raw)
            self._raw_i = 0
            self.get_logger().info(
                f"RAW-реплей {args.raw}: {len(self._raw[0])} сэмплов, "
                f"{self._raw[0][-1]:.1f} sim-с")
        else:
            scn = load_scenario(args.scenario)
            self._steps = list(scn.get('steps', []))
            init = scn.get('init', {})
            self._sem = {'roll': 0.0, 'pitch': 0.0, 'thr': -1.0, 'yaw': 0.0}
            self._sem.update({k: float(v)
                              for k, v in init.get('sticks', {}).items()})
            self._sw = int(init.get('sw', -1))
            self.get_logger().info(
                f"сценарий {args.scenario}: «{scn.get('name', '?')}», "
                f"{len(self._steps)} шагов, init sticks={self._sem} sw={self._sw}")
        self._i = 0
        self._step_t0 = None        # sim-время входа в текущий шаг
        self._ramp_from = None
        self._t0 = None             # sim-время первого тика
        self._aborting = False
        self._done = False
        # период таймера — sim-время (use_sim_time): цадence консистентен со
        # всем стеком; при низком RTF /joy реже в wall, но так же част в sim.
        self.create_timer(1.0 / args.rate, self._tick)

    # ---------- подписки ----------
    def _on_state(self, m):
        self._armed, self._mode = bool(m.armed), m.mode

    def _on_odom(self, m):
        z = m.pose.pose.position.z
        if self._z0 is None:
            self._z_warm.append(z)
            if len(self._z_warm) >= ALT_BASELINE_N:
                s = sorted(self._z_warm)
                self._z0 = s[len(s) // 2]
                self._z_warm = None
            return
        self._alt = z - self._z0

    # ---------- публикация ----------
    def _publish(self, raw6=None):
        if raw6 is None:
            raw6 = [self._sem[a] * self._signs[i] for i, a in enumerate(AXES)]
            raw6 += [0.0, float(self._sw)]      # axes[4]=CH5 (не трогаем), [5]=CH6
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = [float(v) for v in (list(raw6) + [0.0] * 8)[:8]]
        m.buttons = []
        self._pub.publish(m)

    # ---------- главный цикл ----------
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:                          # /clock ещё не пошёл
            return
        if self._t0 is None:
            self._t0 = now
            self.get_logger().info(f"старт (sim t={now:.1f})")
        if self._raw is not None:
            ts, axes = self._raw
            el = now - self._t0
            while self._raw_i + 1 < len(ts) and ts[self._raw_i + 1] <= el:
                self._raw_i += 1
            self._publish(axes[self._raw_i])
            return
        # семантический сценарий: мгновенные шаги схлопываются в один тик
        for _ in range(64):
            if self._i >= len(self._steps):
                if not self._done:
                    self._done = True
                    self.get_logger().info(
                        "сценарий завершён — держу последнее состояние"
                        + (" (был ABORT)" if self._aborting else ""))
                break
            if self._process(now, self._steps[self._i]):
                self._i += 1
                self._step_t0 = None
                self._ramp_from = None
                continue
            break
        self._publish()

    def _enter(self, now, step):
        """Первый тик шага: True + лог."""
        if self._step_t0 is not None:
            return False
        self._step_t0 = now
        note = step.get('note')
        act = {k: v for k, v in step.items() if k != 'note'}
        self.get_logger().info(
            f"[{now - self._t0:7.1f}с] шаг {self._i}: {act or 'note'}"
            + (f" — {note}" if note else ""))
        return True

    def _timeout(self, now, step, default):
        for v in step.values():
            if isinstance(v, dict) and 'timeout' in v:
                default = float(v['timeout'])
        return (now - self._step_t0) > default

    def _abort(self, why):
        if self._aborting:              # таймаут внутри аварийной посадки — дальше
            self.get_logger().error(f"ABORT: {why} — пропускаю шаг")
            return True
        self.get_logger().error(f"ТАЙМАУТ ЯКОРЯ: {why} — аварийная посадка")
        self._aborting = True
        self._steps = list(ABORT_STEPS)
        self._i = -1                    # +1 в _tick → 0
        return True

    def _process(self, now, step):
        """True = шаг завершён (перейти к следующему)."""
        self._enter(now, step)
        if 'sticks' in step:
            self._sem.update({k: float(v) for k, v in step['sticks'].items()})
            return True
        if 'sw' in step:
            self._sw = int(step['sw'])
            return True
        if set(step) == {'note'}:
            return True
        if 'hold' in step:
            return (now - self._step_t0) >= float(step['hold'])
        if 'ramp' in step:              # линейный ход осей за dur sim-с
            tgt = {k: float(v) for k, v in step['ramp']['sticks'].items()}
            dur = float(step['ramp'].get('dur', 2.0))
            if self._ramp_from is None:
                self._ramp_from = {k: self._sem[k] for k in tgt}
            f = min(1.0, (now - self._step_t0) / max(dur, 1e-6))
            for k, v in tgt.items():
                self._sem[k] = self._ramp_from[k] + (v - self._ramp_from[k]) * f
            return f >= 1.0
        if 'arm' in step:               # руддер-арм: газ min + yaw вправо
            self._sem.update({'thr': -1.0, 'yaw': 1.0})
            if self._armed:
                self._sem['yaw'] = 0.0
                self.get_logger().info("    ARMED — жест снят")
                return True
            return self._abort('arm') if self._timeout(now, step, 120) else False
        if 'disarm' in step:            # газ min + yaw влево
            self._sem.update({'thr': -1.0, 'yaw': -1.0})
            if self._armed is False:
                self._sem['yaw'] = 0.0
                self.get_logger().info("    DISARMED — жест снят")
                return True
            return self._abort('disarm') if self._timeout(now, step, 30) else False
        if 'wait_alt' in step:
            w = step['wait_alt']
            if self._alt is not None:
                if 'gte' in w and self._alt >= float(w['gte']):
                    self.get_logger().info(f"    alt={self._alt:.2f}м ≥ {w['gte']}")
                    return True
                if 'lte' in w and self._alt <= float(w['lte']):
                    self.get_logger().info(f"    alt={self._alt:.2f}м ≤ {w['lte']}")
                    return True
            return (self._abort(f"wait_alt {w} (alt={self._alt})")
                    if self._timeout(now, step, 60) else False)
        if 'wait_mode' in step:
            want = step['wait_mode']['is']
            if self._mode == want:
                self.get_logger().info(f"    режим {want}")
                return True
            return (self._abort(f"wait_mode {want} (mode={self._mode!r})")
                    if self._timeout(now, step, 30) else False)
        return True                     # не должно достигаться (валидация в load)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--scenario', help='семантический сценарий .json')
    g.add_argument('--raw', help='сырой таймлайн .jsonl из joy_timeline.py')
    ap.add_argument('--signs', default='',
                    help='знаки осей "r,p,t,y" (= BS_JOY_SIGNS ноды; '
                         'дефолт — JOY_SIGNS_DEFAULT из control_pkg)')
    ap.add_argument('--rate', type=float, default=50.0,
                    help='частота публикации /joy, sim-Гц (50)')
    ap.add_argument('--alt-topic', default='/model/iris_cam/odometry',
                    help='ground-truth одометрия для wait_alt')
    args = ap.parse_args()
    args.signs = (tuple(float(x) for x in args.signs.split(','))
                  if args.signs else default_signs())
    if len(args.signs) != 4:
        sys.exit(f"ОШИБКА: --signs ожидает 4 значения, получил {args.signs}")

    rclpy.init()
    node = JoyReplay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
