#!/usr/bin/env python3
"""
ALT_HOLD bootstrap: взлёт без GPS и инициализация VINS в полёте.

Зачем: GUIDED — позиционный режим, без GPS / сошедшегося VINS он даже не латчится.
ALT_HOLD держит высоту по барометру и НЕ требует горизонтальной позиции, поэтому им
можно оторваться от земли и СОЗДАТЬ движение (climb + раскачка), нужное монокуляр-
ному VINS для инициализации (масштаб/гравитация ненаблюдаемы в покое). Контекст и
обоснование: src/nav/FAQ_gps.md, план — src/nav/todo.txt.

Особенность ALT_HOLD: высота управляется throttle-стиком (пружинный, ЦЕНТР = hold):
газ вверх → подъём, отпустил в центр → держит высоту. Авто-взлёта, как в GUIDED
(cmd/takeoff), тут нет → нужен НЕПРЕРЫВНЫЙ RC override (/mavros/rc/override, ~20 Гц).
Поэтому это НОДА, а не bash: она обязана публиковать override весь полёт в ALT_HOLD,
иначе FCU по таймауту вернётся к своему RC и дрон просядет. Завершить ALT_HOLD-фазу
можно только переходом в самоподдерживающийся режим, которому override не нужен:
GUIDED (--handover) либо LAND. «Просто зависнуть в ALT_HOLD и выйти» — небезопасно.

Конечный автомат (бюджеты — в SIM-времени по /clock, RTF-независимо):
  PREARM (ALT_HOLD, throttle=min) → ARM → CLIMB (throttle>центр до --alt)
    → EXCITE (throttle=центр + импульсы roll/pitch) — ждём сходимости VINS
    → [--handover] GUIDED (наблюдаем рывок)  |  иначе: OBSERVE → LAND (самодостаточно)

--handover ВЫКЛ по умолчанию: выравнивание кадра VINS→NED (yaw-коррекция в
ray_tracer) ещё не реализовано, поэтому переход в GUIDED даст рывок (расхождение
нуля VINS и датума). Сначала смотрим саму инициализацию (climb→init→land без рывка),
handover — отдельным прогоном.

Запуск внутри nav:  python3 /lab/alt_hold_bootstrap.py [--alt 3] [--handover] ...
Через секвенсор:    src/lab/bootstrap.sh  (команда `bootstrap` в capture_scene.sh).
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from mavros_msgs.msg import OverrideRCIn, State, RCIn
from mavros_msgs.srv import SetMode, CommandBool

# --- RC (PWM) ---------------------------------------------------------------
RC_CENTER = 1500       # центр стика; для throttle в ALT_HOLD центр = «держать высоту»
RC_MIN_THR = 1000      # газ в минимум — нужно для арминга в ALT_HOLD
# Каналы 1..4 = roll/pitch/throttle/yaw (ArduCopter). 5..18 — не трогаем:
# 65535 (UINT16_MAX) = «ИГНОРИРОВАТЬ канал», не отдаём его радио и не оверрайдим.
# За удержание РЕЖИМА отвечает не этот канал, а SITL-параметр FLTMODE_CH 0
# (sitl-extra.parm): он убирает RC-переключатель режима, чтобы радио не перебивало
# set_mode("ALT_HOLD"). Подробности — там и в hold_alt_hold() ниже.
RC_NOCHANGE = 65535

# Состояния автомата
S_PREARM, S_ARM, S_CLIMB, S_EXCITE, S_HANDOVER, S_OBSERVE, S_LAND, S_DONE = range(8)
S_NAME = {S_PREARM: "PREARM", S_ARM: "ARM", S_CLIMB: "CLIMB", S_EXCITE: "EXCITE",
          S_HANDOVER: "HANDOVER", S_OBSERVE: "OBSERVE", S_LAND: "LAND", S_DONE: "DONE"}


class AltHoldBootstrap(Node):
    def __init__(self, a):
        super().__init__('alt_hold_bootstrap')
        # use_sim_time: все бюджеты/таймеры считаем по /clock (sim-время). На низком
        # RTF (CPU-бокс ~0.07) wall-секунды = мизер sim-времени — climb/excite не
        # успели бы. Как в fly_square.py / arm.sh (бюджет в sim-сек).
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a = a

        # выходы
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        # входы
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        # rel_alt и rc/in MAVROS публикует с SensorData QoS (BEST_EFFORT) — дефолтная
        # RELIABLE-подписка их НЕ получает («incompatible QoS, no messages», 1-й
        # прогон: rel_alt=None → нода летела вслепую, не выходила из CLIMB).
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self._on_relalt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/vins_estimator/odometry', self._on_odom, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self._on_rcin, qos_profile_sensor_data)
        # сервисы
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        # состояние
        self.mode = None
        self.armed = False
        self.rel_alt = None
        self.odom_count = 0
        self.last_odom_sim = -1e9
        self.rcin_thr = None
        self.rcin_logged = False

        self.state = S_PREARM
        self.state_t0 = None          # базируется лениво при первом тике с живым /clock
        self.last_cmd = -1e9          # троттлинг вызовов сервисов (раз в ~1 sim-сек)
        self.last_mode_assert = -1e9  # отдельный троттл для ре-ассерта ALT_HOLD
        self.result = "?"
        self.finished = False

        # текущая команда RC (обновляется в каждом состоянии)
        self.roll = RC_CENTER
        self.pitch = RC_CENTER
        self.throttle = RC_MIN_THR
        self.yaw = RC_CENTER

        # Автомат — на sim-времени (ROSClock/use_sim_time): бюджеты фаз RTF-
        # независимы (как arm.sh/fly_square). 20 Гц sim ≈ 1.4 Гц wall при RTF 0.07.
        self.timer = self.create_timer(0.05, self.tick)   # автомат (sim-время)
        # RC override публикуется НЕ таймером, а из главного цикла по time.monotonic()
        # (см. main): нужен СТАБИЛЬНЫЙ wall-rate ~20 Гц. FCU считает свежесть override
        # в своём (wall) времени; на sim-таймере при RTF 0.07 вышло бы ≈1.4 Гц wall →
        # override протухал бы. Отдельный STEADY_TIME-таймер rclpy под spin_once с
        # use_sim_time оказался ненадёжен (мог не тикать → нода не публиковала вообще,
        # RCIN оставался 1000) — поэтому публикуем явно в цикле.
        self.get_logger().info(
            f"alt_hold_bootstrap: alt={a.alt}м handover={a.handover} "
            f"excite=±{a.excite}PWM/{a.excite_period}s vins_timeout={a.vins_timeout}s (sim)")

    # --- утилиты -------------------------------------------------------------
    def now_sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        # Ленивое базирование: первый тик с живым /clock задаёт точку отсчёта
        # состояния. Иначе now_sim()=0 до прихода /clock, а затем скачок к реальному
        # sim-времени дал бы ложное превышение бюджета (мгновенный аборт).
        if self.state_t0 is None:
            self.state_t0 = self.now_sim()
            return 0.0
        return self.now_sim() - self.state_t0

    def goto(self, st):
        self.get_logger().info(f">>> {S_NAME[self.state]} → {S_NAME[st]}")
        self.state = st
        self.state_t0 = None          # ре-базируется на следующем тике
        self.last_cmd = -1e9

    def vins_converged(self):
        # VINS публикует /odometry только ПОСЛЕ инициализации (solver NON_LINEAR),
        # поэтому устойчивый поток = сходимость. Требуем N сообщений + свежесть.
        return (self.odom_count >= self.a.vins_min and
                (self.now_sim() - self.last_odom_sim) < 2.0)

    def _try(self, fn):  # троттлим вызов сервиса до ~1 раза в sim-секунду
        if self.now_sim() - self.last_cmd >= 1.0:
            self.last_cmd = self.now_sim()
            fn()

    def set_mode(self, mode):
        if self.mode_cli.service_is_ready():
            req = SetMode.Request(); req.custom_mode = mode
            self.mode_cli.call_async(req)

    def hold_alt_hold(self):
        # Защитный ре-ассерт: на фазах ARM/CLIMB/EXCITE режим обязан быть ALT_HOLD.
        # Основной механизм удержания — FLTMODE_CH 0 в SITL (радио не трогает режим);
        # это страховка на случай смены режима полётником (EKF/failsafe). Отдельный
        # троттл (~раз в 2 sim-сек), чтобы не конфликтовать с throttle вызовов arm().
        # '' — транзиент /mavros/state до первого валидного heartbeat (не режим).
        if self.mode not in (None, "", "ALT_HOLD") and \
                self.now_sim() - self.last_mode_assert >= 2.0:
            self.last_mode_assert = self.now_sim()
            self.get_logger().warn(f"режим={self.mode} ≠ ALT_HOLD — ре-ассерт")
            self.set_mode("ALT_HOLD")

    def arm(self):
        if self.arm_cli.service_is_ready():
            req = CommandBool.Request(); req.value = True
            self.arm_cli.call_async(req)

    # --- входы ---------------------------------------------------------------
    def _on_state(self, m): self.mode = m.mode; self.armed = m.armed
    def _on_relalt(self, m): self.rel_alt = float(m.data)
    def _on_odom(self, m): self.odom_count += 1; self.last_odom_sim = self.now_sim()

    def _on_rcin(self, m):
        if len(m.channels) >= 3:
            self.rcin_thr = m.channels[2]

    # --- публикация RC override (WALL-таймер 20 Гц) -------------------------
    def _wall_publish(self):
        # Гейт публикации: в DONE и в GUIDED (после handover) override НЕ нужен —
        # GUIDED держит позицию сам. До этого — держим override свежим на wall-частоте.
        if self.state == S_DONE or (self.state == S_HANDOVER and self.mode == "GUIDED"):
            return
        msg = OverrideRCIn()
        ch = [RC_NOCHANGE] * 18
        ch[0] = int(self.roll)
        ch[1] = int(self.pitch)
        ch[2] = int(self.throttle)
        ch[3] = int(self.yaw)
        msg.channels = ch
        self.rc_pub.publish(msg)

    # --- автомат -------------------------------------------------------------
    def tick(self):
        st = self.state

        if st == S_PREARM:
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = RC_MIN_THR            # газ в минимум для арминга
            self._try(lambda: self.set_mode("ALT_HOLD"))
            if self.mode == "ALT_HOLD":
                self.goto(S_ARM)
            elif self.elapsed() > self.a.mode_budget:
                self.get_logger().warn(f"⚠️ ALT_HOLD не залатчился (mode={self.mode}) — пробуем дальше")
                self.goto(S_ARM)

        elif st == S_ARM:
            self.throttle = RC_MIN_THR
            self.hold_alt_hold()
            self._try(self.arm)
            if self.armed:
                self.goto(S_CLIMB)
            elif self.elapsed() > self.a.arm_budget:
                self.get_logger().error(f"⚠️ арм не прошёл (armed={self.armed}) — аборт")
                self.result = "ARM_FAIL"; self.goto(S_DONE)

        elif st == S_CLIMB:
            self.throttle = self.a.throttle_climb     # газ вверх → подъём
            self.hold_alt_hold()
            if not self.rcin_logged and self.rcin_thr is not None and self.elapsed() > 2:
                self.get_logger().info(f"    rc/in throttle={self.rcin_thr} (override проходит, если ≈{self.a.throttle_climb})")
                self.rcin_logged = True
            if self.rel_alt is not None and self.rel_alt >= self.a.alt:
                self.get_logger().info(f"    набрали {self.rel_alt:.1f}м (цель {self.a.alt}м)")
                self.goto(S_EXCITE)
            elif self.elapsed() > self.a.climb_budget:
                if self.rel_alt is not None and self.rel_alt >= 0.5:
                    self.get_logger().warn(f"⚠️ climb-бюджет вышел, высота {self.rel_alt:.1f}м — раскачиваем как есть")
                    self.goto(S_EXCITE)
                else:
                    self.get_logger().error(f"⚠️ не взлетели (rel_alt={self.rel_alt}) — RC override не принят? аборт→LAND")
                    self.result = "CLIMB_FAIL"; self.goto(S_LAND)

        elif st == S_EXCITE:
            # throttle=центр (держим высоту), импульсы roll/pitch для параллакса +
            # IMU excitation. Чередуем 4 направления (вперёд/назад/влево/вправо) —
            # суммарный дрейф ~около нуля, но движение реальное.
            self.throttle = self.a.throttle_hold
            self.hold_alt_hold()
            amp = self.a.excite
            phase = int(self.elapsed() / self.a.excite_period) % 4
            offs = [(0, -amp), (0, +amp), (-amp, 0), (+amp, 0)][phase]
            self.roll = RC_CENTER + offs[0]
            self.pitch = RC_CENTER + offs[1]
            if self.vins_converged():
                self.get_logger().info(f"    ✅ VINS сошёлся ({self.odom_count} odom-сообщений)")
                self.result = "VINS_OK"
                self.goto(S_HANDOVER if self.a.handover else S_OBSERVE)
            elif self.elapsed() > self.a.vins_timeout:
                self.get_logger().warn(f"⚠️ VINS не сошёлся за {self.a.vins_timeout}s "
                                       f"({self.odom_count} odom) — садимся")
                self.result = "VINS_TIMEOUT"; self.goto(S_LAND)

        elif st == S_HANDOVER:
            # Переход в GUIDED: он самоудерживает позицию, override больше не нужен.
            # ⚠️ Здесь и проявится рывок (кадр VINS не выровнен к NED) — это и смотрим.
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = self.a.throttle_hold
            self._try(lambda: self.set_mode("GUIDED"))
            if self.mode == "GUIDED":
                self.get_logger().info("    в GUIDED — дрон удерживает позицию по ExternalNav (наблюдаем рывок)")
                self.result += "+GUIDED"; self.goto(S_DONE)
            elif self.elapsed() > self.a.mode_budget:
                self.get_logger().warn("⚠️ GUIDED не залатчился — садимся")
                self.goto(S_LAND)

        elif st == S_OBSERVE:
            # Чистое наблюдение init: держим высоту (центр), без раскачки, --observe
            # sim-секунд, затем садимся. Самодостаточный безопасный манёвр.
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = self.a.throttle_hold
            if self.elapsed() > self.a.observe:
                self.goto(S_LAND)

        elif st == S_LAND:
            # LAND сам снижает и игнорирует throttle → override можно отпустить.
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = self.a.throttle_hold
            self._try(lambda: self.set_mode("LAND"))
            touched = (self.rel_alt is not None and self.rel_alt <= self.a.ground_z)
            if touched or (self.mode == "LAND" and not self.armed and self.elapsed() > 3):
                self.get_logger().info(f"    касание (rel_alt={self.rel_alt}, armed={self.armed})")
                self.goto(S_DONE)
            elif self.elapsed() > self.a.land_budget:
                self.get_logger().warn(f"⚠️ касание не подтверждено (rel_alt={self.rel_alt}) — выходим")
                self.goto(S_DONE)

        elif st == S_DONE:
            self.get_logger().info(f">>> ИТОГ: {self.result} (mode={self.mode}, armed={self.armed}, "
                                   f"rel_alt={self.rel_alt}, odom={self.odom_count})")
            self.finished = True
            return
        # RC override публикует отдельный WALL-таймер (_wall_publish), не tick:
        # на низком RTF sim-таймер дал бы ~1.4 Гц wall и override протух бы. tick
        # лишь обновляет self.roll/pitch/throttle/yaw — их шлёт wall-таймер 20 Гц.


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--alt', type=float, default=3.0, help='целевая высота climb, м (default 3)')
    p.add_argument('--handover', action='store_true',
                   help='после сходимости VINS перейти в GUIDED (увидеть рывок); по умолчанию OFF → OBSERVE+LAND')
    p.add_argument('--excite', type=int, default=80, help='амплитуда импульсов roll/pitch от центра, PWM (default 80)')
    p.add_argument('--excite-period', dest='excite_period', type=float, default=3.0,
                   help='длительность одного направления раскачки, sim-сек (default 3)')
    p.add_argument('--vins-timeout', dest='vins_timeout', type=float, default=90.0,
                   help='сколько ждать сходимости VINS в EXCITE, sim-сек (default 90)')
    p.add_argument('--vins-min', dest='vins_min', type=int, default=40,
                   help='сколько odom-сообщений считать сходимостью (default 40)')
    p.add_argument('--observe', type=float, default=15.0,
                   help='держать высоту после init перед посадкой (без handover), sim-сек (default 15)')
    p.add_argument('--throttle-climb', dest='throttle_climb', type=int, default=1650,
                   help='PWM газа на подъём (default 1650)')
    p.add_argument('--throttle-hold', dest='throttle_hold', type=int, default=RC_CENTER,
                   help='PWM газа на удержание = центр (default 1500)')
    p.add_argument('--mode-budget', dest='mode_budget', type=float, default=40.0, help='бюджет латча режима, sim-сек')
    p.add_argument('--arm-budget', dest='arm_budget', type=float, default=40.0, help='бюджет арминга, sim-сек')
    p.add_argument('--climb-budget', dest='climb_budget', type=float, default=60.0, help='бюджет набора высоты, sim-сек')
    p.add_argument('--land-budget', dest='land_budget', type=float, default=120.0, help='бюджет посадки, sim-сек')
    p.add_argument('--ground-z', dest='ground_z', type=float, default=0.3, help='порог касания по rel_alt, м')
    args = p.parse_args()

    rclpy.init()
    node = AltHoldBootstrap(args)
    try:
        # RC override публикуем ИЗ ЦИКЛА по wall-часам (time.monotonic) ~20 Гц —
        # гарантированный реальный темп независимо от RTF и семантики таймеров rclpy.
        # spin_once крутит автомат (sim-таймер) и колбэки подписок.
        last_pub = 0.0
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_pub >= 0.05:
                last_pub = now
                node._wall_publish()
    except KeyboardInterrupt:
        node.get_logger().info("Прервано — садимся вручную (make land).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
