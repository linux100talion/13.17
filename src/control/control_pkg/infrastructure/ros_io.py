#!/usr/bin/env python3
"""RosLogger + RosDebugSink — мелкие адаптеры портов Logger и DebugSink.

DebugSink → /flow_dbg (Vector3Stamped): sim-штампованный roll_off/flow_lateral/conf для
system-ID (flow_calib.py). Vector3Stamped, т.к. OverrideRCIn без header → под низким
RTF не привязать к sim-времени. publish_axes() шлёт заодно /flow_dbg2
(pitch_off/flow_longitudinal/flow_yaw) — полный флоу-дамп для свипа/диагностики.
"""
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String

from ..domain.rc import RC_CENTER
from ..perception.flow_estimator import ipm_dbg_z


class RosLogger:
    def __init__(self, node):
        self._log = node.get_logger()

    def info(self, m: str) -> None:
        self._log.info(m)

    def warn(self, m: str) -> None:
        self._log.warn(m)

    def error(self, m: str) -> None:
        self._log.error(m)


class RosDebugSink:
    def __init__(self, node):
        self._node = node
        self._pub = node.create_publisher(Vector3Stamped, '/flow_dbg', 10)
        self._pub2 = node.create_publisher(Vector3Stamped, '/flow_dbg2', 10)
        self._pub3 = node.create_publisher(Vector3Stamped, '/flow_dbg3', 10)
        self._pub4 = node.create_publisher(Vector3Stamped, '/flow_dbg4', 10)
        self._pub5 = node.create_publisher(Vector3Stamped, '/flow_dbg5', 10)
        self._pub8 = node.create_publisher(Vector3Stamped, '/flow_dbg8', 10)
        self._pub7 = node.create_publisher(Vector3Stamped, '/flow_dbg7', 10)
        # /flow_dbg9 = БОКОВАЯ скорость канала вида сверху, в метрах. Своего слота у неё
        # не было: она восстанавливалась из /flow_dbg7 как «цель − ошибка», то есть ЧЕРЕЗ
        # крен-контур. Значит прогон БЕЗ демпфера крена (stab=none, наблюдение) оставался
        # вовсе без бокового измерения — ровно та же болезнь, что съела продольную ось
        # серий J..N. Меряется она именно там: под постоянным сносом размах истины велик,
        # и наклон (масштаб) наконец отделим от сдвига.
        self._pub9 = node.create_publisher(Vector3Stamped, '/flow_dbg9', 10)
        # /flow_dbg6 = уставка КУРСА (DpYawHold в pos-режиме). Отдельный топик, а не
        # второй источник в /flow_dbg5: pos-осей стало две, и «первый ответивший»
        # молча подменил бы диагностику тангажа рысканием.
        self._pub6 = node.create_publisher(Vector3Stamped, '/flow_dbg6', 10)
        # /flow_dbg10 = ТАНГАЖ-контур по скорости (DpPitchRate): (цель, ошибка, PWM) —
        # зеркало /flow_dbg7 для крена. До него цель тангажа не писалась НИКУДА:
        # /flow_dbg5 берёт только pos-оси (DpPitchHold), а станция на тангаж (7f1a3e9)
        # живёт в rate-оси. Разбор ab_pitch (2026-08-29) остался без фаз станции —
        # брейк/возврат/гвоздь неотличимы по одному PWM из /flow_dbg2.
        self._pub10 = node.create_publisher(Vector3Stamped, '/flow_dbg10', 10)
        # /mission/status = статус гейта LOITER-на-VINS (String, "k=v k=v ...").
        # ЕДИНСТВЕННЫЙ источник правды для debug-HUD: openhd_streamer рисует
        # баннер «VINS READY» ровно из этого топика, а не из своей оценки
        # (полёт lv1_joy_20260822_232043: гейт молча держал, пилот щёлкал CH6
        # вслепую). Заодно попадает в bag → joy_timeline показывает переходы.
        self._pub_status = node.create_publisher(String, '/mission/status', 10)

    def publish(self, roll_off: float, flow_off: float, conf: float, stamp: float) -> None:
        d = Vector3Stamped()
        d.header.stamp = self._node.get_clock().now().to_msg()
        d.vector.x = float(roll_off)
        d.vector.y = float(flow_off)
        d.vector.z = float(conf)
        self._pub.publish(d)

    def publish_status(self, line: str) -> None:
        """/mission/status: строка гейта от лётной ноды (см. __init__)."""
        m = String()
        m.data = line
        self._pub_status.publish(m)

    def publish_hold(self, hold) -> None:
        """/flow_dbg5 = УСТАВКА холдера положения: (точка удержания, ошибка до неё,
        скорость уставки). Нужен, как только команда пилота начинает двигать уставку
        (DpPitchHold в режиме `pos`): из телеметрии её не восстановить — это интеграл
        команды по КАДРАМ, со стартом в захваченной точке. Без неё «борт не поехал» и
        «уставка не поехала» в разборе неразличимы. hold=None (rate-оси) — не шлём."""
        self._publish_hold(self._pub5, hold)

    def publish_rate_roll(self, dbg) -> None:
        """/flow_dbg7 = КРЕН-контур целиком: (цель по скорости, ошибка до неё, PWM).

        Крен — ось по СКОРОСТИ (`rate`), у неё нет уставки-накопителя, поэтому
        /flow_dbg5 её не берёт. Но команда всё равно должна лежать в бэге: без неё
        командный сегмент ищется по ИСТИННОЙ скорости, и откат борта после торможения
        неотличим от новой команды (замер R1: три «проезда» на две команды миссии,
        разброс калиброванного гейна 26%). dbg=None (pos-оси) — не шлём."""
        if dbg is None:
            return
        self._publish_hold(self._pub7, dbg)

    def publish_rate_pitch(self, dbg) -> None:
        """/flow_dbg10 = ТАНГАЖ-контур целиком: (цель по скорости, ошибка до неё, PWM).
        Зеркало `publish_rate_roll`: тангаж со станцией — тоже `rate`-ось, и без записи
        цели фазы станции (гвоздь/брейк/возврат) по PWM из /flow_dbg2 не восстановить.
        dbg=None (pos-ось DpPitchHold) — не шлём."""
        if dbg is None:
            return
        self._publish_hold(self._pub10, dbg)

    def publish_hold_yaw(self, hold, yaw_off: float = 0.0) -> None:
        """/flow_dbg6 = КУРС-контур целиком: (курс-уставка, ошибка до неё, PWM на выходе).

        Уставка и ошибка — в единицах визуального курса (1 ед. = 1/S градусов). Третий
        слот отдан НЕ скорости уставки (она восстанавливается производной первого), а
        PWM рыскания: команды рыскания в телеметрии не было вовсе (ToDo §9), из-за чего
        свип Y2 пришлось сегментировать по истинной ω_z и калибровать интегрированием
        /flow_dbg2 с угаданным покадровым dt. Теперь сегмент виден по движению уставки,
        насыщение (max_pwm) — по третьему слоту."""
        if hold is None:
            return
        self._publish_hold(self._pub6, (hold[0], hold[1], yaw_off))

    def _publish_hold(self, pub, hold) -> None:
        if hold is None:
            return
        d = Vector3Stamped()
        d.header.stamp = self._node.get_clock().now().to_msg()
        d.vector.x, d.vector.y, d.vector.z = (float(v) for v in hold)
        pub.publish(d)

    def publish_axes(self, s, rc) -> None:
        """Полный флоу-дамп (sim-штамп): /flow_dbg = (roll_off, flow_lateral, conf),
        /flow_dbg2 = (pitch_off, flow_longitudinal, flow_yaw). Для свипа: сверить
        сигнал ↔ команду ↔ истинную скорость (odometry) в одном sim-времени."""
        t = self._node.get_clock().now().to_msg()
        d = Vector3Stamped()
        d.header.stamp = t
        d.vector.x = float(rc.roll - RC_CENTER)
        d.vector.y = float(s.flow_lateral)
        d.vector.z = float(s.flow_conf)
        self._pub.publish(d)
        d2 = Vector3Stamped()
        d2.header.stamp = t
        d2.vector.x = float(rc.pitch - RC_CENTER)
        d2.vector.y = float(s.flow_longitudinal)
        d2.vector.z = float(s.flow_yaw)
        self._pub2.publish(d2)
        # /flow_dbg3 = ОПОРА: (log масштаба = ПОЛОЖЕНИЕ, оконная СКОРОСТЬ опоры,
        # сколько точек видно). Второй слот раньше держал kf_dx (сдвиг X) — в разборе
        # продольной оси он не использовался ни разу, а kf_vel теперь D-член контура,
        # и без него в бэге нельзя проверить демпфер.
        d3 = Vector3Stamped()
        d3.header.stamp = t
        d3.vector.x = float(s.kf_logs)
        d3.vector.y = float(s.kf_vel)
        d3.vector.z = float(s.kf_n)
        self._pub3.publish(d3)
        # /flow_dbg4 = СУДЬБА ОПОРЫ, накопительные счётчики: (сегментов ЗАЧТЕНО,
        # пересевов с ВЫБРОШЕННЫМ сегментом, кадров-выбросов). Из телеметрии их не
        # восстановить: пересев виден только по прыжку kf_n ВВЕРХ, а если новый посев
        # дал точек меньше, чем отслеживалось, он неотличим — по J1b так насчитались
        # 25 пересевов в висении при явно большем числе (накопитель дошёл до 0.42,
        # то есть закрытых сегментов было около полутора десятков).
        d4 = Vector3Stamped()
        d4.header.stamp = t
        d4.vector.x = float(s.kf_segs)
        d4.vector.y = float(s.kf_reseeds)
        d4.vector.z = float(s.kf_rejects)
        self._pub4.publish(d4)
        # /flow_dbg8 = КАНАЛ ВИДА СВЕРХУ: (путь вперёд М, продольная скорость М/С,
        # достоверность С ПРИЧИНОЙ брака). Единственный канал в МЕТРАХ — остальные
        # безразмерны, поэтому без него полёт на DpPitchRate нечем разбирать: и
        # сигнал, и уставка оси живут здесь.
        # z = достоверность С ПРИЧИНОЙ брака кадра: кодировка и таблица кодов —
        # ipm_dbg_z / FlowEstimator.ipm_fail (перцепция; тестируется без ROS).
        # Совместимость: «z>0.5 = ок» работает и на старых, и на новых bag.
        z_ipm = ipm_dbg_z(s.ipm_ok, s.ipm_fail)
        d8 = Vector3Stamped()
        d8.header.stamp = t
        d8.vector.x = float(s.ipm_fwd)
        d8.vector.y = float(s.ipm_vfwd)
        d8.vector.z = z_ipm
        self._pub8.publish(d8)
        # /flow_dbg9 = тот же канал, БОКОВАЯ ось: (боковая скорость М/С, продольная для
        # пары, достоверность с причиной — та же z_ipm). Публикуется из СНАПШОТА, а не
        # из контура — поэтому живёт и в чистом наблюдении, без единого стабилизатора
        # в стеке.
        d9 = Vector3Stamped()
        d9.header.stamp = t
        d9.vector.x = float(s.ipm_vlat)
        d9.vector.y = float(s.ipm_vfwd)
        d9.vector.z = z_ipm
        self._pub9.publish(d9)
