#!/usr/bin/env python3
"""AttitudeCommand — КОМАНДА ДОМЕНА в СИ: её отдают стабилизаторы, стек, шаги плана, арбитр.

Чистый домен: ноль импортов rclpy/mavros. Единицы СИ, соглашения ArduPilot (оси тела
FRD, как в логах полётника и в его коде): крен вправо +, тангаж нос ВВЕРХ + (стик
«вперёд» = отрицательный тангаж), курс по часовой +, набор вверх +. В оси ROS
(FLU/ENU) переводит адаптер MAVROS — домену рамы ROS знать не нужно.

СМЫСЛ ПОЛЕЙ (носитель в СИ с 2026-09-30, фаза A). roll/pitch — наклон «как у стика
ALT_HOLD»: линейный масштаб units.py (20° на 500 µs); форму вектора тяги и потолок наклона
накладывает адаптер углов (att_convert.rc_to_attitude/limit_tilt). yaw_rate — темп курса
(90 °/с на 500 µs). climb — скорость набора, м/с (карта газа — domain/control/altitude.py
ThrottleMap). В ярусе LOITER roll/pitch несут стик пилота как есть (уставку скорости для
LOITER полётника) тем же масштабом — провод получает ровно прежние µs.

Перевод в провод и обратно — application/command_wire.py (override: µs каналов; канал
углов: через те же µs, пока фаза A держит обе стороны бит в бит).
"""
from dataclasses import dataclass


@dataclass
class AttitudeCommand:
    roll: float = 0.0        # рад, вправо +
    pitch: float = 0.0       # рад, нос вверх +
    yaw_rate: float = 0.0    # рад/с, по часовой +
    climb: float = 0.0       # м/с, вверх +

    # ось стека зовётся «yaw» (StabilizationStrategy.axes), поле — темп: псевдоним, чтобы
    # стек перезаписывал оси по имени (setattr(cmd, "yaw", …)) без таблицы соответствий
    @property
    def yaw(self) -> float:
        return self.yaw_rate

    @yaw.setter
    def yaw(self, v: float) -> None:
        self.yaw_rate = v
