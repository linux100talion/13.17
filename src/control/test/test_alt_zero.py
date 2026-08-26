#!/usr/bin/env python3
"""Юнит-тест ЛАТЧА НУЛЯ ВЫСОТЫ ПЕРЦЕПЦИИ (RosPerception.latch_alt_zero) — без ROS.

Зачем это проверять. Гейт земли канала вида сверху судит по ВЫСОТЕ ПЕРЦЕПЦИИ, а в
GPS-denied профиле она = z `/mavros/local_position/pose`, смещённый вниз на 0.2-0.3 м.
На полёте ниже полуметра это БОЛЬШЕ всей высоты: прогоны lv2_joy_20260826_183305 и
185921 (истинные 0.26-0.27 м) дали `/flow_dbg8.z` = код 1 в 100% и 90% кадров, демпфер
выдал 0 PWM, снос 54 и 40 м при стиках в центре. Смещение ПОСТОЯННОЕ, а дельта точна
(земля −0.29 → воздух −0.00 при истинных +0.27), поэтому лечится латчем нуля на арме —
офлайн-реплей (src/lab/ipm_alt_replay.py, вариант C) на обоих bag дал ровно потолок
варианта «истинная высота».

Адаптер создаём через object.__new__ — ROS-подписки в __init__ тут не нужны,
проверяется чистая арифметика латча.

Запуск:
  docker exec p1317_nav bash -lc 'cd /root/sim_ws/src/control && python3 test/test_alt_zero.py'
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.infrastructure.ros_perception import RosPerception    # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class Msg:
    """Заглушка PoseStamped: адаптеру нужен только pose.position.z."""

    def __init__(self, z):
        self.pose = type('P', (), {'position': type('Q', (), {'z': z})()})()


def mk(on=True):
    p = object.__new__(RosPerception)
    p._alt = None
    p._alt_zero_on = on
    p._alt_zero = 0.0
    p._alt_raw = None
    p._alt_zero_pending = False
    return p


# --- 1. ВЫКЛЮЧЕННЫЙ латч — прежнее поведение бит-в-бит (кламп нулём) ---
p = mk(on=False)
p._on_lpos_alt(Msg(-0.28))
check("выкл: отрицательный z клампится нулём", p._alt == 0.0)
p._on_lpos_alt(Msg(0.05))
check("выкл: положительный z как есть", abs(p._alt - 0.05) < 1e-9)
check("выкл: latch_alt_zero — no-op", p.latch_alt_zero() is None and p._alt_zero == 0.0)

# --- 2. ЛАТЧ ПО АРМУ на числах прогона 183305 (земля −0.28, воздух −0.00) ---
p = mk()
p._on_lpos_alt(Msg(-0.28))                     # борт на земле до арма
check("до латча: земля читается нулём (как летали)", p._alt == 0.0)
z0 = p.latch_alt_zero()                        # ← фронт armed
check("латч вернул z земли", abs(z0 - (-0.28)) < 1e-9)
check("сразу после латча высота = 0", p._alt == 0.0)
p._on_lpos_alt(Msg(-0.00))                     # борт в воздухе на истинных +0.27
check("в воздухе высота = дельта от земли (0.28, а не 0.00)",
      abs(p._alt - 0.28) < 1e-9)
check("гейт земли _ALT_GROUND=0.08 ОТКРЫТ (было закрыто)", p._alt >= 0.08)

# --- 3. ОТЛОЖЕННЫЙ латч: local_position ещё молчит на арме (GPS-denied) ---
p = mk()
check("латч без единого сообщения → отложен", p.latch_alt_zero() is None)
check("флаг ожидания взведён", p._alt_zero_pending is True)
p._on_lpos_alt(Msg(-0.31))                     # первое сообщение ПОСЛЕ арма
check("первое сообщение стало нулём", abs(p._alt_zero - (-0.31)) < 1e-9)
check("оно же дало высоту 0", p._alt == 0.0)
check("флаг ожидания снят", p._alt_zero_pending is False)
p._on_lpos_alt(Msg(0.06))
check("дальше высота считается от него", abs(p._alt - 0.37) < 1e-9)

# --- 4. РЕ-АРМ: ноль берётся заново (посадка в другой точке/дрейф EKF) ---
p = mk()
p._on_lpos_alt(Msg(-0.28))
p.latch_alt_zero()
p._on_lpos_alt(Msg(-0.19))                     # сели, ноль EKF уехал на 9 см
check("до ре-арма старый ноль даёт ложные 0.09", abs(p._alt - 0.09) < 1e-9)
p.latch_alt_zero()                             # ← фронт armed второй раз
check("ре-арм: высота снова 0", p._alt == 0.0)
p._on_lpos_alt(Msg(0.11))
check("ре-арм: высота от НОВОГО нуля", abs(p._alt - 0.30) < 1e-9)

# --- 5. Кламп живёт и с латчем: просадка ниже точки арма не даёт минус ---
p = mk()
p._on_lpos_alt(Msg(-0.20))
p.latch_alt_zero()
p._on_lpos_alt(Msg(-0.35))                     # EKF просел ниже точки арма
check("ниже нуля → 0, а не отрицательная высота", p._alt == 0.0)

bad = [n for n, ok in results if not ok]
print(f"\nитого: {len(results) - len(bad)}/{len(results)} OK")
sys.exit(1 if bad else 0)
