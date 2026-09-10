#!/usr/bin/env python3
"""Юнит-тест гейта здоровья моста VINS→EKF (nav_pkg/nn1/bridge_gate.py). Чистый python.

Полёт lv2_joy_20260906_142811: разнос VINS после init (|v| → 49 м/с, скачки позы) →
687 подтяжек якоря → отравленная ориентация EKF → DpHold унесло. Проверяет: здоровый
поток — открыт; потолок скорости закрывает с латчем hold; скачок позы / дыра штампов =
перерождение → закрыт + флаг «якорь заново»; шторм подтяжек закрывает; внешний вердикт
ноды закрывает; после hold и здоровом потоке — открыт; reset() по /restart.

Запуск:  python3 src/nav/test/test_bridge_gate.py
"""
import os
import math
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from nav_pkg.nn1.bridge_gate import BridgeGate, ready_verdict   # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def stream(g, t0, n, v=0.5, dt=0.1, x0=0.0, ext=None, ready=None):
    """n одометрий здорового потока: ход v м/с по x."""
    t, x = t0, x0
    op = None
    for i in range(n):
        op = g.on_odom(t, x, 0.0, v, ext, ready=ready)
        t += dt; x += v * dt
    return op, t, x


# 1. здоровый поток — открыт, латч заново не просится
g = BridgeGate()
op, t, x = stream(g, 100.0, 50)
check("здоровый поток 5 с: открыт, closes 0, relatch_pending нет",
      op and g.closes == 0 and not g.relatch_pending)
# 2. потолок скорости → закрыт, держится hold 5 с, потом открыт
op = g.on_odom(t, x, 0.0, 15.0); t += 0.1
check("|twist| 15 > 12: закрыт, причина vNN", (not op) and g.reason.startswith('v') and g.closes == 1)
op, t, x = stream(g, t, 30, x0=x)                   # 3 с здорового — ещё закрыт
check("через 3 с здорового потока — ещё закрыт (латч hold 5 с)", not g.is_open(t - 0.1))
op, t, x = stream(g, t, 30, x0=x)                   # ещё 3 с — открыт
check("через 6 с — открыт, closes по-прежнему 1", op and g.closes == 1)
check("после потолка скорости якорь заново НЕ просится (рама та же)", not g.relatch_pending)
# 3. скачок позы (перерождение) → закрыт + якорь заново
op = g.on_odom(t, x + 5.0, 0.0, 0.5); t += 0.1     # 5 м за 0.1 с = 50 м/с
check("скачок позы 50 м/с: закрыт, reborn, rebirths 1, якорь заново",
      (not op) and g.reason == 'reborn' and g.rebirths == 1 and g.take_relatch())
check("take_relatch снимает флаг", not g.take_relatch())
# 4. дыра штампов > 1 с — тоже перерождение
g2 = BridgeGate(); stream(g2, 10.0, 10)
op = g2.on_odom(12.5, 0.5, 0.0, 0.3)
check("дыра штампов 1.5 с: перерождение", (not op) and g2.rebirths == 1 and g2.relatch_pending)
# 5. шторм подтяжек: 3 за 5 с → закрыт, якорь заново; 2 за 5 с — нет
g3 = BridgeGate(relatch_n=3, relatch_win=5.0); _, t3, x3 = stream(g3, 20.0, 10)
a = g3.on_relatch(t3); b = g3.on_relatch(t3 + 1.0)
check("2 подтяжки за 5 с: открыт", (not a) and (not b) and g3.is_open(t3 + 1.0))
c = g3.on_relatch(t3 + 2.0)
check("3-я подтяжка за 5 с: шторм — закрыт, причина relatch, якорь заново",
      c and (not g3.is_open(t3 + 2.0)) and g3.reason == 'relatch' and g3.relatch_pending)
g4 = BridgeGate(relatch_n=3, relatch_win=5.0); _, t4, _ = stream(g4, 30.0, 10)
g4.on_relatch(t4); g4.on_relatch(t4 + 3.0); d = g4.on_relatch(t4 + 6.0)
check("3 подтяжки, но растянуты на 6 с (окно 5): не шторм", (not d) and g4.is_open(t4 + 6.0))
# 6. внешний вердикт ноды
g5 = BridgeGate(); _, t5, x5 = stream(g5, 40.0, 10)
op = g5.on_odom(t5, x5, 0.0, 0.5, ext_sane=False)
check("/vins/sane=False: закрыт, причина ext", (not op) and g5.reason == 'ext')
op, t5, x5 = stream(g5, t5 + 0.1, 60, x0=x5, ext=True)
check("после hold с sane=True: открыт", op)
# 7. reset() по /restart: следующая одометрия не сравнивается с прошлой (нет ложного
#    перерождения при скачке рамы), якорь заново
g6 = BridgeGate(); _, t6, x6 = stream(g6, 50.0, 10)
g6.reset()
op = g6.on_odom(t6, x6 + 100.0, 0.0, 0.3)
check("reset(): скачок рамы после /restart — не перерождение, открыт, якорь заново",
      op and g6.rebirths == 0 and g6.take_relatch())
# 7б. ГЕЙТ ЗРЕЛОСТИ (ready, 2026-09-09): пока VINS не доказал себя — закрыт,
# сколько бы здоровым ни выглядел поток (полёт 073004: 2-4 с мусора хватило,
# чтобы EKF уехал на 200 м)
g8 = BridgeGate()
op, t8, x8 = stream(g8, 200.0, 50, ready=False)
check("ready=False: мост ЗАКРЫТ на здоровом потоке, причина ripe",
      (not op) and g8.reason == 'ripe')
op, t8, x8 = stream(g8, t8, 20, x0=x8, ready=True)      # 2 с < hold 5 с
check("ready стал True: ещё закрыт (латч hold после последней причины)",
      not g8.is_open(t8 - 0.1))
op, t8, x8 = stream(g8, t8, 40, x0=x8, ready=True)
check("через hold при ready=True — ОТКРЫТ", op)
# без лётной ноды (ready=None) поведение прежнее
g9 = BridgeGate()
op, _, _ = stream(g9, 300.0, 30, ready=None)
check("ready=None (голый Orin без лётной ноды): мост живёт своими проверками", op)

# 7в. ПЕРВОЕ ОТКРЫТИЕ: гейт просит не усыновлять уехавший EKF (take_open_reset).
# Разбор 114844 (якорь залатчился к уехавшей позе → 16.2 м дрейфа на весь полёт)
# против 120819 (EKF успел сдаться сам, латчиться было не к чему → сброс, 2.25 м)
g10 = BridgeGate()
op, t10, x10 = stream(g10, 400.0, 30, ready=False)
check("пока мост ни разу не открывался — флага нет", not g10.take_open_reset())
op, t10, x10 = stream(g10, t10, 80, x0=x10, ready=True)   # закрытие отпустило
check("после первого открытия флаг ВЗВЕДЁН", op and g10.take_open_reset())
check("флаг одноразовый", not g10.take_open_reset())
# второе закрытие/открытие в том же полёте флага НЕ даёт: там борт уже летит,
# сброс позы полётника в воздухе опаснее усыновлённого дрейфа
op = g10.on_odom(t10, x10, 0.0, 15.0); t10 += 0.1          # закрыли потолком
op, t10, x10 = stream(g10, t10, 80, x0=x10)
check("повторное открытие флага не даёт (в полёте не сбрасываем раму)",
      op and not g10.take_open_reset())
# выключатель
g11 = BridgeGate(open_reset=False)
_, t11, x11 = stream(g11, 500.0, 30, ready=False)
op, t11, x11 = stream(g11, t11, 80, x0=x11, ready=True)
check("open_reset=False: флага нет (поведение до 2026-09-09)",
      op and not g11.take_open_reset())

# 8. state_line формат
g7 = BridgeGate(); _, t7, _ = stream(g7, 60.0, 5)
w = g7.state_line(t7).split()
check("state_line: 'open - 0 0 0'", w == ['open', '-', '0', '0', '0'])

# 9. СБОРКА строки /nn1/bridge как её делает ray_tracer: 5 полей гейта +
#    Δyaw якоря (снимок шва) + Δyaw СЕЙЧАС. Седьмое поле читает лётная нода
#    (ros_telemetry._on_bridge → dyaw_now) и по его устойчивости судит зрелость
#    (rth_ready._dyaw_steady). Полёты 154759/155443: поля не было вовсе —
#    dyaw_now=None весь полёт, brdn= пустое.
def bridge_line(gate, t, yaw_off=None, dnow=None):
    d = f"{math.degrees(yaw_off):+.1f}" if yaw_off is not None else "-"
    n = f"{math.degrees(dnow):+.1f}" if dnow is not None else "-"
    return f"{gate.state_line(t)} {d} {n}"


g12 = BridgeGate(); _, t12, _ = stream(g12, 60.0, 5)
w = bridge_line(g12, t12, yaw_off=math.radians(12.0), dnow=math.radians(-3.5)).split()
check("строка моста: 7 полей", len(w) == 7)
check("шестое поле — Δyaw якоря", w[5] == '+12.0')
check("седьмое поле — Δyaw сейчас", w[6] == '-3.5')
w = bridge_line(g12, t12).split()
check("без латча/курса — прочерки, поля на месте",
      len(w) == 7 and w[5] == '-' and w[6] == '-')

# 10. ВЕРДИКТ ЗРЕЛОСТИ ОБЯЗАТЕЛЕН (ready_verdict). Разбор 192430: до первого
# /vins/bridge_ok мост был ОТКРЫТ, и в EKF ушли позы в раме, залатченной от позы
# КРУТЯЩЕГОСЯ НА ЗЕМЛЕ борта (спавн diagonal, в мире нет трения) — курс EKF утащило,
# а latch_yaw потом наследовал ошибку из него. Попытка «подождать вердикт N секунд»
# ПРОВАЛИЛАСЬ в полёте 170043: ray_tracer живёт со стеком, а лётная нода поднимается
# на каждый прогон и заговорила через 22.6 с — таймаут истёк раньше и открыл мост.
check("свежий вердикт «зрел» → True", ready_verdict(True, 0.5, 3.0, True) is True)
check("свежий вердикт «не зрел» → False", ready_verdict(False, 0.5, 3.0, True) is False)
check("вердикта нет и он ОБЯЗАТЕЛЕН → False (мост закрыт)",
      ready_verdict(None, 999.0, 3.0, True) is False)
check("вердикт протух и он обязателен → False (нода замолчала — не доверяем)",
      ready_verdict(True, 5.0, 3.0, True) is False)
check("required=False (голый стример) → None, гейт зрелости не применяется",
      ready_verdict(None, 999.0, 3.0, False) is None)
check("required=False, но вердикт свежий → он и есть ответ",
      ready_verdict(True, 0.5, 3.0, False) is True)
# и то же через сам гейт: без вердикта здоровый поток НЕ открывает мост
g13 = BridgeGate()
op13, t13, _ = stream(g13, 700.0, 30, ready=ready_verdict(None, 999.0, 3.0, True))
check("гейт без вердикта: мост ЗАКРЫТ, причина ripe",
      (not op13) and g13.reason == 'ripe')
g14 = BridgeGate()
op14, _, _ = stream(g14, 800.0, 30, ready=ready_verdict(None, 999.0, 3.0, False))
check("гейт без вердикта при required=False: мост живёт своими проверками", op14)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ BRIDGE GATE OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
