#!/usr/bin/env python3
"""Юнит-тест ЛАТЧА ДОВЕРИЯ К ВОЗВРАТУ (RthReadiness). Чистый python, без ROS.

Проверяет: круг лечения по IPM (с поворотом по курсу), латч по зрелости VINS,
запись дома и трека, все причины дисквалификации (перерождение / insane /
закрытый мост / выход из круга / таймаут), отдельный выход `ripe` для гейта
зрелости моста и чистый старт после дизарма.

Запуск:  python3 src/control/test/test_rth_ready.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.rth_ready import RthReadiness      # noqa: E402
from control_pkg.domain.state import DroneState                 # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def snap(t, fwd=0.0, lat=0.0, seq=0, yaw=0.0, odom=300, age=0.0, armed=True,
         reb=0, brg_seen=False, brg_open=True, x=0.0, y=0.0, z=2.0, dyaw=0.0):
    return DroneState(now_sim=t, armed=armed, ipm_fwd=fwd, ipm_lat=lat,
                      flow_seq=seq, att_yaw=yaw, vins_odom_count=odom,
                      vins_last_sim=t - age, vins_rebirths=reb,
                      bridge_seen=brg_seen, bridge_open=brg_open,
                      ekf_x=x, ekf_y=y, ekf_z=z, dyaw_now=dyaw)


def mk(**kw):
    # home_settle=1 — чтобы тесты были короткими; отдельный блок ниже проверяет
    # саму выдержку (полётный дефолт 3 с ≥ anchor_open_reset_sec ray_tracer)
    args = dict(radius=5.0, heal_sec=30.0, ripe_sec=5.0, min_count=300,
                fresh_sec=2.0, track_m=3.0, home_settle=1.0)
    args.update(kw)
    return RthReadiness(**args)


def run(r, t0, dur, dt=0.05, sane=True, seq0=0, **kw):
    """Кормит политику dur секунд снапшотами snap(**kw); возвращает (state, t, seq)."""
    t, seq = t0, seq0
    n = int(round(dur / dt))
    for _ in range(n):
        t += dt
        seq += 1
        r.update(snap(t, seq=seq, **kw), sane=sane)
    return r.state, t, seq


# --- 1. на земле ничего не происходит ---
r = mk()
r.update(snap(10.0, armed=False))
check("не заармлен: heal, дома нет", r.state == 'heal' and r.home is None)

# --- 2. латч по зрелости: ripe_sec здоровья в круге ---
r = mk()
st, t, seq = run(r, 100.0, 4.0, x=1.0, y=0.0)
check("здоровый VINS < ripe_sec: ещё heal", st == 'heal' and not r.ripe)
st, t, seq = run(r, t, 1.5, seq0=seq, x=1.0, y=0.0)
check("зрелость есть, но рама ещё не устоялась (home_settle) — heal/settle",
      st == 'heal' and r.ripe and r.why == 'settle')
st, t, seq = run(r, t, 1.1, seq0=seq, x=1.0, y=0.0)
check("рама спокойна home_settle → READY", st == 'ready')
check("ripe взведён (гейт зрелости моста откроет мост)", r.ripe)
check("дом записан позой EKF", r.home == (1.0, 0.0, 2.0))
check("трек начат домом", r.track == [(1.0, 0.0, 2.0)])

# --- 3. незрелый VINS (odom < min_count) не латчится ---
r = mk()
st, _, _ = run(r, 200.0, 10.0, odom=100)
check("odom < min_count: латча нет (и ripe пуст)", st == 'heal' and not r.ripe)

# --- 4. протухший поток не латчится ---
r = mk()
st, _, _ = run(r, 300.0, 10.0, age=5.0)
check("поток VINS протух (age > fresh_sec): латча нет", st == 'heal')

# --- 5. вышли из круга, не успев вылечиться → LOST:circle ---
r = mk()
st, t, seq = run(r, 400.0, 2.0, fwd=0.0, odom=100)
# уезжаем на 6 м вперёд одним кадром пути (IPM: путь растёт монотонно)
r.update(snap(t + 0.05, fwd=6.0, seq=seq + 1, odom=100), sane=True)
check("выход из круга без латча → LOST:circle",
      r.state == 'lost' and r.why == 'circle')

# --- 6. таймаут лечения → LOST:timeout ---
r = mk(heal_sec=10.0)
st, t, seq = run(r, 500.0, 11.0, odom=100)
check("не вылечились за heal_sec → LOST:timeout",
      r.state == 'lost' and r.why == 'timeout')

# --- 7. круг считается ПО IPM с поворотом на курс ---
r = mk()
r.update(snap(600.0, seq=1, yaw=math.pi / 2))
r.update(snap(600.05, fwd=4.0, seq=2, yaw=math.pi / 2))
check("курс 90°: ход вперёд 4 м → смещение по y (мировые оси)",
      abs(r._x) < 1e-6 and abs(r._y - 4.0) < 1e-6 and abs(r.dist - 4.0) < 1e-6)

# --- 8. после латча рама рвётся: три причины ---
for why, kw in (('reborn', dict(reb=1)), ('bridge', dict(brg_seen=True, brg_open=False))):
    r = mk()
    st, t, seq = run(r, 700.0, 7.0)
    check(f"перед разрывом READY ({why})", st == 'ready')
    st, t, seq = run(r, t, 0.2, seq0=seq, **kw)
    check(f"после латча {why} → LOST:{why}", r.state == 'lost' and r.why == why)
r = mk()
st, t, seq = run(r, 800.0, 7.0)
st, t, seq = run(r, t, 0.2, seq0=seq, sane=False)
check("после латча гейт объявил VINS больным → LOST:insane",
      r.state == 'lost' and r.why == 'insane')

# --- 9. пока мост ЗАКРЫТ, латча нет: доверять раме, которой FCU не видит, нельзя
#        (BridgeGate держит закрытие hold_sec после причины — латч тут же ловил бы
#        дисквалификацию brg=0). Открылся — латчимся сразу, зрелость уже есть ---
r = mk()
st, t, seq = run(r, 900.0, 6.0, brg_seen=True, brg_open=False)
check("мост закрыт: латча нет, ждём (why=bridge-wait)",
      st == 'heal' and r.ripe and r.why == 'bridge-wait')
st, t, seq = run(r, t, 1.2, seq0=seq, brg_seen=True, brg_open=True)
check("мост открылся → латч после выдержки (зрелость уже набрана)",
      r.state == 'ready')

# --- 10. дом ФИКСИРУЕТСЯ в точке латча, дальше пишется трек ---
r = mk()
st, t, seq = run(r, 1000.0, 7.0, x=0.0, y=0.0)
check("дом = точка латча (где созрела система)", r.home == (0.0, 0.0, 2.0))
for i in range(1, 13):                       # 12 м по x: и IPM-путь, и поза EKF
    t += 0.05
    seq += 1
    r.update(snap(t, seq=seq, fwd=float(i), x=float(i), y=0.0), sane=True)
check("дом НЕ уехал за бортом", r.home == (0.0, 0.0, 2.0))
check("трек пишется шагом 3 м от дома",
      r.track == [(0.0, 0.0, 2.0), (3.0, 0.0, 2.0), (6.0, 0.0, 2.0),
                  (9.0, 0.0, 2.0), (12.0, 0.0, 2.0)])
check("длина пути считается", abs(r.path_m - 12.0) < 1e-6)
check("расстояние до дома по прямой",
      abs(r.home_dist(snap(t, x=12.0, y=0.0)) - 12.0) < 1e-6)

# --- 10в. вернулись домой: дом и трек НЕ сбрасываются (полёт «туда и обратно») ---
n_tr = len(r.track)
for i in range(11, 0, -1):                   # летим обратно к дому
    t += 0.05
    seq += 1
    r.update(snap(t, seq=seq, fwd=float(i), x=float(i), y=0.0), sane=True)
check("возврат домой: дом на месте", r.home == (0.0, 0.0, 2.0))
check("возврат домой: трек не стёрт", len(r.track) >= n_tr)

# --- 10б. СКАЧОК КАДРА EKF (сброс к vision_pose / перелатч якоря) ---
# в круге — норма: дом идёт следом (полёт 103244: мост открылся, EKF прыгнул 10 м)
r = mk()
st, t, seq = run(r, 1050.0, 7.0, x=0.0, y=0.0)
t += 0.05; seq += 1
r.update(snap(t, seq=seq, x=10.0, y=0.0), sane=True)   # прыжок кадра в круге
check("скачок кадра ВНУТРИ круга: не дисквалификация, дом переставлен",
      r.state == 'ready' and r.home == (10.0, 0.0, 2.0))
# за кругом — дом и трек оказались в раме, которой больше нет
r = mk()
st, t, seq = run(r, 1060.0, 7.0, x=0.0, y=0.0)
for i in range(1, 9):                        # выходим из круга (8 м)
    t += 0.05; seq += 1
    r.update(snap(t, seq=seq, fwd=float(i), x=float(i), y=0.0), sane=True)
check("за кругом ещё READY", r.state == 'ready')
t += 0.05; seq += 1
r.update(snap(t, seq=seq, fwd=8.0, x=18.0, y=0.0), sane=True)   # скачок 10 м
check("скачок кадра ЗА кругом → LOST:jump", r.state == 'lost' and r.why == 'jump')

# --- 10г. ВЫДЕРЖКА ПЕРЕД ЛАТЧОМ: скачок кадра перезапускает отсчёт ---
# (ray_tracer на первом открытии моста нарочно отдаёт сырой VINS — полётник
# пересаживается на свежую раму скачком; дом до этого ставить нельзя)
r = mk(home_settle=3.0)
st, t, seq = run(r, 1400.0, 7.0, x=0.0, y=0.0)
check("зрелость (5 с) + выдержка ещё не вышла → heal", st == 'heal' and r.ripe)
st, t, seq = run(r, t, 1.5, seq0=seq, x=0.0, y=0.0)
check("зрелость 5 с + выдержка 3 с → READY", r.state == 'ready')
r2 = mk(home_settle=3.0)
st, t, seq = run(r2, 1500.0, 6.0, x=0.0, y=0.0)     # зрелость на 5-й с, идёт выдержка
t += 0.05; seq += 1
r2.update(snap(t, seq=seq, x=9.0, y=0.0), sane=True)   # скачок кадра
check("скачок в выдержке: латча ещё нет, отсчёт заново", r2.state == 'heal')
st, t, seq = run(r2, t, 2.0, seq0=seq, x=9.0, y=0.0)
check("2 с после скачка — всё ещё ждём", r2.state == 'heal')
st, t, seq = run(r2, t, 1.2, seq0=seq, x=9.0, y=0.0)
check("прошло 3 с спокойной рамы → латч в НОВЫХ координатах",
      r2.state == 'ready' and r2.home == (9.0, 0.0, 2.0))

# --- 10д. ЗРЕЛОСТЬ ПО УСТОЙЧИВОСТИ Δyaw (dyaw_tol): пока разность курсов
# «AHRS − VINS» гуляет, кадр VINS не сошёлся — латча нет ---
r = mk(dyaw_tol=2.0)
t, seq = 1600.0, 0
for i in range(200):                       # 10 с, Δyaw пляшет на ±5°
    t += 0.05
    seq += 1
    r.update(snap(t, seq=seq, dyaw=5.0 * (1 if i % 2 else -1)), sane=True)
check("Δyaw гуляет: зрелости нет, латча нет", not r.ripe and r.state == 'heal')
st, t, seq = run(r, t, 9.0, seq0=seq, dyaw=12.0)   # успокоился (значение любое)
check("Δyaw встал: зрелость набралась и дом залатчен", r.ripe and r.state == 'ready')
# чек выключен (dyaw_tol=0) — как было до 2026-09-09
r2 = mk(dyaw_tol=0.0)
st, t, seq = run(r2, 1700.0, 9.0, dyaw=5.0 * 1.0)
check("dyaw_tol=0: чек выключен, латч по прежним условиям", r2.state == 'ready')
# разность курсов через ±180 не «прыгает» (обёртка)
r3 = mk(dyaw_tol=2.0)
st, t, seq = run(r3, 1800.0, 9.0, dyaw=179.5)
st, t, seq = run(r3, t, 0.2, seq0=seq, dyaw=-179.5)   # это уход на 1°, не на 359°
check("Δyaw через ±180: обёртка, не ложный уход", r3.state == 'ready')

# --- 10е. чек Δyaw ДЕГРАДИРУЕТ В «НЕ МЕШАЮ», если числа нет вовсе ---
# (старый ray_tracer без седьмого поля, голый Orin, молчащий IMU). Разбор
# 154759/155443: обратное поведение заблокировало мост на весь полёт
r4 = mk(dyaw_tol=2.0)
t, seq = 1900.0, 0
for _ in range(200):
    t += 0.05
    seq += 1
    s = snap(t, seq=seq)
    s.dyaw_now = None
    r4.update(s, sane=True)
check("dyaw_now=None: зрелость НЕ блокируется (чек молчит)",
      r4.ripe and r4.state == 'ready')

# --- 11. LOST терминален, но ripe (мост) живёт своей жизнью ---
r = mk()
st, t, seq = run(r, 1100.0, 7.0)
st, t, seq = run(r, t, 0.2, seq0=seq, sane=False)
check("LOST зафиксирован", r.state == 'lost')
st, t, seq = run(r, t, 6.0, seq0=seq)         # VINS снова здоров
check("после LOST состояние не возвращается", r.state == 'lost')
check("но ripe снова True — мост открыт, ярусы работают", r.ripe)

# --- 12. дизарм = чистый старт следующего полёта ---
r = mk()
st, t, seq = run(r, 1200.0, 7.0)
r.update(snap(t + 0.05, armed=False))
check("дизарм сбрасывает латч", r.state == 'heal' and r.home is None)

# --- 13. статусная строка для HUD ---
r = mk()
check("status в HEAL: heal/<путь>", r.status(snap(1300.0)).startswith('heal/'))
st, t, seq = run(r, 1300.0, 7.0)
check("status в READY: ready/<до дома>", r.status(snap(t, x=7.0)) == 'ready/7')
r.state, r.why = 'lost', 'reborn'
check("status в LOST: причина видна", r.status(snap(t)) == 'lost:reborn')

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ RTH READY OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
