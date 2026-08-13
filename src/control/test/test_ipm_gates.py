#!/usr/bin/env python3
"""Юнит-тест ГЕЙТОВ ДОВЕРИЯ к каналу вида сверху (`_IpmGated`: DpRollRate/DpPitchRate).

Зачем. Курс уже вылечен от слепой веры в картинку (разбор YW1s1: один мусорный кадр
развернул борт на 96°). У осей по СКОРОСТИ болезнь та же по природе, но повод другой
и цена выше: замер по 8 прогонам показал, что тангаж сидит В НАСЫЩЕНИИ 22–43% первых
трёх секунд после отрыва — гоняется за фантомом набора высоты (канал показывает
4.5–7.2 м/с вперёд при истинных ≤1.3, потому что полоса земли лежит ВПЕРЕДИ борта и
ход по высоте читается как ход вперёд).

Что проверяем:
1. потолок правдоподобия: невозможная скорость → ось молчит, кадр ВЫБРОШЕН (не подрезан);
2. гейт по вертикальной скорости: на наборе продольная ось молчит, боковая — работает
   (её порог выключен по умолчанию, и это осознанно);
3. взведение: после срыва ось молчит `arm_frames` кадров и только потом командует;
4. сценарий взлёта целиком: фантом 4.5 м/с на наборе НЕ доходит до рулей.

Запуск:  python3 src/control/test/test_ipm_gates.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.stabilization import (               # noqa: E402
    DpPitchRate, DpRollRate)
from control_pkg.domain.rc import RC_CENTER                          # noqa: E402
from control_pkg.domain.setpoint import Setpoint                     # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class Fly:
    """Кадры с ЧАСАМИ и ВЫСОТОЙ: гейт по vz считает наклон баро-высоты в своём окне,
    поэтому кадры обязаны идти по времени и нести rel_alt — одиночным `DroneState`
    эту ветку не проверить."""

    def __init__(self, alt=3.0, dt=0.05):
        self.t, self.alt, self.dt, self.seq = 0.0, alt, dt, 0

    def step(self, vz=0.0, **kw):
        self.seq += 1
        self.t += self.dt
        self.alt += vz * self.dt
        kw.setdefault('ipm_ok', True)
        return DroneState(flow_seq=self.seq, now_sim=self.t, flow_dt=self.dt,
                          flow_conf=0.5, rel_alt=self.alt, **kw)


def run(ax, fly, n, vz=0.0, **kw):
    """n кадров подряд; вернуть последнюю команду по оси стратегии."""
    rc = None
    for _ in range(n):
        rc = ax.update(fly.step(vz=vz, **kw), Setpoint(), fly.dt)
    return getattr(rc, next(iter(ax.axes)))


# --- 1. потолок правдоподобия: кадр выбрасывается, а не подрезается ---
p = DpPitchRate(kp=100.0, ki=0.0, kd=0.0, max_speed=8.0, vz_max=0.0, arm_frames=0)
f = Fly()
p.enter(DroneState(flow_seq=-1))
check("правдоподобие: 1.0 м/с — нормальный кадр, ось командует",
      run(p, f, 1, ipm_vfwd=1.0) == RC_CENTER + 100)
out = run(p, f, 1, ipm_vfwd=50.0)
check("правдоподобие: 50 м/с — кадр выброшен, команда в центр", out == RC_CENTER)
check("правдоподобие: выброс посчитан", p._rejects == 1)
# ⚠️ ИМЕННО ВЫБРОШЕН, не подрезан: подрезка до потолка дала бы 8 м/с → 800 → потолок
# PWM 150, то есть максимальную команду ПО МУСОРНОМУ КАДРУ.
check("правдоподобие: подрезки нет (иначе было бы насыщение 150)", out != RC_CENTER + 150)

# --- 2. гейт по вертикальной скорости: продольная ось глохнет, боковая нет ---
# Набор 1 м/с — ровно как на взлёте. Сигнал при этом «хороший» (2 м/с, потолок не
# трогает), и без гейта ось выдала бы 200 → насыщение 150.
p = DpPitchRate(kp=100.0, ki=0.0, kd=0.0, max_speed=8.0, vz_max=0.35, arm_frames=0)
f = Fly()
p.enter(DroneState(flow_seq=-1))
check("набор 1 м/с: продольная ось молчит, хотя канал показывает 2 м/с",
      run(p, f, 20, vz=1.0, ipm_vfwd=2.0) == RC_CENTER)
check("набор: закрытые кадры посчитаны", p._vz_blocks > 0)

r = DpRollRate(kp=30.0, ki=0.0, kd=0.0, max_speed=8.0, vz_max=0.0, arm_frames=0)
f = Fly()
r.enter(DroneState(flow_seq=-1))
check("набор 1 м/с: БОКОВАЯ ось продолжает демпфировать (порог выключен)",
      run(r, f, 20, vz=1.0, ipm_vlat=1.0) == RC_CENTER + 30)

# --- 3. взведение: после срыва ось молчит arm_frames кадров ---
p = DpPitchRate(kp=100.0, ki=0.0, kd=0.0, max_speed=8.0, vz_max=0.35, arm_frames=5)
f = Fly()
p.enter(DroneState(flow_seq=-1))
run(p, f, 10, vz=1.0, ipm_vfwd=2.0)                 # набор: гейт закрыт, счётчик в нуле
outs = [run(p, f, 1, vz=0.0, ipm_vfwd=1.0) for _ in range(25)]
# ⚠️ ПАУЗА ДЛИННЕЕ ВЗВЕДЕНИЯ, и это свойство, а не сбой: наклон высоты считается по
# окну 0.5 с, где ещё лежит ХВОСТ набора, и только после того как окно очистится,
# начинают набираться 5 кадров взведения. Итого ~0.75 с молчания после конца набора.
# Проверяем не «ровно шестой кадр», а обе границы: сначала молчит, к секунде работает.
wake = next((i for i, o in enumerate(outs) if o != RC_CENTER), None)
check("взведение: сразу после набора ось ещё молчит", outs[0] == RC_CENTER)
check(f"взведение: ось оживает на {wake}-м кадре — дольше взведения, но внутри 1 с",
      wake is not None and 5 <= wake <= 20)
check("взведение: ожив, ось командует полным законом (100 PWM на 1 м/с)",
      outs[-1] == RC_CENTER + 100)

# срыв канала обнуляет счётчик: одиночный хороший кадр после провала не взводит
p2 = DpPitchRate(kp=100.0, ki=0.0, kd=0.0, max_speed=8.0, vz_max=0.0, arm_frames=5)
f2 = Fly()
p2.enter(DroneState(flow_seq=-1))
run(p2, f2, 10, ipm_vfwd=1.0)                        # взведена и работает
check("взведённая ось командует", run(p2, f2, 1, ipm_vfwd=1.0) == RC_CENTER + 100)
run(p2, f2, 3, ipm_ok=False, ipm_vfwd=1.0)           # канал провалился
check("после провала канала первый же хороший кадр НЕ командует",
      run(p2, f2, 1, ipm_vfwd=1.0) == RC_CENTER)
check("после провала ось оживает на 6-м хорошем кадре",
      run(p2, f2, 5, ipm_vfwd=1.0) == RC_CENTER + 100)

# --- 4. сценарий взлёта: фантом набора не доходит до рулей ---
# Как в замере: отрыв, набор ~1.1 м/с, канал врёт про 4.5 м/с вперёд две секунды,
# потом набор кончается и канал становится честным (0.2 м/с).
p = DpPitchRate(kp=100.0, ki=100.0, kd=0.0, imax=120.0,
                max_speed=8.0, vz_max=0.35, arm_frames=5)
f = Fly(alt=0.3)
p.enter(DroneState(flow_seq=-1))
sat = 0
for _ in range(40):                                  # 2 с набора с фантомом
    rc = p.update(f.step(vz=1.1, ipm_vfwd=4.5), Setpoint(), f.dt)
    sat += int(abs(rc.pitch - RC_CENTER) >= 149)
check("взлёт: насыщения тангажа НЕТ ни в одном кадре набора", sat == 0)
check("взлёт: интегратор не заряжен фантомом", abs(p._i) < 1e-9)
tail = [p.update(f.step(vz=0.0, ipm_vfwd=0.2), Setpoint(), f.dt).pitch for _ in range(15)]
check("взлёт: после набора ось возвращается в работу (20 PWM на 0.2 м/с)",
      tail[-1] != RC_CENTER)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ГЕЙТЫ ВИДА СВЕРХУ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
