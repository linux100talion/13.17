#!/usr/bin/env python3
"""RcReverse — компенсация реверса каналов полётника на override (без ROS).

Проверяем против ФОРМУЛЫ ARDUPILOT (`RC_Channel::norm_input_dz`, a824813 — override
идёт через неё так же, как приёмник): «домен → зеркало → FCU с REVERSED 1» даёт ту же
нормированную команду, что «домен → FCU с REVERSED 0». Параметры — реального борта
(stellar_cld.txt: 988/1500/2011, DZ 20) и SITL (1000/1500/2000, DZ 0).
Плюс: реверс 0 (сим) — выход бит в бит; не прочитано / реверс газа → None (release);
спецзначения 0/65535 не зеркалятся.

Запуск:  python3 src/control/test/test_rc_reverse.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.rc import RC_NOCHANGE, RC_RELEASE              # noqa: E402
from control_pkg.infrastructure.rc_reverse import RcReverse            # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def fcu_norm(radio_in, lo, trim, hi, dz, reversed_):
    """RC_Channel::norm_input_dz() ArduPilot, дословно (с constrain)."""
    dz_min, dz_max = trim - dz, trim + dz
    mul = -1 if reversed_ else 1
    if radio_in < dz_min and dz_min > lo:
        ret = mul * (radio_in - dz_min) / (dz_min - lo)
    elif radio_in > dz_max and hi > dz_max:
        ret = mul * (radio_in - dz_max) / (hi - dz_max)
    else:
        ret = 0.0
    return max(-1.0, min(1.0, ret))


def rev(params):
    r = RcReverse()
    for k, v in params.items():
        r.on_param(k, v)
    return r


SIM = {'RC1_REVERSED': 0, 'RC2_REVERSED': 0, 'RC3_REVERSED': 0, 'RC4_REVERSED': 0}
BOARD = {'RC1_REVERSED': 0, 'RC2_REVERSED': 1, 'RC3_REVERSED': 0, 'RC4_REVERSED': 0,
         'RC1_TRIM': 1500, 'RC2_TRIM': 1500, 'RC4_TRIM': 1500}

# 1. сим (реверс 0): выход бит в бит
r = rev(SIM)
cases = [[1500, 1500, 1500, 1500], [1100, 1900, 1000, 1700], [1523, 1477, 1600, 1499]]
check("сим: реверс 0 → каналы без изменений", all(r.apply(c) == c for c in cases))

# 2. борт: тангаж зеркалится, остальные нет
r = rev(BOARD)
check("борт: готов, зеркалим только тангаж", r.ready() and r.reversed_channels() == [2])
check("борт: 1400 по тангажу → 1600, крен/газ/курс не тронуты",
      r.apply([1450, 1400, 1300, 1550]) == [1450, 1600, 1300, 1550])

# 3. эквивалентность по формуле FCU: зеркало + REVERSED 1 == домен + REVERSED 0
worst = 0.0
for v in range(1100, 1901):
    want = fcu_norm(v, 988, 1500, 2011, 20, False)
    got = fcu_norm(r.apply([1500, v, 1500, 1500])[1], 988, 1500, 2011, 20, True)
    worst = max(worst, abs(got - want))
# остаток — асимметрия калибровки борта: выше центра 491 µs, ниже 492 (0.2 %)
check(f"FCU борта: команда та же, что без реверса (макс. расхождение {worst:.4f} ≤ 0.003)",
      worst <= 0.003)
check("FCU борта БЕЗ компенсации — тангаж зеркален (контрольный опыт)",
      fcu_norm(1300, 988, 1500, 2011, 20, True) > 0 > fcu_norm(1300, 988, 1500, 2011, 20, False))
worst_sim = max(abs(fcu_norm(v, 1000, 1500, 2000, 0, False)
                    - fcu_norm(rev(SIM).apply([1500, v, 1500, 1500])[1], 1000, 1500, 2000, 0, False))
                for v in range(1000, 2001))
check("FCU сима: команда та же (расхождение 0)", worst_sim == 0.0)

# 4. зеркало вокруг TRIM канала, а не 1500
r = rev(dict(BOARD, RC2_TRIM=1510))
check("TRIM 1510: 1400 → 1620 (FCU видит ту же отклонённость)",
      r.apply([1500, 1400, 1500, 1500])[1] == 1620)

# 5. спецзначения не зеркалятся
r = rev(BOARD)
check("RC_RELEASE (0) и RC_NOCHANGE (65535) не зеркалятся",
      r.apply([1500, RC_RELEASE, 1500, 1500])[1] == RC_RELEASE
      and r.apply([1500, RC_NOCHANGE, 1500, 1500])[1] == RC_NOCHANGE)

# 6. не прочитано → None (release), что не хватает — видно
r = rev({'RC1_REVERSED': 0, 'RC2_REVERSED': 1})
check("часть параметров → не готов, apply None",
      not r.ready() and r.apply([1500] * 4) is None)
check("недостающие: RC3/RC4_REVERSED и TRIM реверсного тангажа",
      r.missing() == ['RC3_REVERSED', 'RC4_REVERSED', 'RC2_TRIM'])
check("TRIM нереверсных каналов не требуется (сим готов без TRIM)", rev(SIM).ready())

# 7. реверс газа → отказ (канал-диапазон, FCU зеркалит вокруг середины MIN/MAX)
r = rev(dict(BOARD, RC3_REVERSED=1))
check("реверс газа → apply None (release) и сообщение в describe",
      r.apply([1500] * 4) is None and 'ГАЗА' in r.describe())

# 8. значения приходят и int, и float (ParamEvent/get_parameters: INTEGER|DOUBLE)
r = RcReverse()
for k, v in BOARD.items():
    r.on_param(k, float(v))
check("float-значения параметров понимаются", r.ready() and r.reversed_channels() == [2])
check("чужой параметр игнорируется", r.on_param('ATC_ANGLE_MAX', 20) is False)
check("повтор того же значения — без изменения (лог не спамит)",
      r.on_param('RC2_REVERSED', 1) is False and r.on_param('RC2_REVERSED', 0) is True)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ RC REVERSE OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
