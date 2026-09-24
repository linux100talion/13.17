#!/bin/bash
# hw_check.sh — проверка оборудования реального борта через полётник (MAVLink).
#
#   ~/hw_check.sh [-t сек] [-u url] [секция ...]
#
#   секции   baro                     по умолчанию — все готовые
#   -t сек   длительность замера      умолч. 10
#   -u url   куда цепляться pymavlink умолч. tcp:127.0.0.1:5760 (TcpServer
#            mavlink-router; Mission Planner может сидеть там же — не мешает)
#
# Итог каждой проверки: [ OK ] / [WARN] / [FAIL]. Код выхода: 0 — без FAIL,
# 1 — есть FAIL, 2 — нет связи с полётником (дальше проверять нечего).
#
# Только читает: параметры, телеметрию, STATUSTEXT. Ничего не пишет в полётник
# и не трогает сервисы — если mavlink-router лежит, скажет, как поднять.
#
# baro — оба барометра на ОДНОЙ шине I2C2 (distro/doc/Ardupilot_Params/StellarH7V2/
#   hwdef.dat): встроенный DPS310 @0x76 и внешний BMP390 @0x77; там же компас
#   модуля GPS (QMC5883L @0x0D). «Config Error: Baro: unable to initialise driver»
#   = не нашёлся НИ ОДИН барометр = мёртвая шина (2026-09-24: шину клал модуль
#   GPS, с одним BMP390 всё ожило). Постоянный сдвиг между барометрами не
#   страшен — ArduPilot берёт ноль земли для каждого отдельно; важны шум и
#   чтобы сдвиг не плыл за время замера.

set -euo pipefail

DUR=10
URL="tcp:127.0.0.1:5760"
usage() { sed -n '2,13p' "$0"; exit "${1:-0}"; }
while getopts "t:u:h" o; do
    case $o in
        t) DUR="$OPTARG" ;;
        u) URL="$OPTARG" ;;
        h) usage 0 ;;
        *) usage 1 ;;
    esac
done
shift $((OPTIND - 1))
SECTIONS=("$@")
[ ${#SECTIONS[@]} -eq 0 ] && SECTIONS=(baro)

# ---- связь: USB полётника и mavlink-router ------------------------------
if [ "$URL" = "tcp:127.0.0.1:5760" ]; then
    if [ ! -e /dev/ttyACM0 ]; then
        echo "[FAIL] полётника нет на USB (/dev/ttyACM0): переподключить USB-кабель без BOOT"
        echo "       (после старта он иногда виснет на USB — см. hw.txt), затем:"
        echo "       sudo systemctl reset-failed mavlink-router && sudo systemctl start mavlink-router"
        exit 2
    fi
    if ! systemctl is-active --quiet mavlink-router; then
        echo "[FAIL] mavlink-router не запущен (при выдернутом USB он сдаётся насовсем):"
        echo "       sudo systemctl reset-failed mavlink-router && sudo systemctl start mavlink-router"
        exit 2
    fi
fi

exec python3 - "$URL" "$DUR" "${SECTIONS[@]}" <<'PY'
import math, statistics, sys, time
from pymavlink import mavutil

url, dur, sections = sys.argv[1], float(sys.argv[2]), sys.argv[3:]
fails = 0

def res(level, text):
    global fails
    if level == 'FAIL':
        fails += 1
    print('[%-4s] %s' % (level, text) if level != 'OK' else '[ OK ] %s' % text)

# ---- подключение ---------------------------------------------------------
m = mavutil.mavlink_connection(url, source_system=250)
hb = None
t0 = time.time()
while time.time() - t0 < 10:
    h = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    # компонент 1 и не GCS/роутер — это сам полётник
    if h and h.get_srcComponent() == 1 and h.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        hb = h
        break
if hb is None:
    print('[FAIL] нет HEARTBEAT полётника за 10 с (%s)' % url)
    sys.exit(2)
SYS = hb.get_srcSystem()
m.target_system, m.target_component = SYS, 1
print('полётник: sysid %d, режим %s, %s' % (
    SYS, mavutil.mode_string_v10(hb),
    'ARMED' if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else 'disarmed'))

def param(name):
    m.mav.param_request_read_send(SYS, 1, name.encode(), -1)
    t = time.time()
    while time.time() - t < 2:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if r and r.param_id == name:
            return r.param_value
    return None

def sample(msg_ids, secs, rate=5.0):
    """Опрос REQUEST_MESSAGE с темпом rate (стрим-рейты полётника не трогаем).
    Возвращает {тип: [сообщения]} и список STATUSTEXT."""
    got, texts = {}, []
    t0, nxt = time.time(), 0.0
    while time.time() - t0 < secs:
        if time.time() >= nxt:
            for i in msg_ids:
                m.mav.command_long_send(SYS, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                        0, i, 0, 0, 0, 0, 0, 0)
            nxt = time.time() + 1.0 / rate
        r = m.recv_match(blocking=True, timeout=0.05)
        if r is None:
            continue
        if r.get_type() == 'STATUSTEXT':
            texts.append(r.text)
        elif r.get_srcSystem() == SYS:
            got.setdefault(r.get_type(), []).append(r)
    return got, texts

def devid(v):
    """ArduPilot DEVID: bus_type[0:3] bus[3:8] address[8:16] devtype[16:24]."""
    v = int(v)
    bt = {1: 'I2C', 2: 'SPI', 3: 'UAVCAN', 4: 'SITL', 5: 'MSP', 6: 'SERIAL'}.get(v & 7, '?%d' % (v & 7))
    return '%s%d @0x%02X type 0x%02X' % (bt, (v >> 3) & 0x1F, (v >> 8) & 0xFF, (v >> 16) & 0xFF)

def hpa_to_m(dp, p):
    # производная барометрической формулы у p: dh/dp = 44330/5.255 · p0^(-1/5.255) · p^(1/5.255-1)
    return dp * 44330.0 / 5.255 / 1013.25 * (p / 1013.25) ** (1 / 5.255 - 1)

# ---- baro ----------------------------------------------------------------
# Ожидаемые устройства — stellar_cld.txt («Барометры (I2C2)»), замер 2026-09-23/24.
BARO_EXPECT = {1: (423425, 'встроенный DPS310'), 2: (1341185, 'внешний BMP390')}
BARO_MSG = {1: 'SCALED_PRESSURE', 2: 'SCALED_PRESSURE2', 3: 'SCALED_PRESSURE3'}
NOISE_WARN_M, NOISE_FAIL_M = 0.3, 1.0   # СКО высоты в покое
OFFSET_WARN_HPA = 1.5                   # сдвиг пары: паспортная абс. точность ±1 + ±0.5
DRIFT_WARN_M = 0.5                      # сдвиг пары уплыл за замер

def check_baro():
    print('\n== baro: барометры (I2C2), замер %.0f с, борт не трогать ==' % dur)
    got, texts = sample([29, 137, 143, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS], dur)
    cfg = [t for t in dict.fromkeys(texts) if 'Config Error' in t or 'Baro' in t]
    for t in cfg:
        print('       FCU: %s' % t)
    if any('Baro' in t and 'initialise' in t for t in cfg):
        res('FAIL', 'полётник не нашёл НИ ОДНОГО барометра и завис в Config Error — '
                    'шина I2C2 мертва (встроенный DPS310 на ней же). Снять SDA/SCL '
                    'внешних модулей (GPS/компас, BMP390), обесточить борт, включить; '
                    'возвращать модули по одному')
        return
    ss = got.get('SYS_STATUS')
    if ss:
        s = ss[-1]
        b = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        if not s.onboard_control_sensors_present & b:
            res('FAIL', 'SYS_STATUS: барометр не заявлен (abs_press present=0)')
        elif not s.onboard_control_sensors_health & b:
            res('FAIL', 'SYS_STATUS: барометр нездоров (abs_press healthy=0)')
        else:
            res('OK', 'SYS_STATUS: abs_press present/enabled/healthy')
    else:
        res('WARN', 'SYS_STATUS не пришёл — здоровье по флагам не проверено')

    series = {}
    for i in (1, 2, 3):
        dv = param('BARO%d_DEVID' % i)
        L = got.get(BARO_MSG[i], [])
        exp = BARO_EXPECT.get(i)
        if not dv:
            if exp:
                res('FAIL', 'BARO%d (%s) не найден: BARO%d_DEVID=0' % (i, exp[1], i))
            continue
        name = exp[1] if exp and int(dv) == exp[0] else 'НЕОЖИДАННОЕ устройство'
        if exp and int(dv) != exp[0]:
            res('WARN', 'BARO%d_DEVID=%d (%s), ждали %d (%s)' % (i, dv, devid(dv), exp[0], exp[1]))
        if len(L) < max(3, dur):   # опрос 5 Гц — хотя бы ~1 Гц должно дойти
            res('FAIL', 'BARO%d %s: %d сообщений %s за %.0f с — данные не идут'
                % (i, name, len(L), BARO_MSG[i], dur))
            continue
        ps = [x.press_abs for x in L]
        T = L[-1].temperature / 100.0
        p = statistics.fmean(ps)
        sd = hpa_to_m(statistics.pstdev(ps), p)
        line = 'BARO%d %s [%s]: %.2f гПа, %.1f °C, шум σ %.2f м (n=%d)' % (
            i, name, devid(dv), p, T, sd, len(ps))
        if not 850 <= p <= 1100 or not -20 <= T <= 85:
            res('FAIL', line + ' — давление/температура вне физики')
        elif sd > NOISE_FAIL_M:
            res('FAIL', line + ' — шумит')
        elif sd > NOISE_WARN_M:
            res('WARN', line + ' — шумнее обычного (сквозняк/пропы/вибрация?)')
        else:
            res('OK', line)
        series[i] = L
    bp = param('BARO_PRIMARY')
    if bp is not None:
        print('       BARO_PRIMARY=%d (0 = BARO1)' % bp)

    if 1 in series and 2 in series:
        n = min(len(series[1]), len(series[2]))
        d = [a.press_abs - b.press_abs for a, b in zip(series[1][:n], series[2][:n])]
        p = statistics.fmean(x.press_abs for x in series[1])
        off = statistics.fmean(d)
        h = max(1, n // 3)   # дрейф сдвига: первая треть против последней
        drift = hpa_to_m(abs(statistics.fmean(d[-h:]) - statistics.fmean(d[:h])), p)
        line = 'пара BARO1−BARO2: сдвиг %+.2f гПа (по высоте %+.1f м), уплыл за замер на %.2f м' % (
            off, -hpa_to_m(off, p), drift)
        if abs(off) > OFFSET_WARN_HPA:
            res('WARN', line + ' — сдвиг больше паспортной точности пары')
        elif drift > DRIFT_WARN_M:
            res('WARN', line + ' — сдвиг плывёт (прогрев платы? DPS310 греется от FC)')
        else:
            res('OK', line)

CHECKS = {'baro': check_baro}
for s in sections:
    if s not in CHECKS:
        print('[FAIL] нет такой секции: %s (есть: %s)' % (s, ' '.join(CHECKS)))
        fails += 1
        continue
    CHECKS[s]()
print('\nитог: %s' % ('FAIL: %d' % fails if fails else 'без FAIL'))
sys.exit(1 if fails else 0)
PY
