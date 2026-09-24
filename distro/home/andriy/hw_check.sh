#!/bin/bash
# hw_check.sh — проверка оборудования реального борта через полётник (MAVLink).
#
#   ~/hw_check.sh [-t сек] [-u url] [секция ...]
#
#   секции   baro compass gps         по умолчанию — все
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
# baro — оба барометра на ОДНОЙ шине I2C2 (distro/doc/HW/Ardupilot_Params/StellarH7V2/
#   hwdef.dat): встроенный DPS310 @0x76 и внешний BMP390 @0x77; там же компас
#   модуля GPS (QMC5883L @0x0D). «Config Error: Baro: unable to initialise driver»
#   = не нашёлся НИ ОДИН барометр = мёртвая шина (2026-09-24: мёртвая при модуле
#   GPS, с одним BMP390 ожила; после пересборки на обесточенном борте с модулем —
#   норма, причина не установлена). Постоянный сдвиг между барометрами не
#   страшен — ArduPilot берёт ноль земли для каждого отдельно; важны шум и
#   чтобы сдвиг не плыл за время замера.
# compass — QMC5883L модуля GPS на той же I2C2 (встроенного компаса у платы нет):
#   найден ли (COMPASS_DEV_ID), здоровье, модуль поля (Украина ~500 мГс; до
#   калибровки и в помещении меньше — WARN, не FAIL), шум, откалиброван ли.
# gps — M9N на UART3: включён ли (GPS1_TYPE), отвечает ли приёмник, фикс,
#   спутники, точность. Без фикса в помещении — WARN. Координаты НЕ печатаются.
#
# Любые перетыкания проводов для этих проверок — ТОЛЬКО на обесточенном борте.

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
[ ${#SECTIONS[@]} -eq 0 ] && SECTIONS=(baro compass gps)

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
    # сам полётник: компонент 1, автопилот и тип — летательный аппарат (на роутере сидят
    # ещё GCS/компаньоны со своими HEARTBEAT, напр. sysid 255 — их не брать)
    if (h and h.get_srcComponent() == 1
            and h.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            and h.type not in (mavutil.mavlink.MAV_TYPE_GCS,
                               mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER)):
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

I2C_DEAD = ('полётник не нашёл НИ ОДНОГО барометра и завис в Config Error — шина I2C2 '
            'мертва (встроенный DPS310 на ней же). ОБЕСТОЧИТЬ борт, снять SDA/SCL внешних '
            'модулей (GPS/компас, BMP390), включить; возвращать модули по одному, каждый '
            'раз на обесточенном борте')

def config_error(texts):
    """Полётник завис в Config Error (шина мертва) — дальше данные датчиков врут."""
    return any('Config Error' in t for t in texts)

def sys_flag(ss, bit, name):
    if not ss:
        res('WARN', 'SYS_STATUS не пришёл — здоровье %s по флагам не проверено' % name)
        return
    s = ss[-1]
    if not s.onboard_control_sensors_present & bit:
        res('FAIL', 'SYS_STATUS: %s не заявлен (present=0)' % name)
    elif not s.onboard_control_sensors_health & bit:
        res('FAIL', 'SYS_STATUS: %s нездоров (healthy=0)' % name)
    else:
        res('OK', 'SYS_STATUS: %s present/enabled/healthy' % name)

def check_baro():
    print('\n== baro: барометры (I2C2), замер %.0f с, борт не трогать ==' % dur)
    got, texts = sample([29, 137, 143, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS], dur)
    cfg = [t for t in dict.fromkeys(texts) if 'Config Error' in t or 'Baro' in t]
    for t in cfg:
        print('       FCU: %s' % t)
    if any('Baro' in t and 'initialise' in t for t in cfg):
        res('FAIL', I2C_DEAD)
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
        if dv is None:
            res('FAIL', 'BARO%d_DEVID: полётник не ответил на запрос параметра' % i)
            continue
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

# ---- compass -------------------------------------------------------------
# QMC5883L @0x0D на I2C2 (stellar_cld.txt «Компас»), находится авто-поиском.
COMPASS_EXPECT = (855297, 'QMC5883L модуля GPS')
COMPASS_ORIENT = 4          # модуль GPS смонтирован стрелкой назад: ROTATION_YAW_180 (stellar_cld.txt)
FIELD_MG = (250, 750)       # |B| в мГс: Украина ~500; до калибровки/у железа меньше
MAG_NOISE_WARN_MG = 15      # СКО модуля поля в покое

def check_compass():
    print('\n== compass: компас (I2C2), замер %.0f с, борт не трогать ==' % dur)
    got, texts = sample([mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
                         mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                         mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS], dur)
    if config_error(texts):
        res('FAIL', I2C_DEAD)
        return
    dv = param('COMPASS_DEV_ID')
    if dv is None:
        res('FAIL', 'COMPASS_DEV_ID: полётник не ответил на запрос параметра')
        return
    if not dv:
        res('FAIL', 'компас не найден (COMPASS_DEV_ID=0): SDA/SCL и питание модуля GPS '
                    '(проверять на обесточенном борте)')
        return
    if int(dv) != COMPASS_EXPECT[0]:
        res('WARN', 'COMPASS_DEV_ID=%d (%s), ждали %d (%s)' % (dv, devid(dv), *COMPASS_EXPECT))
    else:
        res('OK', 'компас %s найден [%s]' % (COMPASS_EXPECT[1], devid(dv)))
    sys_flag(got.get('SYS_STATUS'), mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG, 'mag')

    L = got.get('RAW_IMU', [])
    if len(L) < max(3, dur):
        res('FAIL', 'RAW_IMU: %d сообщений за %.0f с — поле компаса не идёт' % (len(L), dur))
        return
    v = [(x.xmag, x.ymag, x.zmag) for x in L]
    n = [math.sqrt(a * a + b * b + c * c) for a, b, c in v]
    B, sd = statistics.fmean(n), statistics.pstdev(n)
    x, y, z = (statistics.fmean(c) for c in zip(*v))
    line = 'поле x %+.0f y %+.0f z %+.0f мГс, |B| %.0f, шум σ %.1f (n=%d)' % (x, y, z, B, sd, len(n))
    if B < 50 or B > 2000:
        res('FAIL', line + ' — мусор/насыщение датчика')
    elif sd > MAG_NOISE_WARN_MG:
        res('WARN', line + ' — шумит (рядом ток: силовые провода, моторы?)')
    elif not FIELD_MG[0] <= B <= FIELD_MG[1]:
        res('WARN', line + ' — модуль поля вне %d..%d (не откалиброван / железо рядом?)' % FIELD_MG)
    else:
        res('OK', line)
    a = got.get('ATTITUDE', [])
    if a and abs(a[-1].roll) < 0.17 and abs(a[-1].pitch) < 0.17 and z < 0:
        res('WARN', 'борт стоит ровно, а z поля < 0: в северном полушарии поле смотрит вниз '
                    '(z > 0) — ориентация компаса? (COMPASS_ORIENT / COMPASS_AUTO_ROT)')

    ofs = [param('COMPASS_OFS_' + c) for c in 'XYZ']
    print('       COMPASS_USE=%s ORIENT=%s AUTO_ROT=%s EXTERNAL=%s' % tuple(
        param(p) for p in ('COMPASS_USE', 'COMPASS_ORIENT', 'COMPASS_AUTO_ROT', 'COMPASS_EXTERNAL')))
    orient = param('COMPASS_ORIENT')
    if orient is not None and int(orient) != COMPASS_ORIENT:
        res('WARN', 'COMPASS_ORIENT=%d, ждали %d: модуль GPS стоит стрелкой назад (hw.txt) — '
                    'курс будет врать на 180°, если модуль не переставляли' % (orient, COMPASS_ORIENT))
    if all(o == 0 for o in ofs if o is not None):
        res('WARN', 'компас не откалиброван (COMPASS_OFS_* = 0): калибровка в Mission '
                    'Planner — на улице, подальше от железа')
    else:
        res('OK', 'калибровка есть: OFS %s' % ' '.join('%+.0f' % o for o in ofs))

# ---- gps -----------------------------------------------------------------
# Matek M9N-5883 (u-blox NEO-M9N) на UART3 = SERIAL3_PROTOCOL 5 (stellar_cld.txt «Порты»).
GPS_FIX = {0: 'нет GPS', 1: 'нет фикса', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK float', 6: 'RTK fixed'}
GPS_MIN_SATS, GPS_MAX_HDOP = 8, 2.0

def check_gps():
    print('\n== gps: приёмник GPS (UART3), замер %.0f с ==' % dur)
    t = param('GPS1_TYPE')
    if t is None:
        res('FAIL', 'GPS1_TYPE: полётник не ответил на запрос параметра')
        return
    if not t:
        res('FAIL', 'GPS выключен в параметрах (GPS1_TYPE=%s)' % t)
        return
    got, texts = sample([mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                         mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS], dur)
    if config_error(texts):
        res('FAIL', I2C_DEAD + ' (GPS при этом не инициализирован)')
        return
    sys_flag(got.get('SYS_STATUS'), mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS, 'gps')
    L = got.get('GPS_RAW_INT', [])
    if not L:
        res('FAIL', 'GPS_RAW_INT не пришёл за %.0f с' % dur)
        return
    g = L[-1]
    fix, sats = g.fix_type, g.satellites_visible
    hdop = g.eph / 100.0 if g.eph != 65535 else float('nan')
    hacc = getattr(g, 'h_acc', 0)
    acc = ', точность %.1f м' % (hacc / 1000.0) if 0 < hacc < 4294967295 else ''
    line = 'GPS: %s, спутников %d, HDOP %.2f%s (GPS1_TYPE=%d, SERIAL3_PROTOCOL=%s)' % (
        GPS_FIX.get(fix, fix), sats, hdop, acc, t, param('SERIAL3_PROTOCOL'))
    if fix == 0:
        res('FAIL', line + ' — приёмник не отвечает: UART3 TX/RX, питание модуля')
    elif fix < 3:
        res('WARN', line + ' — нет 3D-фикса (в помещении норма; на улице холодный '
                           'старт до ~5 мин)')
    elif sats < GPS_MIN_SATS or not hdop <= GPS_MAX_HDOP:
        res('WARN', line + ' — слабый фикс (мало спутников / большой HDOP)')
    else:
        res('OK', line)

CHECKS = {'baro': check_baro, 'compass': check_compass, 'gps': check_gps}
for s in sections:
    if s not in CHECKS:
        print('[FAIL] нет такой секции: %s (есть: %s)' % (s, ' '.join(CHECKS)))
        fails += 1
        continue
    CHECKS[s]()
print('\nитог: %s' % ('FAIL: %d' % fails if fails else 'без FAIL'))
sys.exit(1 if fails else 0)
PY
