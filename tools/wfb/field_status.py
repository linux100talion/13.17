#!/usr/bin/env python3
"""field_status.py — статус борта в поле, на ноуте с наземным Alfa (только радиолинк WFB-ng).

  python3 tools/wfb/field_status.py [-u url] [-i сек] [--mask]

Каждые -i секунд (умолч. 3) одна строка: время | режим | ARMED/disarmed | GPS фикс,
спутники, HDOP | координаты «широта, долгота» (вставляются в Google Карты как есть) |
напряжение батареи | готовность к армингу (флаг полётника «prearm-проверки пройдены»;
если не готов — последние причины PreArm/Arm, полётник повторяет их раз в ~30 с).

  -u url   умолч. tcp:10.5.0.2:5760 — mavlink-router борта через IP-туннель WFB-ng.
           UDP 14551 не трогаем: его держит проброс VirtualBox к Mission Planner.
  --mask   координаты скрыть (для логов/скриншотов/проверок) — печатаются как «скрыто».

Координаты — сырые от приёмника (GPS_RAW_INT), не из EKF: видно, что говорит сам GPS.
Ничего не пишет в полётник и на диск; только запросы сообщений (REQUEST_MESSAGE).
Ctrl-C — выход.
"""
import argparse, time
from pymavlink import mavutil

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('-u', '--url', default='tcp:10.5.0.2:5760')
ap.add_argument('-i', '--interval', type=float, default=3.0)
ap.add_argument('--mask', action='store_true')
args = ap.parse_args()

L = mavutil.mavlink
FIX = {0: 'нет GPS', 1: 'нет фикса', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK float', 6: 'RTK fix'}
PREARM_BIT = L.MAV_SYS_STATUS_PREARM_CHECK
REASON_TTL = 40                                   # с: PreArm повторяются раз в ~30 с


def connect():
    while True:
        try:
            m = mavutil.mavlink_connection(args.url, source_system=250, source_component=201,
                                           autoreconnect=True)
            return m
        except OSError as e:
            print('%s нет связи с бортом (%s) — повтор через 3 с' % (time.strftime('%H:%M:%S'), e),
                  flush=True)
            time.sleep(3)


m = connect()
hb = gps = sys = None
hb_t = 0.0
reasons = {}                                      # текст → время последнего появления
last_check = 0.0                                  # когда последний раз просили prearm-проверку
nxt = 0.0
print('поле: %s, раз в %.0f с; Ctrl-C — выход' % (args.url, args.interval), flush=True)
try:
    while True:
        now = time.time()
        if now >= nxt:
            for mid in (L.MAVLINK_MSG_ID_GPS_RAW_INT, L.MAVLINK_MSG_ID_SYS_STATUS):
                try:
                    m.mav.command_long_send(1, 1, L.MAV_CMD_REQUEST_MESSAGE, 0, mid, 0, 0, 0, 0, 0, 0)
                except OSError:
                    pass
        r = m.recv_match(blocking=True, timeout=0.2)
        if r is not None and r.get_srcSystem() == 1 and r.get_srcComponent() == 1:
            t = r.get_type()
            if t == 'HEARTBEAT' and r.type not in (L.MAV_TYPE_GCS, L.MAV_TYPE_ONBOARD_CONTROLLER):
                hb, hb_t = r, time.time()
            elif t == 'GPS_RAW_INT':
                gps = r
            elif t == 'SYS_STATUS':
                sys = r
            elif t == 'STATUSTEXT' and (r.text.startswith('PreArm') or r.text.startswith('Arm')):
                reasons[r.text.strip()] = time.time()
        if now < nxt:
            continue
        nxt = now + args.interval

        ts = time.strftime('%H:%M:%S')
        if hb is None:
            print('%s | жду полётник…' % ts, flush=True)
            continue
        if now - hb_t > 3:
            print('%s | НЕТ СВЯЗИ с полётником %.0f с — радиолинк? борт включён?' % (ts, now - hb_t),
                  flush=True)
            continue
        armed = bool(hb.base_mode & L.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mavutil.mode_string_v10(hb)
        if gps is not None:
            g = '%s, спутников %d, HDOP %.1f' % (FIX.get(gps.fix_type, gps.fix_type),
                                                  gps.satellites_visible, gps.eph / 100)
            if gps.fix_type >= 2:
                pos = 'скрыто' if args.mask else '%.6f, %.6f' % (gps.lat / 1e7, gps.lon / 1e7)
            else:
                pos = 'координат нет'
        else:
            g, pos = 'GPS ?', 'координат нет'
        volt = '%.1f В' % (sys.voltage_battery / 1000) if sys and sys.voltage_battery > 0 else '? В'
        if armed:
            ready = 'В ВОЗДУХЕ/ЗААРМЛЕН'
        elif sys is None:
            ready = 'арм: ?'
        elif sys.onboard_control_sensors_health & PREARM_BIT:
            ready = 'арм: ГОТОВ'
        else:
            live = [k for k, v in reasons.items() if now - v < REASON_TTL]
            if not live and now - last_check > 10:
                # причин ещё нет — попросить полётник прогнать проверки сейчас (он пришлёт PreArm-тексты)
                m.mav.command_long_send(1, 1, L.MAV_CMD_RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, 0)
                last_check = now
            ready = 'арм: НЕТ — ' + ('; '.join(live) if live else 'запрошена проверка, причина — в следующей строке')
        print('%s | %-9s | %-8s | GPS %s | %s | %s | %s' % (
            ts, mode, 'ARMED' if armed else 'disarmed', g, pos, volt, ready), flush=True)
except KeyboardInterrupt:
    print()
