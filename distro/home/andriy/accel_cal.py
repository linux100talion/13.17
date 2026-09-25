#!/usr/bin/env python3
"""accel_cal.py — калибровка акселерометра полётника по MAVLink, без Mission Planner.

  ~/accel_cal.py [-u url] [--fifo путь] [--log путь] [accel|level]

  accel   6 положений (по умолчанию): MAV_CMD_PREFLIGHT_CALIBRATION param5=1 → полётник
          просит положение (COMMAND_LONG MAV_CMD_ACCELCAL_VEHICLE_POS, param1 = 1..6),
          кладём борт, подтверждаем той же командой с тем же param1. Итог — SUCCESS/FAILED.
          По первому положению («ровно») ArduPilot сам ставит AHRS_TRIM_X/Y («Trim OK»).
  level   только горизонт (param5=2, AHRS_TRIM): борт на ровной поверхности.
  -u url  pymavlink, умолч. tcp:127.0.0.1:5760 (TcpServer mavlink-router; с ноута —
          tcp:192.168.0.104:5760)
  --fifo  подтверждения строкой «ok» из FIFO вместо Enter в терминале («quit» — прервать);
          так калибровку ведёт агент, а человек только перекладывает борт
  --log   дублировать вывод в файл

Подтверждать — только когда борт ЛЕЖИТ неподвижно (не в руках); угол ±5–10° не важен.
После SUCCESS — ПЕРЕЗАГРУЗИТЬ полётник («Accels calibrated requires reboot»: до ребута
углы врут на ~1°, проверено 2026-09-25), затем проверить в покое: |a| ≈ 1000 мG, крен и
тангаж ATTITUDE ≈ наклон по датчику минус трим.

Первая калибровка борта 2026-09-25: INS_ACCOFFS 0.009/−0.008/0.067 м/с², INS_ACCSCAL
0.9998/0.9992/0.9981, трим крен 0.09° тангаж 0.93° (плата на раме ~1° носом вверх).
"""
import argparse, os, select, sys, threading, time
from pymavlink import mavutil

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('mode', nargs='?', default='accel', choices=('accel', 'level'))
ap.add_argument('-u', '--url', default='tcp:127.0.0.1:5760')
ap.add_argument('--fifo')
ap.add_argument('--log')
args = ap.parse_args()

L = mavutil.mavlink
POS = {1: 'РОВНО (обычное положение, верхом вверх)', 2: 'НА ЛЕВЫЙ БОК', 3: 'НА ПРАВЫЙ БОК',
       4: 'НОСОМ ВНИЗ (камеру не нагружать)', 5: 'НОСОМ ВВЕРХ',
       6: 'НА СПИНУ (GPS и антенны не нагружать)',
       L.ACCELCAL_VEHICLE_POS_SUCCESS: 'SUCCESS', L.ACCELCAL_VEHICLE_POS_FAILED: 'FAILED'}
logf = open(args.log, 'a', buffering=1) if args.log else None

def log(s):
    line = '%s %s' % (time.strftime('%H:%M:%S'), s)
    print(line, flush=True)
    if logf:
        logf.write(line + '\n')

m = mavutil.mavlink_connection(args.url, source_system=250, source_component=190)
t0 = time.time()
while True:
    h = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if h is None and time.time() - t0 > 10:
        log('нет HEARTBEAT полётника за 10 с (%s)' % args.url)
        sys.exit(2)
    # сам полётник: компонент 1, не GCS и не компаньон (на роутере сидят и они)
    if h and h.get_srcComponent() == 1 and h.type not in (L.MAV_TYPE_GCS,
                                                          L.MAV_TYPE_ONBOARD_CONTROLLER):
        break
S = h.get_srcSystem()
if h.base_mode & L.MAV_MODE_FLAG_SAFETY_ARMED:
    log('ARMED — калибровка запрещена')
    sys.exit(1)
log('полётник sysid %d, disarmed, калибровка %s' % (S, args.mode))

def heartbeat():
    while True:
        m.mav.heartbeat_send(L.MAV_TYPE_GCS, L.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)
threading.Thread(target=heartbeat, daemon=True).start()

# источник подтверждений: FIFO («ok»/«quit») или терминал (Enter / q)
if args.fifo:
    fd = os.open(args.fifo, os.O_RDWR | os.O_NONBLOCK)   # RDWR: нет EOF между писателями
    CONFIRM, QUIT = (b'ok',), (b'quit',)
else:
    fd = sys.stdin.fileno()
    CONFIRM, QUIT = (b'',), (b'q', b'quit')
hint = '«ok» в FIFO' if args.fifo else 'Enter (q — прервать)'

def param(name):
    m.mav.param_request_read_send(S, 1, name.encode(), -1)
    t = time.time()
    while time.time() - t < 2:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if r and r.param_id == name:
            return r.param_value

m.mav.command_long_send(S, 1, L.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                        0, 0, 0, 0, 1 if args.mode == 'accel' else 2, 0, 0)
cur, buf, ok = None, b'', False
t0 = time.time()
while time.time() - t0 < 1800:
    r = m.recv_match(blocking=True, timeout=0.2)
    if r is not None and r.get_srcSystem() == S:
        t = r.get_type()
        if t == 'STATUSTEXT' and not r.text.startswith('PreArm'):
            log('FCU: ' + r.text)
        elif t == 'COMMAND_ACK' and r.command == L.MAV_CMD_PREFLIGHT_CALIBRATION:
            if r.result != L.MAV_RESULT_ACCEPTED:
                log('полётник отказал в калибровке (MAV_RESULT %d)' % r.result)
                sys.exit(1)
            if args.mode == 'level':
                ok = True
                break
        elif t == 'COMMAND_LONG' and r.command == L.MAV_CMD_ACCELCAL_VEHICLE_POS:
            p = int(r.param1)    # полётник повторяет запрос раз в секунду — печатаем смену
            if p in (L.ACCELCAL_VEHICLE_POS_SUCCESS, L.ACCELCAL_VEHICLE_POS_FAILED):
                ok = p == L.ACCELCAL_VEHICLE_POS_SUCCESS
                log('ИТОГ: ' + POS[p])
                break
            if p != cur:
                cur = p
                log('ПОЛОЖЕНИЕ %d/6: %s — положить неподвижно, затем %s' % (p, POS.get(p, '?'), hint))
    if select.select([fd], [], [], 0)[0]:
        try:
            d = os.read(fd, 256)
        except BlockingIOError:
            d = b''
        buf += d
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        line = line.strip()
        if line in QUIT:
            log('ПРЕРВАНО')
            sys.exit(1)
        if line in CONFIRM and cur in range(1, 7):
            m.mav.command_long_send(S, 1, L.MAV_CMD_ACCELCAL_VEHICLE_POS, 0, cur, 0, 0, 0, 0, 0, 0)
            log('подтверждено положение %d, полётник собирает данные — не трогать' % cur)
else:
    log('таймаут 30 мин')
    sys.exit(1)

t = time.time()
while time.time() - t < 2:   # добрать хвост STATUSTEXT («Trim OK», «Calibration successful»)
    r = m.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
    if r and not r.text.startswith('PreArm'):
        log('FCU: ' + r.text)
if not ok:
    sys.exit(1)
names = (['INS_ACCOFFS_' + a for a in 'XYZ'] + ['INS_ACCSCAL_' + a for a in 'XYZ']
         if args.mode == 'accel' else []) + ['AHRS_TRIM_X', 'AHRS_TRIM_Y']
for n in names:
    v = param(n)
    log('%-14s %s' % (n, '%.4f' % v if v is not None else '?'))
log('ГОТОВО. Перезагрузить полётник, затем проверить в покое (|a| ≈ 1000 мG, углы ≈ 0).')
