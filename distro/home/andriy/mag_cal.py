#!/usr/bin/env python3
"""mag_cal.py — калибровка компаса полётника по MAVLink (onboard mag cal), без Mission Planner.

  ~/mag_cal.py [-u url] [--save] [--delay сек] [--say] [--log путь] [--timeout сек]

  Без --save — РЕПЕТИЦИЯ: полётник считает калибровку и шлёт отчёт, но ничего не пишет
          (autosave=0, в конце DO_CANCEL_MAG_CAL). Так проверяют процедуру и связь в комнате:
          поле там искажено (арматура, проводка, техника), поправки из комнаты на улице вредят.
  --save  autosave=1: полётник сам пишет COMPASS_OFS/DIA/ODI/SCALE и — при COMPASS_AUTO_ROT=2 —
          COMPASS_ORIENT по найденной ориентации. Только на улице, подальше от железа и машин.
  --delay задержка старта (умолч. 2): успеть взять борт в руки и отойти от клавиатуры
  --say   озвучивать прогресс голосом (spd-say, ноут): «старт», каждые 10 %, итог — крутящему
          борт некогда смотреть в экран
  -u url  умолч. tcp:127.0.0.1:5760 (на борту); с ноута по радио — tcp:10.5.0.2:5760 (туннель
          WFB-ng, на улице другого пути нет), дома по Wi-Fi — tcp:192.168.0.104:5760

Как крутить: после «СТАРТ» непрерывно и ЭНЕРГИЧНО (переворот за 2–3 с, без пауз) вращать борт так, чтобы каждая сторона
(верх, низ, нос, хвост, бока) побывала и вверху, и внизу — «шесть граней + повороты вокруг
каждой оси». Прогресс — MAG_CAL_PROGRESS (% и покрытие 80 секторов сферы); отчёт —
MAG_CAL_REPORT: fitness (RMS невязки, мГс; порог COMPASS_CAL_FIT 16), поправки, масштаб,
ориентация и уверенность в ней. Ctrl-C — отменить калибровку в полётнике.
Моторы не крутятся, пропы сняты не обязательно; батарея — как в полёте (ток силовой не течёт).
"""
import argparse, shutil, subprocess, sys, threading, time
from pymavlink import mavutil

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('-u', '--url', default='tcp:127.0.0.1:5760')
ap.add_argument('--save', action='store_true')
ap.add_argument('--log')
ap.add_argument('--timeout', type=float, default=600)
ap.add_argument('--delay', type=int, default=2)
ap.add_argument('--say', action='store_true')
args = ap.parse_args()

L = mavutil.mavlink
STATUS = {0: 'NOT_STARTED', 1: 'WAITING_TO_START', 2: 'RUNNING_STEP_ONE', 3: 'RUNNING_STEP_TWO',
          4: 'SUCCESS', 5: 'FAILED', 6: 'BAD_ORIENTATION', 7: 'BAD_RADIUS'}
ROT = {0: 'NONE', 1: 'YAW_45', 2: 'YAW_90', 3: 'YAW_135', 4: 'YAW_180', 5: 'YAW_225', 6: 'YAW_270',
       7: 'YAW_315', 8: 'ROLL_180', 12: 'PITCH_180', 24: 'PITCH_90', 25: 'PITCH_270'}
logf = open(args.log, 'a', buffering=1) if args.log else None

def log(s):
    line = '%s %s' % (time.strftime('%H:%M:%S'), s)
    print(line, flush=True)
    if logf:
        logf.write(line + '\n')

SAY = shutil.which('spd-say') if args.say else None

def say(text):
    if SAY:
        subprocess.Popen([SAY, '-l', 'ru', text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

m = mavutil.mavlink_connection(args.url, source_system=250, source_component=190)
t0 = time.time()
while True:
    h = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if h is None and time.time() - t0 > 10:
        log('нет HEARTBEAT полётника за 10 с (%s)' % args.url)
        sys.exit(2)
    if h and h.get_srcComponent() == 1 and h.type not in (L.MAV_TYPE_GCS,
                                                          L.MAV_TYPE_ONBOARD_CONTROLLER):
        break
S = h.get_srcSystem()
if h.base_mode & L.MAV_MODE_FLAG_SAFETY_ARMED:
    log('ARMED — калибровка запрещена')
    sys.exit(1)

def heartbeat():
    while True:
        m.mav.heartbeat_send(L.MAV_TYPE_GCS, L.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)
threading.Thread(target=heartbeat, daemon=True).start()

def param(name):
    m.mav.param_request_read_send(S, 1, name.encode(), -1)
    t = time.time()
    while time.time() - t < 3:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if r and r.param_id == name:
            return r.param_value

def cancel():
    m.mav.command_long_send(S, 1, L.MAV_CMD_DO_CANCEL_MAG_CAL, 0, 0, 0, 0, 0, 0, 0, 0)

log('полётник sysid %d, disarmed; %s' % (S, 'СОХРАНЯТЬ (--save)' if args.save
                                        else 'РЕПЕТИЦИЯ — ничего не пишем'))
# MAG_CAL_PROGRESS/REPORT идут в потоке EXTRA3, а его включает себе только MP: без этого
# полётник калибрует молча (2026-09-25: 2 мин вращения вслепую). Интервал живёт до ребута FCU.
for mid, us in ((L.MAVLINK_MSG_ID_MAG_CAL_PROGRESS, 250000), (L.MAVLINK_MSG_ID_MAG_CAL_REPORT, 1000000)):
    m.mav.command_long_send(S, 1, L.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, us, 0, 0, 0, 0, 0)
# param1 маска компасов (0 = все), 2 повтор при провале, 3 autosave, 4 задержка старта, 5 autoreboot
m.mav.command_long_send(S, 1, L.MAV_CMD_DO_START_MAG_CAL, 0,
                        0, 0, 1 if args.save else 0, args.delay, 0, 0, 0)
last_pct, rep, t0, started = {}, {}, time.time(), False
try:
    while time.time() - t0 < args.timeout:
        r = m.recv_match(blocking=True, timeout=0.5)
        if r is None or r.get_srcSystem() != S:
            continue
        t = r.get_type()
        if t == 'COMMAND_ACK' and r.command == L.MAV_CMD_DO_START_MAG_CAL:
            if r.result != L.MAV_RESULT_ACCEPTED:
                log('полётник отказал в калибровке (MAV_RESULT %d)' % r.result)
                sys.exit(1)
            log('СТАРТ через %d с — крутить борт во все стороны, энергично и без пауз' % args.delay)
            say('калибровка компаса, старт через %d секунд' % args.delay)
        elif t == 'STATUSTEXT' and not r.text.startswith('PreArm'):
            log('FCU: ' + r.text)
        elif t == 'MAG_CAL_PROGRESS':
            pct = r.completion_pct
            if r.cal_status >= 2 and not started:
                started = True
                say('крутите')
            if pct // 10 != last_pct.get(r.compass_id, -1) // 10:
                last_pct[r.compass_id] = pct
                cov = sum(bin(b).count('1') for b in r.completion_mask)
                log('компас %d: %3d %%, %s, покрыто секторов %d/80' % (
                    r.compass_id, pct, STATUS.get(r.cal_status, r.cal_status), cov))
                if pct >= 10:
                    say('%d' % (pct // 10 * 10))
        elif t == 'MAG_CAL_REPORT':
            rep[r.compass_id] = r
            say('готово, можно класть. %s' % ('успех' if r.cal_status == 4 else 'неудача'))
            log('ОТЧЁТ компас %d: %s, fitness %.1f мГс (порог %s)' % (
                r.compass_id, STATUS.get(r.cal_status, r.cal_status), r.fitness,
                param('COMPASS_CAL_FIT')))
            log('  поправки OFS %+.0f %+.0f %+.0f мГс (предел COMPASS_OFFS_MAX %s)' % (
                r.ofs_x, r.ofs_y, r.ofs_z, param('COMPASS_OFFS_MAX')))
            log('  DIA %.3f %.3f %.3f  ODI %+.3f %+.3f %+.3f  масштаб %.3f' % (
                r.diag_x, r.diag_y, r.diag_z, r.offdiag_x, r.offdiag_y, r.offdiag_z,
                r.scale_factor))
            log('  ориентация: была %s, найдена %s, уверенность %.1f; сохранено: %s' % (
                ROT.get(r.old_orientation, r.old_orientation),
                ROT.get(r.new_orientation, r.new_orientation),
                r.orientation_confidence, 'ДА' if r.autosaved else 'нет'))
            break
    else:
        log('таймаут %.0f с — отменяю' % args.timeout)
        say('время вышло, калибровка отменена')
        cancel()
        sys.exit(1)
except KeyboardInterrupt:
    log('Ctrl-C — отменяю калибровку в полётнике')
    cancel()
    sys.exit(1)

if not args.save:
    cancel()     # отчёт есть, в полётнике не висит «ждёт accept»
    log('РЕПЕТИЦИЯ окончена, параметры не тронуты')
else:
    time.sleep(1)
    for n in ('COMPASS_OFS_X', 'COMPASS_OFS_Y', 'COMPASS_OFS_Z', 'COMPASS_DIA_X',
              'COMPASS_SCALE', 'COMPASS_ORIENT'):
        v = param(n)
        log('%-14s %s' % (n, '%.3f' % v if v is not None else '?'))
    log('ГОТОВО. Перезагрузить полётник, затем hw_check.sh compass (|B| ≈ 500 мГс, калибровка есть).')
ok = rep and all(x.cal_status == 4 for x in rep.values())
sys.exit(0 if ok else 1)
