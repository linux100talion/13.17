#!/usr/bin/env python3
# ============================================================================
# Одноразовая accel-калибровка SITL (через pymavlink, ВНУТРИ контейнера
# simulator). Результат пишется в /root/sitl_state/eeprom.bin — это named
# volume sitl_eeprom, поэтому калибровка переживает fresh-start и повторять её
# не нужно (запускать только после `make clean` / первого подъёма стека).
#
# Зачем: на свежем eeprom ArduCopter режет арм обязательной (ARMING_CHECK 0 её
# НЕ выключает) проверкой "Arm: 3D Accel calibration needed". Простая level-cal
# (PREFLIGHT_CALIBRATION param5=4) проставляет калибровку (дрон в SITL стоит
# ровно). Делать через MAVROS ненадёжно (ACK флапает на низком RTF), поэтому
# идём напрямую к SITL по pymavlink (tcp:5762, SERIAL1).
#
# ВАЖНО: cal принимается (ACK result=0) только когда FCU УСТАКАНИЛСЯ — boot-
# калибровка гиро прошла, EKF сошёлся. Сразу после fresh-start cal отклоняется
# (result=1, TEMPORARILY_REJECTED). Поэтому СНАЧАЛА ждём готовности (латч GUIDED,
# как arm.sh), и калибруем только при result=0.
#
# Запуск:  make sitl-cal   (или CPU=1 make sitl-cal)
# Подробности и история диагностики — docker/sim/todo.txt.
# ============================================================================
import sys, time
from pymavlink import mavutil

CONN = 'tcp:127.0.0.1:5762'   # SERIAL1 SITL (5760 занят mavlink_router'ом)
GUIDED = 4                    # custom_mode ArduCopter GUIDED

def connect():
    m = mavutil.mavlink_connection(CONN, source_system=254)
    m.wait_heartbeat(timeout=30)
    print(f"  связь с SITL (sys={m.target_system})", flush=True)
    return m

def set_guided(m):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, GUIDED, 0,0,0,0,0)

def wait_ready(m, wall_budget=500):
    """Ждём готовности FCU: GUIDED латчится только когда EKF сошёлся (как arm.sh).
    На RTF≈0.07 ~15 sim-сек ≈ 210 wall-сек, берём запас."""
    print("  жду готовности FCU (латч GUIDED = EKF сошёлся)...", flush=True)
    t0 = time.time()
    while time.time() - t0 < wall_budget:
        set_guided(m)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=6)
        if hb and hb.custom_mode == GUIDED:
            print(f"    GUIDED залатчился через {int(time.time()-t0)} wall-сек", flush=True)
            return True
        time.sleep(2)
    print("  ⚠️ не дождались GUIDED в бюджете", flush=True)
    return False

def do_cal(m):
    """Шлём level-cal, возвращаем result ACK (0=ACCEPTED, 1=TEMP_REJECTED)."""
    print("  → PREFLIGHT_CALIBRATION param5=4 (level accel cal)", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0,0,0,0,4,0,0)
    t0 = time.time()
    while time.time() - t0 < 25:
        msg = m.recv_match(type=['COMMAND_ACK','STATUSTEXT'], blocking=True, timeout=25)
        if not msg:
            continue
        if msg.get_type() == 'STATUSTEXT':
            print(f"    ST: {msg.text}", flush=True)
        elif msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
            print(f"    ACK result={msg.result}", flush=True)
            return msg.result
    return None

def accel_needed(m, secs=18):
    """Пробуем арм, ловим причину. True = аксель ещё не откалиброван."""
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    t0 = time.time()
    while time.time() - t0 < secs:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=secs)
        if not msg:
            break
        if 'Accel' in msg.text or 'accel' in msg.text:
            print(f"    arm-проверка: {msg.text}", flush=True); return True
        if 'Arm' in msg.text or 'PreArm' in msg.text:
            print(f"    arm-проверка: {msg.text}", flush=True)
    return False

m = connect()
if not wait_ready(m):
    sys.exit("FCU не пришёл в готовность — калибровка невозможна")

for attempt in (1, 2, 3, 4):
    print(f"=== попытка калибровки {attempt} ===", flush=True)
    res = do_cal(m)
    if res != 0:
        print(f"  cal отклонена (result={res}) — FCU ещё не готов, ждём и повторяем...", flush=True)
        time.sleep(15)
        continue
    print("  cal принята (result=0). Ждём ребут/стабилизацию FCU...", flush=True)
    try:
        m.wait_heartbeat(timeout=25)
    except Exception:
        pass
    time.sleep(8)
    if not accel_needed(m):
        print("✅ accel-калибровка проставлена (3D Accel cal needed больше нет).", flush=True)
        print("   eeprom в /root/sitl_state (volume) — переживёт fresh-start.", flush=True)
        sys.exit(0)
    print("  аксель всё ещё требует калибровки — повтор...", flush=True)

print("❌ не удалось откалибровать аксель.", flush=True)
sys.exit(1)
