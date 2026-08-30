#!/usr/bin/env python3
"""js_probe.py — ГДЕ в /joy лежит тумблер/кнопка пульта (без стека, на хосте).

Читает /dev/input/js0 напрямую (тот же joydev, что и joy_linux_node в
контейнере): печатает число осей/кнопок, начальное состояние и КАЖДОЕ
изменение с меткой времени. Дёргаешь орган управления — видишь его индекс
(`axis[i]` = /joy axes[i], `button[i]` = /joy buttons[i] = config.land_joy 'b<i>').

  python3 src/lab/joystick/js_probe.py [/dev/input/js0] [сек=30]

Квирк TX12 (EdgeTX, классический USB-joystick): в HID-дескрипторе 8 осей, но
две последние — обе `Slider` (CH7 и CH8) → Linux кладёт их на ОДИН код
ABS_THROTTLE → joydev видит 7 осей, а CH8 ДЕРЁТСЯ с CH7 (SF-мастер) за
axes[6]: при разных значениях ось дрожит между ними на частоте репорта.
CH8 для нас непригоден. Кнопки b0..b23 = каналы CH9..CH32 (нажата при
значении канала > 0) → кнопка SA живёт на CH9 = b0 (дефолт BS_LAND_JOY).
"""
import array
import fcntl
import os
import struct
import sys
import time

JSIOCGAXES, JSIOCGBUTTONS, JSIOCGNAME = 0x80016a11, 0x80016a12, 0x80806a13


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else '/dev/input/js0'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    b = array.array('B', [0]); fcntl.ioctl(fd, JSIOCGAXES, b); n_ax = b[0]
    b = array.array('B', [0]); fcntl.ioctl(fd, JSIOCGBUTTONS, b); n_bt = b[0]
    nm = array.array('B', [0] * 128); fcntl.ioctl(fd, JSIOCGNAME, nm)
    name = nm.tobytes().split(b'\0')[0].decode(errors='replace').strip()
    print(f"{dev}: '{name}' axes={n_ax} buttons={n_bt}  (слушаю {dur:.0f} с)",
          flush=True)
    ax, bt = {}, {}
    t0 = time.time()
    t_end = t0 + dur
    while time.time() < t_end:
        try:
            e = os.read(fd, 8)
        except BlockingIOError:
            time.sleep(0.005)
            continue
        _, v, typ, num = struct.unpack('IhBB', e)
        init, typ = typ & 0x80, typ & 0x7f
        if typ == 2:
            if not init and ax.get(num) != v:
                print(f"  {time.time() - t0:7.2f}s  axis[{num}] -> {v / 32767:+.2f}",
                      flush=True)
            ax[num] = v
        elif typ == 1:
            if not init and bt.get(num) != v:
                print(f"  {time.time() - t0:7.2f}s  button[{num}] -> {v}", flush=True)
            bt[num] = v
    print("axes  :", ' '.join(f"a{i}={ax.get(i, 0) / 32767:+.2f}" for i in range(n_ax)))
    print("buttons pressed:", [i for i in range(n_bt) if bt.get(i)])


if __name__ == '__main__':
    main()
