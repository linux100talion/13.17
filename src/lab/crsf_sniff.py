#!/usr/bin/env python3
"""crsf_sniff.py — поймать кадры CRSF с отвода RC-линии на UART Orin.

Зачем. Приёмник ELRS воткнут в полётник штатно, а его TX-линия ОТВЕТВЛЕНА на RX
хедера Orin (один передающий, два слушающих — pin 10 + RX полётника). Это первый
шаг пути «намерение пилота мимо FCU» (docker/sim/laptop_move.md §5.4): пока нода
пишет rc/override, настоящие стики из телеметрии не видны, и читать их можно
только с провода. Скрипт проверяет ЖЕЛЕЗО и порт: идут ли байты, целы ли кадры,
какие каналы приходят — до всякого софта поверх.

Формат кадра CRSF:  [addr][len][type][payload…][crc8]
  addr 0xC8 — «полётнику» (кадры каналов от приёмника), 0xEA/0xEE — прочие;
  len = сколько байт ПОСЛЕ него (type + payload + crc);
  type 0x16 = RC_CHANNELS_PACKED: 22 байта = 16 каналов по 11 бит, LSB first;
  crc8 — полином 0xD5 (DVB-S2) по type+payload.
Значение канала → µs: us = (v − 992)·5/8 + 1500 (172…1811 → 988…2012).

Бод 420000 — НЕстандартный: pyserial на Linux ставит его через termios2/BOTHER.

Запуск на Orin:  python3 crsf_sniff.py [--ports /dev/ttyTHS1,/dev/ttyTHS2] [--sec 6]
Порт не задан — пробуются оба ttyTHS: какой отзовётся, тот и подключён к пинам 8/10.
"""
import argparse
import sys
import time

CRSF_SYNC = (0xC8, 0xEA, 0xEE, 0xEC)
TYPE_RC = 0x16
NAMES = "roll pitch throttle yaw ch5 ch6 ch7 ch8".split()


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_channels(payload):
    bits = int.from_bytes(payload, 'little')
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


def to_us(v):
    return round((v - 992) * 5 / 8 + 1500)


def sniff(port, baud, sec):
    import serial
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"  {port}: НЕ ОТКРЫЛСЯ — {e}")
        return None
    buf = bytearray()
    stats = {'bytes': 0, 'frames': 0, 'crc_bad': 0, 'types': {}, 'last': None}
    t0 = time.monotonic()
    with ser:
        while time.monotonic() - t0 < sec:
            chunk = ser.read(256)
            if chunk:
                stats['bytes'] += len(chunk)
                buf += chunk
            while len(buf) >= 2:
                if buf[0] not in CRSF_SYNC:
                    del buf[0]
                    continue
                ln = buf[1]
                if not 2 <= ln <= 62:
                    del buf[0]
                    continue
                if len(buf) < ln + 2:
                    break
                frame = bytes(buf[:ln + 2])
                del buf[:ln + 2]
                body, crc = frame[2:-1], frame[-1]
                if crc8(body) != crc:
                    stats['crc_bad'] += 1
                    continue
                stats['frames'] += 1
                t = body[0]
                stats['types'][t] = stats['types'].get(t, 0) + 1
                if t == TYPE_RC and len(body) == 23:
                    stats['last'] = unpack_channels(body[1:])
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ports', default='/dev/ttyTHS1,/dev/ttyTHS2')
    ap.add_argument('--baud', type=int, default=420000)
    ap.add_argument('--sec', type=float, default=6.0)
    args = ap.parse_args()

    print(f"== СНИФЕР CRSF ({args.baud} бод, по {args.sec:g} с на порт) ==")
    found = False
    for port in args.ports.split(','):
        st = sniff(port.strip(), args.baud, args.sec)
        if st is None:
            continue
        if st['bytes'] == 0:
            print(f"  {port}: тишина (0 байт) — не тот пин, нет земли или приёмник не запитан")
            continue
        types = ' '.join(f"0x{t:02X}×{n}" for t, n in sorted(st['types'].items()))
        print(f"  {port}: байт {st['bytes']}, кадров ЦЕЛЫХ {st['frames']}, "
              f"битых CRC {st['crc_bad']}, типы: {types or '—'}")
        if st['frames'] and st['last']:
            found = True
            us = [to_us(v) for v in st['last']]
            print(f"     КАНАЛЫ (µs): " +
                  ' '.join(f"{NAMES[i]}={us[i]}" for i in range(8)))
            print(f"     ch9..16: {' '.join(str(u) for u in us[8:])}")
        elif st['frames']:
            print("     кадры есть, но RC_CHANNELS (0x16) среди них нет")
    print()
    if found:
        print("ИТОГ: ✅ ОТВОД РАБОТАЕТ — кадры каналов приходят на Orin")
    else:
        print("ИТОГ: ⚠️ кадров каналов нет. Проверить: земля общая, пин 10 (RX), "
              "приёмник запитан и связан с пультом, бод 420000")
    return 0 if found else 1


if __name__ == '__main__':
    sys.exit(main())
