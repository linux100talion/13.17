#!/usr/bin/env python3
"""Юнит-тест парсера CRSF (мостик пульта на борту, 2026-09-23). Чистый python.

Проверяет то, на чём стоит живой пульт реального борта: сборку/разбор кадра, CRC,
пересинхронизацию на мусоре, склейку кадра из двух кусков, перевод в µs — и, главное,
НАСТОЯЩИЙ ЗАХВАТ с провода (data/crsf_sample.bin, 1 с с /dev/ttyTHS1 борта
2026-09-23, отвод TX приёмника → RX хедера Orin). Синтетика ловит логику, захват
ловит расхождение с железом: порядок бит, адрес, реальные типы кадров.

Запуск:  python3 src/control/test/test_crsf.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.infrastructure.crsf import (CrsfParser, TYPE_LINK_STATS,   # noqa: E402
                                             TYPE_RC_CHANNELS, crc8, to_us,
                                             unpack_channels, unpack_link)

SAMPLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'crsf_sample.bin')
results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def pack_channels(vals):
    """16 сырых значений → 22 байта payload (обратная unpack_channels)."""
    bits = 0
    for i, v in enumerate(vals):
        bits |= (v & 0x7FF) << (11 * i)
    return bits.to_bytes(22, 'little')


def frame(ftype, payload, addr=0xC8):
    body = bytes([ftype]) + bytes(payload)
    return bytes([addr, len(body) + 1]) + body + bytes([crc8(body)])


# 1. Перевод значений в µs — границы и центр
check("to_us: 992 → 1500 (центр)", to_us(992) == 1500)
check("to_us: 172 → 988 (минимум)", to_us(172) == 988)
check("to_us: 1811 → 2012 (максимум)", to_us(1811) == 2012)

# 2. Round-trip: упаковали 16 каналов → разобрали те же
vals = [172, 992, 1811, 500, 1000, 1500, 200, 1800] + [992] * 8
got = unpack_channels(pack_channels(vals))
check("16 каналов round-trip (11 бит, LSB first)", got == vals)

# 3. Целый кадр разбирается
p = CrsfParser()
out = p.feed(frame(TYPE_RC_CHANNELS, pack_channels(vals)))
check("кадр 0x16 разобран", len(out) == 1 and out[0][0] == TYPE_RC_CHANNELS)
check("payload кадра = 22 байта", len(out[0][1]) == 22)
check("счётчик ok=1, битых нет", p.ok == 1 and p.crc_bad == 0)

# 4. Битый CRC не проходит и НЕ считается кадром
p = CrsfParser()
f = bytearray(frame(TYPE_RC_CHANNELS, pack_channels(vals)))
f[-1] ^= 0xFF
out = p.feed(bytes(f))
check("кадр с битым CRC отвергнут", out == [] and p.ok == 0 and p.crc_bad >= 1)

# 5. Мусор перед кадром — пересинхронизация, кадр всё равно найден
p = CrsfParser()
out = p.feed(b'\x00\x01\x02\xff' + frame(TYPE_RC_CHANNELS, pack_channels(vals)))
check("мусор перед кадром: пересинхронизировались", len(out) == 1 and p.resync > 0)

# 6. Кадр, разорванный между чтениями, склеивается
p = CrsfParser()
f = frame(TYPE_RC_CHANNELS, pack_channels(vals))
first = p.feed(f[:10])
second = p.feed(f[10:])
check("разорванный кадр склеен из двух feed", first == [] and len(second) == 1)

# 7. LINK_STATISTICS разбирается (rssi отдаём в дБм, отрицательным)
p = CrsfParser()
link_payload = bytes([85, 90, 100, 5, 0, 2, 3, 80, 99, 4])
out = p.feed(frame(TYPE_LINK_STATS, link_payload))
link = unpack_link(out[0][1])
check("link stats: LQ 100 %, RSSI −85 дБм", link['lq'] == 100 and link['rssi'] == -85)

# 8. НАСТОЯЩИЙ ЗАХВАТ С ПРОВОДА (1 с, борт 2026-09-23)
raw = open(SAMPLE, 'rb').read()
p = CrsfParser()
frames = p.feed(raw)
types = {}
last_rc = None
for ftype, payload in frames:
    types[ftype] = types.get(ftype, 0) + 1
    if ftype == TYPE_RC_CHANNELS and len(payload) == 22:
        last_rc = unpack_channels(payload)
print(f"     захват: {len(raw)} байт → кадров {p.ok}, битых CRC {p.crc_bad}, "
      f"resync {p.resync}, типы {[hex(t) for t in sorted(types)]}")
check("захват: кадры каналов есть (≈197 Гц)", types.get(TYPE_RC_CHANNELS, 0) > 100)
check("захват: статистика линка есть (≈10 Гц)", types.get(TYPE_LINK_STATS, 0) > 0)
check("захват: НИ ОДНОГО битого CRC (провод чист)", p.crc_bad == 0)
# Захват начат в произвольный момент, поэтому хвост первого кадра отбрасывается —
# допустимо меньше длины кадра (26 байт). Больше означало бы дыры В ПОТОКЕ.
check("захват: пересинхронизация только на входе в поток (< 26 байт)", p.resync < 26)
check("захват: кадры не теряются (≈200 за секунду)", 150 < p.ok < 260)
check("захват: 16 каналов", last_rc is not None and len(last_rc) == 16)
us = [to_us(v) for v in last_rc]
check("захват: стики в диапазоне 880…2020 µs",
      all(880 <= u <= 2020 for u in us[:4]))
print(f"     каналы захвата (µs): {us[:8]} / {us[8:]}")

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ CRSF OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
