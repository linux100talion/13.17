#!/usr/bin/env python3
"""Парсер CRSF — ЧИСТЫЙ (ноль импортов rclpy/serial), поэтому проверяем офлайн.

Откуда байты. Приёмник ELRS воткнут в полётник штатно, а его TX-линия ОТВЕТВЛЕНА
на RX хедера Orin (`/dev/ttyTHS1`, 420000): один передающий, два слушающих. Это
путь «намерение пилота МИМО FCU» (docker/sim/laptop_move.md §5.4) — пока нода
пишет rc/override, настоящие стики из телеметрии не видны (там эхо нашей же
команды), и прочитать их можно только с провода.

Кадр:  [addr][len][type][payload…][crc8]
  addr  — 0xC8 «полётнику» (кадры от приёмника), 0xEA/0xEE/0xEC — прочие адреса;
  len   — сколько байт ПОСЛЕ него (type + payload + crc), 2..62;
  crc8  — полином 0xD5 (DVB-S2) по type+payload.
Типы, которые нас касаются:
  0x16 RC_CHANNELS_PACKED — 22 байта: 16 каналов по 11 бит, LSB first (~197 Гц);
  0x14 LINK_STATISTICS    — качество линка (~10 Гц), 10 байт.

Значение канала → µs: us = (v − 992)·5/8 + 1500 (172…1811 → 988…2012).
Замер на борту 2026-09-23: 1242 целых кадра за 6 с, 0 битых CRC.
"""
CRSF_ADDRS = (0xC8, 0xEA, 0xEE, 0xEC)
TYPE_RC_CHANNELS = 0x16
TYPE_LINK_STATS = 0x14
CH_COUNT = 16
# Границы кадра: len считает type+payload+crc, максимум по спеке — 62.
LEN_MIN, LEN_MAX = 2, 62


def crc8(data) -> int:
    """CRC8/DVB-S2 (полином 0xD5) — как в CRSF, по type+payload."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def to_us(v: int) -> int:
    """Сырое значение канала CRSF → µs привычной RC-конвенции (центр 1500)."""
    return round((v - 992) * 5 / 8 + 1500)


def unpack_channels(payload) -> list:
    """22 байта RC_CHANNELS_PACKED → 16 каналов (сырые 11-битные значения)."""
    bits = int.from_bytes(bytes(payload), 'little')
    return [(bits >> (11 * i)) & 0x7FF for i in range(CH_COUNT)]


def unpack_link(payload) -> dict:
    """LINK_STATISTICS → то, что нужно человеку: качество и уровень аплинка.
    rssi отдаётся в дБм (в кадре лежит положительное число = |дБм|)."""
    if len(payload) < 10:
        return {}
    return {'lq': payload[2], 'rssi': -payload[0], 'snr': payload[3] - 256
            if payload[3] > 127 else payload[3]}


class CrsfParser:
    """Побайтовый разбор потока: feed(bytes) → список (type, payload) ЦЕЛЫХ кадров.

    Синхронизация без состояния «ищем заголовок»: не подошёл адрес, длина или CRC —
    сдвигаемся на байт и пробуем снова. На мусоре это теряет кадр, но никогда не
    залипает; счётчики ok/crc_bad/resync видно снаружи (диагностика провода).
    """

    def __init__(self, max_buf: int = 4096):
        self.buf = bytearray()
        self.max_buf = max_buf
        self.ok = 0
        self.crc_bad = 0
        self.resync = 0

    def feed(self, chunk) -> list:
        if chunk:
            self.buf += chunk
        out = []
        while len(self.buf) >= 2:
            if self.buf[0] not in CRSF_ADDRS:
                del self.buf[0]
                self.resync += 1
                continue
            ln = self.buf[1]
            if not LEN_MIN <= ln <= LEN_MAX:
                del self.buf[0]
                self.resync += 1
                continue
            if len(self.buf) < ln + 2:
                break                            # кадр ещё не дочитан
            frame = bytes(self.buf[:ln + 2])
            body, crc = frame[2:-1], frame[-1]
            if crc8(body) != crc:
                del self.buf[0]                  # НЕ съедаем весь кадр: битой могла
                self.crc_bad += 1                # быть сама длина — ищем дальше
                continue
            del self.buf[:ln + 2]
            self.ok += 1
            out.append((body[0], body[1:]))
        # Защита от бесконечного роста — ПОСЛЕ разбора и только для ОСТАТКА: срезать
        # до разбора значило бы молча терять кадры на большом чанке (поймано тестом
        # на реальном захвате: 1192 байта и ~40 кадров исчезали без следа).
        if len(self.buf) > self.max_buf:
            del self.buf[:-self.max_buf]
        return out
