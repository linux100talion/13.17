"""Буфер ориентации: угол на момент кадра интерполяцией (чистый python, без ROS)."""


class AttitudeBuffer:
    """Штампованные отсчёты ориентации → угол НА МОМЕНТ КАДРА интерполяцией между
    двумя соседними отсчётами. Кадр при этом ЖДЁТ следующий отсчёт (≤ 1/темп ATTITUDE).

    Зачем. Полёты lv2_joy_20260829_182126 / ab_soft: шум пути канала вида сверху за
    кадр растёт с высотой 14 → 386 мм (0.3 → 17.5 м), а реплей тех же кадров с
    ИСТИННЫМИ углами Gazebo даёт 6–11 мм на любой высоте — шум целиком от углов.
    Полоса лежит впереди на 1.5·h, её продольный след на земле ∝ дальность²/h ∝ h:
    на 17.5 м рычаг 40 м/рад, и «последнее пришедшее» ATTITUDE (~15–25 Гц, держится
    ступенькой до следующего) даёт те самые сотни мм. Реплей со ступенькой 12.5/25 Гц
    воспроизводит полёт (463/196 мм на 17.5 м); дотяжка гироскопом (`attitude_at`)
    возвращает лишь до 285/122 — ω за интервал успевает измениться; интерполяция
    между отсчётами — уровень «задержка 20 мс» ≈ 70 мм, впятеро чище полёта.
    Штампы MAVROS — время ПРИЁМА (timesync NONE), отсчёт опаздывает на транспорт
    (~15–20 мс по local_position против истины): `latency` сдвигает запрос вперёд.
    Пустой буфер / запрос за концом — удержание последнего (как было)."""

    def __init__(self, keep_sec=6.0):
        self.buf = []            # (t, pitch, roll), по времени
        self.keep = float(keep_sec)

    def push(self, t, pitch, roll):
        self.buf.append((float(t), float(pitch), float(roll)))
        while self.buf and t - self.buf[0][0] > self.keep:
            self.buf.pop(0)

    def newest(self):
        return self.buf[-1][0] if self.buf else None

    def ready(self, t):
        return bool(self.buf) and self.buf[-1][0] >= t

    def at(self, t):
        """(pitch, roll) на момент t — или None, если отсчётов нет вовсе."""
        b = self.buf
        if not b:
            return None
        if t <= b[0][0]:
            return b[0][1], b[0][2]
        if t >= b[-1][0]:
            return b[-1][1], b[-1][2]
        lo, hi = 0, len(b) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if b[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, p0, r0 = b[lo]
        t1, p1, r1 = b[hi]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return p0 + a * (p1 - p0), r0 + a * (r1 - r0)

