"""TiltFilter — крен/тангаж из сырого IMU в обход EKF полётника (чистый python, без ROS).

ЗАЧЕМ (2026-10-01, cmd/att_oracle). Канал вида сверху выпрямлял полосу земли по ориентации EKF
(/mavros/imu/data), а его скорость сама уходит в EKF (vision_vel → EK3_SRC1_VELXY=6). На 5 м
петля раскручивалась: EKF ошибся наклоном → IPM занижает скорость → EKF правится заниженной
скоростью → ошибается сильнее (до 10° по тангажу, vins_init_5.json); полётник выставлял
заказанный угол по испорченной ориентации и полз назад со стиком в упор. Опыт с оракулом
(истина Gazebo вместо EKF) петлю разорвал: EKF < 1° после моста VINS, висение 0.04 м/с. Этот
фильтр — замена оракулу на борту: ориентация канала не зависит ни от EKF, ни от его подсказок.

СХЕМА (Махони, комплементарный на кватернионе тело → мир ENU, оси тела FLU как у MAVROS):
  q̇ = ½ q ⊗ (ω_гиро − b + Kp·e),   e = â × û,   ḃ = −Ki·e
  â — направление удельной силы акселерометра (в покое — «вверх»), û — «вверх» мира в теле по q.
  Kp = 1/τ: наклон по акселерометру подтягивает интеграл гироскопа с постоянной τ (10–20 с).
⚠️ Акселерометр мультикоптера в ДЛИТЕЛЬНОМ РАЗГОНЕ видит тягу, а не гравитацию: поправка тянет
наклон к «нулю» со скоростью 1/τ. Поэтому τ длинная — за секунды разгона ошибка ≲ atan(a/g)·t/τ;
в установившемся висении (и под ветром: сил нет — ускорения нет) акселерометр честен. Поправка
не применяется, когда |a| дальше gate от g (рывки, удары).
Смещение гироскопа учится тем же e (Ki); на земле до арма его можно задать средним гироскопа
(seed_bias) — тогда фильтр с первой секунды полёта без дрейфа.
Выход — (тангаж, крен) в конвенции канала вида сверху: та же формула Эйлера, что берёт
ros_perception из кватерниона MAVROS (тангаж + = нос вниз, ROS)."""
import math

G = 9.80665


class TiltFilter:
    def __init__(self, tau=15.0, bias_tau=60.0, gate=0.15):
        self.kp = 1.0 / tau if tau > 0 else 0.0
        self.ki = (self.kp / bias_tau) if (tau > 0 and bias_tau > 0) else 0.0
        self.gate = float(gate)              # | |a|/g − 1 | больше — без поправки
        self.q = None                        # (w, x, y, z) тело → мир
        self.b = [0.0, 0.0, 0.0]             # смещение гироскопа, рад/с
        self.t = None

    def seed_bias(self, bx, by, bz):
        self.b = [float(bx), float(by), float(bz)]

    def _init(self, ax, ay, az):
        """Начальная ориентация по акселерометру (курс 0): крен/тангаж из «вверх»."""
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.hypot(ay, az))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        self.q = [cr * cp, sr * cp, cr * sp, -sr * sp]

    def update(self, t, gx, gy, gz, ax, ay, az):
        """Отсчёт IMU (штамп, гироскоп рад/с, акселерометр м/с², оси FLU)."""
        an = math.sqrt(ax * ax + ay * ay + az * az)
        if self.q is None:
            if an > 0.5 * G:
                self._init(ax, ay, az)
                self.t = t
            return
        dt = t - self.t
        self.t = t
        if dt <= 0.0 or dt > 0.5:
            return
        w, x, y, z = self.q
        ex = ey = ez = 0.0
        if an > 0.0 and abs(an / G - 1.0) < self.gate and self.kp > 0.0:
            # «вверх» мира (0,0,1) в осях тела: третья строка R(q)
            ux = 2.0 * (x * z - w * y)
            uy = 2.0 * (y * z + w * x)
            uz = w * w - x * x - y * y + z * z
            mx, my, mz = ax / an, ay / an, az / an
            ex, ey, ez = my * uz - mz * uy, mz * ux - mx * uz, mx * uy - my * ux
            self.b[0] -= self.ki * ex * dt
            self.b[1] -= self.ki * ey * dt
            self.b[2] -= self.ki * ez * dt
        ox = gx - self.b[0] + self.kp * ex
        oy = gy - self.b[1] + self.kp * ey
        oz = gz - self.b[2] + self.kp * ez
        # q ← q ⊗ exp(½ ω dt): точный поворот на угол |ω|·dt
        th = math.sqrt(ox * ox + oy * oy + oz * oz) * dt
        if th > 1e-12:
            s = math.sin(th / 2) / (th / dt)
            dw, dx, dy, dz = math.cos(th / 2), ox * s, oy * s, oz * s
            w, x, y, z = (w * dw - x * dx - y * dy - z * dz,
                          w * dx + x * dw + y * dz - z * dy,
                          w * dy - x * dz + y * dw + z * dx,
                          w * dz + x * dy - y * dx + z * dw)
            n = math.sqrt(w * w + x * x + y * y + z * z)
            self.q = [w / n, x / n, y / n, z / n]

    def ready(self) -> bool:
        return self.q is not None

    def pitch_roll(self):
        """(тангаж, крен), рад — формула ros_perception по кватерниону MAVROS."""
        w, x, y, z = self.q
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        return pitch, roll
