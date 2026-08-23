#!/usr/bin/env python3
# ============================================================================
# HudRenderer — отрисовка debug-HUD (зрелость VINS/фичи/оси) на кадре.
#
# БЕЗ ROS-импортов сознательно: один и тот же код рисует
#   - живой FPV-оверлей в openhd_streamer (колбэки ноды кормят set_*-методы);
#   - пост-рендер scene_hud.mp4 из bag (src/lab/hud_video.py кормит те же
#     методы сообщениями из записи) — картинка пиксельно та же, что видел
#     бы пилот в OpenHD, расхождение стало бы враньём разбора.
#
# Контракт времени: все set_* принимают t = «часы вызывающего» (node clock у
# стримера: sim в симе, wall на Orin; sim-штампы сообщений у пост-рендера).
# draw(frame, now) меряет возрасты в этих же часах → пороги RTF-независимы.
# Протухший источник гаснет сам (баннер гейта — 3 с, режим — 5 с, CMD — 2 с);
# чего в источниках нет — того нет и на экране (честная деградация: на Orin
# без лётной ноды нет баннера, в bag без /feature нет строки FEAT).
#
# Геометрия задана для эталонного кадра 1280×720 и масштабируется k=w/1280:
# на любом разрешении HUD занимает ту же долю кадра, что в FPV-даунлинке.
# ============================================================================
import collections

import cv2

FONT = cv2.FONT_HERSHEY_SIMPLEX
# BGR-палитра HUD; баннер гейта заливается цветом состояния, текст на нём чёрный
HUD_GREEN = (60, 200, 60)
HUD_YELLOW = (0, 210, 240)
HUD_RED = (50, 50, 230)
HUD_WHITE = (235, 235, 235)
HUD_SCENE = (0, 255, 255)


class HudRenderer:
    def __init__(self):
        self.status = {}              # разобранный /mission/status (k=v)
        self.status_t = None
        self.odom_times = collections.deque(maxlen=64)   # приходы /odometry
        self.feat_n, self.feat_t = 0, None
        self.feat_pts = None          # [(u,v)] фич трекера — зелёные точки
        self.fcu_mode, self.fcu_armed, self.fcu_t = "", False, None
        self.cmd_roll, self.cmd_pitch, self.cmd_t = 0.0, 0.0, None
        self.drift = None             # (норма поправки м, время прихода)
        self.scene = ""               # метка NN2

    # --- питание кэшей (стример — из колбэков, пост-рендер — из bag) ---
    def set_status(self, line: str, t: float) -> None:
        self.status = dict(p.split("=", 1) for p in line.split() if "=" in p)
        self.status_t = t

    def add_odom(self, t: float) -> None:
        self.odom_times.append(t)

    def set_feat(self, n: int, t: float, pts=None) -> None:
        """pts — [(u, v), ...] отслеживаемых фич В КООРДИНАТАХ РИСУЕМОГО КАДРА
        (масштабирует вызывающий: стример рисует на полном кадре — 1:1,
        hud_video после resize домножает на свой коэффициент). None = только
        счётчик FEAT, без точек."""
        self.feat_n, self.feat_t = n, t
        self.feat_pts = pts

    def set_state(self, mode: str, armed: bool, t: float) -> None:
        self.fcu_mode, self.fcu_armed, self.fcu_t = mode, armed, t

    def set_cmd_roll(self, x: float, t: float) -> None:
        # /flow_dbg: vector.x = PWM-смещение крена (rc.roll − центр) от стека
        self.cmd_roll, self.cmd_t = x, t

    def set_cmd_pitch(self, x: float) -> None:
        self.cmd_pitch = x

    def set_drift(self, x: float, y: float, z: float, t: float) -> None:
        self.drift = ((x * x + y * y + z * z) ** 0.5, t)

    def set_scene(self, s: str) -> None:
        self.scene = s

    # --- отрисовка ---
    def _line(self, frame, k, y, text, color, scale=0.8, fill=None):
        """Строка HUD на подложке (читаемость поверх любой сцены); вернёт next y."""
        (tw, th), base = cv2.getTextSize(text, FONT, scale * k,
                                         max(1, round(2 * k)))
        x = round(10 * k)
        pad = round(6 * k)
        cv2.rectangle(frame, (x - pad, y - th - pad),
                      (x + tw + pad, y + base + round(4 * k)),
                      (0, 0, 0) if fill is None else fill, -1)
        cv2.putText(frame, text, (x, y), FONT, scale * k,
                    color if fill is None else (0, 0, 0), max(1, round(2 * k)))
        return y + th + base + round(18 * k)

    def draw(self, frame, now: float) -> None:
        k = frame.shape[1] / 1280.0
        # 0) фичи VINS-трекера — то, за что реально цепляется одометрия.
        # /feature идёт 10 Гц против ~30 у камеры — точки «залипают» на 2-3
        # кадра (та же философия, что рамки NN); протухли (>0.5 с) — гаснут
        # раньше строки FEAT: точки врут быстрее счётчика. Рисуются ПОД
        # текстовым блоком, чтобы не портить читаемость строк.
        if (self.feat_pts and self.feat_t is not None
                and now - self.feat_t < 0.5):
            r = max(2, round(3 * k))
            for u, v in self.feat_pts:
                cv2.circle(frame, (round(u), round(v)), r, (0, 255, 0), -1)
        y = round(34 * k)
        # 1) баннер гейта — правда лётной ноды, тухнет за 3 с без /mission/status
        if self.status_t is not None and now - self.status_t < 3.0:
            st = self.status.get("st", "")
            why = self.status.get("why", "-")
            if st == "READY":
                y = self._line(frame, k, y, "VINS READY", HUD_GREEN,
                               scale=1.0, fill=HUD_GREEN)
            elif st == "WAIT":
                y = self._line(frame, k, y, f"VINS WAIT ({why})", HUD_YELLOW,
                               scale=1.0, fill=HUD_YELLOW)
            else:
                y = self._line(frame, k, y, f"NO VINS ({why})", HUD_RED,
                               scale=1.0, fill=HUD_RED)
        # 2) режим FCU + armed
        if self.fcu_t is not None and now - self.fcu_t < 5.0:
            arm = "ARM" if self.fcu_armed else "DISARM"
            y = self._line(frame, k, y, f"{self.fcu_mode} {arm}", HUD_WHITE)
        # 3) /odometry: Гц (окно 3 с) + возраст; красный ODO -- = VINS без init
        if self.odom_times:
            age = now - self.odom_times[-1]
            win = [t for t in self.odom_times if now - t < 3.0]
            hz = ((len(win) - 1) / (win[-1] - win[0])
                  if len(win) >= 2 and win[-1] > win[0] else 0.0)
            col = (HUD_GREEN if age < 0.5 else
                   HUD_YELLOW if age < 1.5 else HUD_RED)
            y = self._line(frame, k, y, f"ODO {hz:4.1f}Hz {age:4.1f}s", col)
        else:
            y = self._line(frame, k, y, "ODO --", HUD_RED)
        # 4) фичи трекера: замолк при живой камере — это ЧП, красним
        if self.feat_t is not None:
            if now - self.feat_t < 3.0:
                y = self._line(frame, k, y, f"FEAT {self.feat_n}", HUD_WHITE)
            else:
                y = self._line(frame, k, y, "FEAT --", HUD_RED)
        # 5) PWM-смещения крена/тангажа от стека (/flow_dbg, /flow_dbg2)
        if self.cmd_t is not None and now - self.cmd_t < 2.0:
            y = self._line(frame, k, y,
                           f"CMD R{self.cmd_roll:+04.0f} "
                           f"P{self.cmd_pitch:+04.0f}", HUD_WHITE)
        # 6) поправка NN1: засечки редкие, старше 10 с — показываем возраст
        if self.drift is not None:
            d, t = self.drift
            age = now - t
            txt = f"DRIFT {d:.2f}m" + (f" ({age:.0f}s)" if age > 10.0 else "")
            y = self._line(frame, k, y, txt, HUD_WHITE)
        # 7) семантика сцены NN2 (бывший одинокий баннер)
        if self.scene:
            self._line(frame, k, y, f"scene: {self.scene}", HUD_SCENE)
