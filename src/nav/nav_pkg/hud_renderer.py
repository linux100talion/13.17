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
# Причина брака кадра IPM (ipmf= в /mission/status) — та же таблица, что
# FlowEstimator.ipm_fail. ⚠️ ТОЛЬКО ASCII: Hershey-шрифт OpenCV кириллицу не
# рисует (вышли бы «?»), поэтому подписи английские, как и остальной HUD.
IPM_FAIL = {0: "OK", 1: "ALT GATE", 2: "NO WINDOW", 3: "WARP OOB",
            4: "FEW PTS", 5: "FEW LK", 6: "OFF", 7: "NO REF"}


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
        self.ipm_hist = collections.deque(maxlen=128)   # (t, ipm_ok) из статуса

    # --- питание кэшей (стример — из колбэков, пост-рендер — из bag) ---
    def set_status(self, line: str, t: float) -> None:
        self.status = dict(p.split("=", 1) for p in line.split() if "=" in p)
        self.status_t = t
        # доля годных кадров IPM за окно: мгновенный код скачет (гейт у земли
        # открывается и закрывается по дребезгу высоты — прогон 185921: 10%
        # кадров годны, и по одному кадру этого не увидеть)
        if "ipm" in self.status:
            try:
                self.ipm_hist.append((t, int(self.status["ipm"])))
            except ValueError:
                pass

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
            # 1a) статус борта — ПОСТОЯННЫЙ баннер (машина состояний):
            #   EKF WARMUP (жёлт) → EKF READY - TAKEOFF OK (зел) → после
            #   арма ARMED (зел) → после дизарма снова READY/WARMUP по ekf=.
            # До взлёта VINS мёртв по построению (нет параллакса) и гейт ниже
            # всегда красный — пилоту нужен свой сигнал готовности борта.
            # ekf= — тот же критерий, каким WaitEkfPos пускает арм (свежий
            # local_position). В полёте показываем ARMED, а не ekf: после
            # GPS-kill local_position молчит штатно — WARMUP в воздухе врал
            # бы. Старые bag без ekf= — баннера нет (честная деградация).
            ekf = self.status.get("ekf")
            if ekf is not None:
                armed = (self.fcu_t is not None and now - self.fcu_t < 5.0
                         and self.fcu_armed)
                if armed:
                    y = self._line(frame, k, y, "ARMED", HUD_GREEN,
                                   scale=1.0, fill=HUD_GREEN)
                elif ekf == "1":
                    y = self._line(frame, k, y, "EKF READY - TAKEOFF OK",
                                   HUD_GREEN, scale=1.0, fill=HUD_GREEN)
                else:
                    y = self._line(frame, k, y, "EKF WARMUP", HUD_YELLOW,
                                   scale=1.0, fill=HUD_YELLOW)
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
            # 1b) диагностика детектора зрелости (мелко, под баннером):
            # res — residual «поза/скорость» (тихо < 0.15 м/с), rat —
            # вертикальный ratio VINS/rel_alt (зрел в [0.8,1.25]); «--» =
            # данных ещё нет (-1 от лётной ноды / старый bag без полей)
            res, rat = self.status.get("res"), self.status.get("rat")
            if res is not None:
                def _f(v):
                    try:
                        x = float(v)
                    except (TypeError, ValueError):
                        return "--"
                    return "--" if x < 0 else f"{x:.2f}"
                y = self._line(frame, k, y, f"res {_f(res)}  rat {_f(rat)}",
                               HUD_WHITE, scale=0.6)
            # 1c) высота ТРЕМЯ источниками: baro — высота миссии (alt=,
            # rel_alt: баро при BS_ALT_SRC=baro), ekf — z local_position
            # глазами EKF3 (zekf=), perc — высота ПЕРЦЕПЦИИ (palt=), по
            # которой судит гейт земли IPM. Третья не от жадности: разбор
            # 183305 упёрся ровно в то, что HUD показывал первые две, а
            # канал закрывала ТРЕТЬЯ (perc_alt_src=local, смещение −0.27 м).
            # ⚠️ Порог жёлтого ОТНОСИТЕЛЬНЫЙ: max(0.2, 0.2·baro). Прежние
            # фиксированные 0.5 м на низком полёте бесполезны — в 183305
            # расхождение 0.3 м было ровно 100% высоты полёта и не
            # подсветилось. Судим по perc (её и слушает гейт), при её
            # отсутствии — по ekf. «--» = источника нет / протух.
            alt = self.status.get("alt")
            if alt is not None:
                def _v(s):
                    try:
                        return float(s)
                    except (TypeError, ValueError):
                        return None

                def _t(x):
                    return f"{x:.1f}" if x is not None else "--"
                a = _v(alt)
                z, pa = _v(self.status.get("zekf")), _v(self.status.get("palt"))
                cmp_v = pa if pa is not None else z
                thr = max(0.2, 0.2 * abs(a)) if a is not None else 0.2
                col = (HUD_YELLOW if a is not None and cmp_v is not None
                       and abs(a - cmp_v) > thr else HUD_WHITE)
                txt = f"ALT baro {_t(a)}m  ekf {_t(z)}m"
                if "palt" in self.status:
                    txt += f"  perc {_t(pa)}m"
                y = self._line(frame, k, y, txt, col)
            # 1d) КАНАЛ ВИДА СВЕРХУ — на чём стоит демпфер у земли. Мгновенный
            # код (ipmf=) + доля годных за окно 3 с: гейт высоты у земли
            # дребезжит, и по одному кадру «10% годных» неотличимы от нуля.
            # Зелёный — годен; жёлтый — фильтр ipm_vel_tau мостит брак кадра
            # (ipm=1 при ipmf≠0); красный — оси слепы (ipm=0).
            if "ipm" in self.status:
                try:
                    ok = int(self.status["ipm"])
                    fail = int(self.status.get("ipmf", 0))
                except ValueError:
                    ok, fail = 0, 0
                win = [v for t, v in self.ipm_hist if now - t < 3.0]
                share = 100.0 * sum(win) / len(win) if win else 0.0
                col = (HUD_GREEN if ok and not fail else
                       HUD_YELLOW if ok else HUD_RED)
                name = IPM_FAIL.get(fail, f"?{fail}")
                y = self._line(frame, k, y,
                               f"IPM {name} {share:3.0f}%", col)
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
