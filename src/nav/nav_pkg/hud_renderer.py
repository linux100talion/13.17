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
# Протухший источник гаснет сам (баннеры и лесенка — 3 с, CMD — 2 с);
# чего в источниках нет — того нет и на экране (честная деградация: на Orin
# без лётной ноды нет баннера, в bag без /feature нет строки FEAT).
#
# Геометрия задана для эталонного кадра 1280×720 и масштабируется k=w/1280:
# на любом разрешении HUD занимает ту же долю кадра, что в FPV-даунлинке.
# ============================================================================
import collections
import math

import cv2

FONT = cv2.FONT_HERSHEY_SIMPLEX
# BGR-палитра HUD; баннер гейта заливается цветом состояния, текст на нём чёрный
HUD_GREEN = (60, 200, 60)
HUD_YELLOW = (0, 210, 240)
HUD_RED = (50, 50, 230)
HUD_WHITE = (235, 235, 235)
# стрелка ветра из ТРИМА (wns=ipm/vins): гаснет ниже LO PWM, полная с HI (hud.md 3.13)
WIND_ARROW_LO = 8.0
WIND_ARROW_HI = 15.0


def wind_arrow_fade(pwm, src) -> float:
    """Яркость стрелки ветра 0..1 по |трим| PWM: 0 ниже WIND_ARROW_LO (стрелки
    нет, подпись calm), 1 с WIND_ARROW_HI, между — линейно (без дребезга на
    пороге). Только источники-тримы (IPM/VINS): направление слабого трима — шум
    (полёты 185004/191610 «стрелка крутится», базовый ветер 1 м/с ≈ 6 PWM; 2 м/с
    ≈ 15 PWM по стенду). Ветер EKF (LOITER) — всегда 1."""
    if str(src).upper() not in ("IPM", "VINS"):
        return 1.0
    return max(0.0, min(1.0, (pwm - WIND_ARROW_LO) / (WIND_ARROW_HI - WIND_ARROW_LO)))
HUD_SCENE = (0, 255, 255)
# Причина брака кадра IPM (ipmf= в /mission/status) — та же таблица, что
# FlowEstimator.ipm_fail. ⚠️ ТОЛЬКО ASCII: Hershey-шрифт OpenCV кириллицу не
# рисует (вышли бы «?»), поэтому подписи английские, как и остальной HUD.
IPM_FAIL = {0: "OK", 1: "ALT GATE", 2: "NO WINDOW", 3: "WARP OOB",
            4: "FEW PTS", 5: "FEW LK", 6: "OFF", 7: "NO REF"}
# Ярусы лесенки SF-мастера (tier=/lvl= в /mission/status) — копия
# control_pkg.application.hud.TIER_NAMES (nav_pkg control_pkg не импортирует).
TIER_NAMES = {0: "DAMPER", 1: "VINSHOLD", 2: "LOITER"}
# Мягкая посадка по кнопке SA (land= в /mission/status, SoftLand.land_state) —
# копия control_pkg.application.hud.LAND_NAMES. Баннер «LANDING <x>»: зелёный —
# позицию держит FCU (LAND на EKF-от-VINS) или касание; жёлтый — снижение в
# ALT_HOLD под нашим стеком (демпфер/VinsHold, стик = наклон).
LAND_NAMES = {"pos": "FCU POS", "damper": "DAMPER", "vinshold": "VINSHOLD",
              "touch": "TOUCHDOWN"}
# FCU не подтверждает LOITER дольше этого — «refuses» (зеркало _latch_warned
# в Freefly._mode_target: предупреждение в лог через 5 с ре-ассерта).
LATCH_REFUSE_SEC = 5.0
# Общий коэффициент размера шрифта HUD (просьба 2026-08-30: «на ~30 % мельче»):
# масштабирует глифы, толщину, подложку и межстрочный шаг всех строк разом —
# пропорции строк между собой (баннеры 1.0 / строки 0.8 / ярусы 0.7)
# не меняются. 1.0 = прежний размер.
FONT_K = 0.7


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
    @staticmethod
    def _metrics(k, text, scale):
        """(tw, th, base, thickness) текста при масштабе кадра k и FONT_K."""
        f = k * FONT_K
        thick = max(1, round(2 * f))
        (tw, th), base = cv2.getTextSize(text, FONT, scale * f, thick)
        return tw, th, base, thick

    def _box(self, frame, k, x, y, text, color, scale, fill):
        """Текст с подложкой (читаемость поверх любой сцены) в точке базовой
        линии (x, y); вернёт (tw, th, base)."""
        f = k * FONT_K
        tw, th, base, thick = self._metrics(k, text, scale)
        pad = round(6 * f)
        cv2.rectangle(frame, (x - pad, y - th - pad),
                      (x + tw + pad, y + base + round(4 * f)),
                      (0, 0, 0) if fill is None else fill, -1)
        cv2.putText(frame, text, (x, y), FONT, scale * f,
                    color if fill is None else (0, 0, 0), thick)
        return tw, th, base

    def _line(self, frame, k, y, text, color, scale=0.8, fill=None):
        """Строка левой стопки HUD (отступ 10 px @1280); вернёт next y."""
        _tw, th, base = self._box(frame, k, round(10 * k), y, text, color,
                                  scale, fill)
        return y + th + base + round(18 * k * FONT_K)

    def _line_right(self, frame, k, y, text, color, scale=0.8, fill=None):
        """Строка ПРАВОЙ стопки (якорь по правому краю, отступ 10 px @1280);
        вернёт next y."""
        tw, th, base, _thick = self._metrics(k, text, scale)
        x = frame.shape[1] - tw - round(10 * k)
        self._box(frame, k, x, y, text, color, scale, fill)
        return y + th + base + round(18 * k * FONT_K)

    def _line_bottom_center(self, frame, k, text, color, scale=0.8, fill=None):
        """Строка, заякоренная по ЦЕНТРУ НИЗА кадра (отступ 14 px @1280):
        высота — как приборная лента у пилота, не в общей стопке слева."""
        tw, _th, base, _thick = self._metrics(k, text, scale)
        x = (frame.shape[1] - tw) // 2
        y = frame.shape[0] - base - round(4 * k * FONT_K) - round(14 * k)
        self._box(frame, k, x, y, text, color, scale, fill)

    # --- лесенка: гейты ярусов по полям /mission/status ---
    def _num(self, key, default=None):
        try:
            return float(self.status[key])
        except (KeyError, ValueError):
            return default

    def _tier_gate(self, i):
        """(st, text) яруса i по полям статуса: st — READY/WAIT/DEAD, text —
        причина с ПРОГРЕССОМ (счётчик к порогу, секунды к порогу, высота к
        loiter_alt) — пилоту и разбору важно «сколько ещё», а не голое слово.
        Ярус 0 — канал вида сверху (ipm/ipmf); 1 — VinsHold (t1/w1, зеркало
        VinsHandover.vins_ready); 2 — штатный LOITER (st/why + латч lat=)."""
        odom = self.status.get("odom", "?")
        age = self._num("age")
        age_s = f"{age:.1f}s" if age is not None else "?"
        if i == 0:
            ok, fail = int(self._num("ipm", 0)), int(self._num("ipmf", 0))
            if ok:
                return "READY", "OK"
            return "WAIT", f"BLIND {IPM_FAIL.get(fail, f'?{fail}')}"
        if i == 1:
            st, why = self.status.get("t1", "?"), self.status.get("w1", "-")
            vmin = self.status.get("vmin", "?")
            if st == "READY":
                return st, f"OK odom {odom}"
            if why == "odom":
                return st, f"WAIT odom {odom}/{vmin}"
        else:
            st, why = self.status.get("st", "?"), self.status.get("why", "-")
            lat = self._num("lat", -1.0)
            if lat is not None and lat >= 0.0:
                if lat > LATCH_REFUSE_SEC:
                    return "DEAD", f"FCU REFUSES {lat:.0f}s"
                return "WAIT", f"LATCH {lat:.0f}s"
            if st == "READY":
                return st, "OK"
            if why == "extnav":
                ripe, rsec = self._num("ripe", -1.0), self._num("rsec", 0.0)
                rcnt = self.status.get("rcnt", "?")
                return st, (f"WAIT extnav {odom}/{rcnt} "
                            f"{max(ripe, 0.0):.0f}/{rsec:.0f}s")
            if why == "ground":
                alt, lalt = self._num("alt"), self._num("lalt")
                a = f"{alt:.1f}" if alt is not None else "?"
                la = f"{lalt:g}" if lalt is not None else "?"
                return st, f"WAIT ground {a}<{la}m"
        if st == "DEAD":
            return st, ("NO VINS" if why == "no_odom" else f"NO VINS stale {age_s}")
        if why == "stale":
            return st, f"WAIT stale {age_s}"
        return st, f"{st} {why}"

    def _next_tier(self):
        """Ярус, на который лесенка ПЫТАЕТСЯ подняться (tier+1, пока ярус ниже
        потолка SC и пилот не в MANUAL); None — лесенка на месте."""
        if self.status.get("sw") == "1":
            return None
        tier, lvl = int(self._num("tier", 0)), int(self._num("lvl", 0))
        return tier + 1 if tier < lvl and tier + 1 in TIER_NAMES else None

    def _tier_rows(self, now=None):
        """Строки блока лесенки: [(text, color, fill)] по ярусам 0..2. Маркер
        «>» — ярус, которого лесенка ждёт (следующий над активным при потолке
        выше): его причина и есть ответ «почему не выше» — раньше она
        дублировалась в баннере, теперь живёт только здесь.
        В ХВОСТ строки АКТИВНОГО яруса — режим FCU из /mavros/state (просьба
        2026-08-31: отдельной строки режима больше нет, а имя режима нигде
        больше не показано). Только имя, без ARM/DISARM (armed — баннер 3.1);
        протух (>5 с) или нет mavros_msgs — хвоста нет, строка как была."""
        tier, lvl = int(self._num("tier", 0)), int(self._num("lvl", 0))
        nxt = self._next_tier()
        mode = (self.fcu_mode if (now is not None and self.fcu_t is not None
                                  and now - self.fcu_t < 5.0) else "")
        rows = []
        for i in (0, 1, 2):
            st, text = self._tier_gate(i)
            col = (HUD_GREEN if st == "READY" else
                   HUD_YELLOW if st == "WAIT" else HUD_RED)
            fill = None
            if i == tier:
                fill = col                     # активный ярус — заливка
                if mode:
                    text = f"{text}  {mode}"   # режим FCU — в хвост активного
            elif i > lvl:
                col = HUD_WHITE                # выше потолка SC — не выбран
            mark = ">" if i == nxt else " "
            rows.append((f"{mark} {i} {TIER_NAMES[i]}  {text}", col, fill))
        return rows

    def _tier_banner(self):
        """(text, color) большого баннера: ТОЛЬКО «TIER n NAME» и цвет — всё
        остальное (причина следующего яруса, латч, MANUAL) живёт в блоке
        ярусов под баннером, здесь бы дублировалось. Цвет:
          зелёный — ярус = потолок SC, гейт активного яруса открыт;
          жёлтый  — лесенка ниже потолка (ждёт следующий ярус — см. «>» в
                    блоке) или активный ярус держится на гистерезисе;
          красный — следующий ярус мёртв (VINS нет) / FCU не латчит LOITER;
          белый   — MANUAL (SF не-вверх): лесенка борт не ведёт, ярус — что
                    держало бы без перехвата.
        Без лесенки (старый bag, легаси-селектор, миссия) — гейт LOITER под
        своим именем: LOITER READY / LOITER WAIT (why) / NO VINS (why) —
        блока ярусов там нет, причина остаётся в баннере.
        Мягкая посадка (land=, шаг SoftLand после кнопки SA) — поверх всего:
        LANDING FCU POS / DAMPER / VINSHOLD / TOUCHDOWN."""
        land = self.status.get("land")
        if land:
            col = HUD_GREEN if land in ("pos", "touch") else HUD_YELLOW
            return f"LANDING {LAND_NAMES.get(land, land.upper())}", col
        if "tier" not in self.status:
            st = self.status.get("st", "")
            why = self.status.get("why", "-")
            if st == "READY":
                return "LOITER READY", HUD_GREEN
            if st == "WAIT":
                return f"LOITER WAIT ({why})", HUD_YELLOW
            return f"NO VINS ({why})", HUD_RED
        tier = int(self._num("tier", 0))
        text = f"TIER {tier} {TIER_NAMES.get(tier, '?')}"
        if self.status.get("sw") == "1":
            return text, HUD_WHITE
        nxt = self._next_tier()
        if nxt is None:
            st, _ = self._tier_gate(tier)
            return text, HUD_GREEN if st == "READY" else HUD_YELLOW
        st, _ = self._tier_gate(nxt)
        return text, HUD_RED if st == "DEAD" else HUD_YELLOW

    def _draw_ladder_block(self, frame, k, y, now):
        """ЛЕСЕНКА: по строке на ярус (0 DAMPER / 1 VINSHOLD / 2 LOITER) с
        гейтом и прогрессом каждого. Активный ярус — заливка; закрытый —
        жёлтым с причиной; мёртвый — красным; выше потолка SC — белым (не
        выбран); «>» — ярус, которого лесенка ждёт; режим FCU — в хвосте строки
        активного яруса. Шапки «<MODE> ARM · SC n · TIER n» больше нет (просьба
        2026-08-31): потолок SC и активный ярус читаются по самому блоку
        (заливка = активный ярус, «>» = куда лезем), armed — по баннеру статуса
        борта (3.1), режим уехал в хвост активной строки. Блок живёт ТОЛЬКО при свежем (<3 с)
        статусе с tier=; без лесенки строк нет вовсе. Вернёт next y."""
        if not (self.status_t is not None and now - self.status_t < 3.0
                and "tier" in self.status):
            return y
        for text, col, fill in self._tier_rows(now):
            y = self._line(frame, k, y, text, col, scale=0.7, fill=fill)
        return y

    def _draw_wind(self, frame, k):
        """СТРЕЛКА ВЕТРА из трима активного стабилизатора (wnp/wnr/wns статуса,
        правый нижний угол). Компас ОТНОСИТЕЛЬНО НОСА (метка сверху = нос,
        видео и так «глазами носа»): стрелка показывает, КУДА ДУЕТ (куда несло
        бы борт), подпись — сила и датчик (IPM = трим демпфера / VINS = трим
        DpVins; выбирает лётная нода по активному ярусу).
        Семантика PWM (знаковый якорь — полёт joystick/1 2026-09-04, ветер-10 к
        98°, курсы −92° и 0°): наклон трима смотрит ПРОТИВ ветра; pitch+ =
        лечь назад, roll+ = вправо → «дует к» в теле = (wnp, −wnr) по
        (вперёд, вправо). Сила: якорь 100 PWM ↔ 10 м/с и F ∝ v² → v =
        10·√(PWM/100). Оценка честна на удержании (трим выучен) и врёт первые
        секунды обучения — как всякий трим."""
        if "wns" not in self.status:
            return
        p, r = self._num("wnp"), self._num("wnr")
        if p is None or r is None:
            return
        pwm = math.hypot(p, r)
        spd = 10.0 * math.sqrt(pwm / 100.0)
        a = math.atan2(-r, p)              # куда дует, по часовой от носа
        src = self.status.get("wns", "").upper()
        # ЗАТУХАНИЕ СТРЕЛКИ ТРИМА (2026-09-06): ниже WIND_ARROW_LO PWM стрелки нет,
        # к WIND_ARROW_HI — полная яркость, между — линейно (без дребезга на пороге).
        # Направление слабого трима — шум (полёты 185004/191610: «стрелка крутится»
        # при базовом ветре 1 м/с ≈ 6 PWM; ветер 2 м/с ≈ 15 PWM по стенду). Только
        # для источников-тримов (IPM/VINS); ветер EKF (LOITER) рисуем как есть.
        fade = wind_arrow_fade(pwm, src)
        col = tuple(int(round(c * (0.25 + 0.75 * fade))) for c in HUD_WHITE)
        rad = round(34 * k)
        cx = frame.shape[1] - round(10 * k) - rad
        cy = frame.shape[0] - round(14 * k) - rad
        thick = max(1, round(2 * k * FONT_K))
        cv2.circle(frame, (cx, cy), rad, (0, 0, 0), -1)
        cv2.circle(frame, (cx, cy), rad, HUD_WHITE, thick)
        cv2.line(frame, (cx, cy - rad), (cx, cy - rad + round(8 * k)),
                 HUD_WHITE, thick)         # метка носа
        dx, dy = math.sin(a), -math.cos(a)
        e = 0.72 * rad
        if fade > 0.0:
            cv2.arrowedLine(frame, (round(cx - e * dx), round(cy - e * dy)),
                            (round(cx + e * dx), round(cy + e * dy)),
                            col, thick + 1, tipLength=0.35)
        text = f"WIND {spd:.1f} {src}" if fade > 0.0 else f"WIND calm {src}"
        tw, _th, base, _t = self._metrics(k, text, 0.7)
        self._box(frame, k, frame.shape[1] - tw - round(10 * k),
                  cy - rad - base - round(8 * k), text, col, 0.7, None)
        # |скорость| того же датчика — СЛЕВА от компаса (как быстро реально
        # несёт по тому же сенсору, чей ветер показан). spd= в статусе, м/с;
        # поля нет (нет активного датчика) — числа не рисуем.
        vm = self._num("spd")
        if vm is not None:
            vt = f"V {vm:.1f}"
            vtw, _vth, vbase, _vt = self._metrics(k, vt, 0.7)
            self._box(frame, k, cx - rad - round(8 * k) - vtw,
                      cy + vbase // 2, vt, HUD_WHITE, 0.7, None)

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
        # Раскладка (просьбы 2026-08-30): ЛЕВАЯ стопка — режимы (баннеры
        # статуса борта и яруса, блок ярусов, DRIFT, scene); ПРАВАЯ
        # стопка сверху — датчики/каналы (IPM, ODO, FEAT, CMD); НИЗ по
        # центру — высота ALT. Отсутствующий источник места не оставляет.
        st_ok = self.status_t is not None and now - self.status_t < 3.0
        # ---------------- ЛЕВАЯ стопка: режимы ----------------
        y = round(34 * k * FONT_K)
        if st_ok:
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
            # 1b) баннер ЯРУСА (лесенка SF-мастера): что держит борт сейчас.
            # Раньше тут был «VINS READY/WAIT» — по st/why, т.е. по гейту
            # ОДНОГО яруса (LOITER), а назывался именем VINS: разбор ab_noise
            # 2026-08-30 — VINS шёл 10 Гц, баннер писал «VINS WAIT». Живость
            # VINS — строка ODO справа; здесь — режимы. Bag/план без лесенки
            # (нет tier=) — голый гейт LOITER под честным именем.
            text, col = self._tier_banner()
            y = self._line(frame, k, y, text, col, scale=1.0, fill=col)
        # 2) лесенка — СРАЗУ под баннером яруса: баннер говорит «какой ярус»,
        # блок под ним — «почему не выше»; между ними ничего не вклинивается.
        y = self._draw_ladder_block(frame, k, y, now)
        # 3) поправка NN1: засечки редкие, старше 10 с — показываем возраст
        if self.drift is not None:
            d, t = self.drift
            age = now - t
            txt = f"DRIFT {d:.2f}m" + (f" ({age:.0f}s)" if age > 10.0 else "")
            y = self._line(frame, k, y, txt, HUD_WHITE)
        # 4) семантика сцены NN2 (бывший одинокий баннер)
        if self.scene:
            self._line(frame, k, y, f"scene: {self.scene}", HUD_SCENE)
        # ---------------- НИЗ по центру: высота ----------------
        if st_ok:
            # 5) высота ТРЕМЯ источниками — якорь по центру нижнего края (не
            # в стопках: читается как приборная лента, отдельно от режимов).
            # baro — высота миссии (alt=, rel_alt: баро при BS_ALT_SRC=baro),
            # ekf — z local_position глазами EKF3 (zekf=), perc — высота
            # ПЕРЦЕПЦИИ (palt=), по которой судит гейт земли IPM. Третья не от
            # жадности: разбор 183305 упёрся ровно в то, что HUD показывал
            # первые две, а канал закрывала ТРЕТЬЯ (perc_alt_src=local,
            # смещение −0.27 м).
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
                self._line_bottom_center(frame, k, txt, col)
        # ---------------- ПРАВАЯ стопка: датчики и каналы ----------------
        yr = round(34 * k * FONT_K)
        if st_ok:
            # 6) КАНАЛ ВИДА СВЕРХУ — на чём стоит демпфер у земли. Мгновенный
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
                yr = self._line_right(frame, k, yr,
                                      f"IPM {name} {share:3.0f}%", col)
            # Строки «res/rat» (диагностика детектора зрелости VINS) здесь
            # больше нет (просьба 2026-08-31): пилоту в полёте она не нужна,
            # а разбору — есть res=/rat= в /mission/status (bag). Прогресс
            # очереди зрелости виден в блоке ярусов: «WAIT extnav n/N s/S».
        # 7) /odometry: Гц (окно 3 с) + возраст; красный ODO -- = VINS без
        # init. Единственная строка про ЖИВОСТЬ VINS — собственный замер
        # стримера, независим от лётной ноды.
        if self.odom_times:
            age = now - self.odom_times[-1]
            win = [t for t in self.odom_times if now - t < 3.0]
            hz = ((len(win) - 1) / (win[-1] - win[0])
                  if len(win) >= 2 and win[-1] > win[0] else 0.0)
            col = (HUD_GREEN if age < 0.5 else
                   HUD_YELLOW if age < 1.5 else HUD_RED)
            yr = self._line_right(frame, k, yr,
                                  f"ODO {hz:4.1f}Hz {age:4.1f}s", col)
        else:
            yr = self._line_right(frame, k, yr, "ODO --", HUD_RED)
        # 8) фичи трекера — СРАЗУ под ODO (просьба 2026-08-31: пара «жив ли
        # VINS / за что он цепляется» читается вместе); замолк при живой
        # камере — это ЧП, красним
        if self.feat_t is not None:
            if now - self.feat_t < 3.0:
                yr = self._line_right(frame, k, yr, f"FEAT {self.feat_n}",
                                      HUD_WHITE)
            else:
                yr = self._line_right(frame, k, yr, "FEAT --", HUD_RED)
        # 9) PWM-смещения крена/тангажа от стека (/flow_dbg, /flow_dbg2)
        if self.cmd_t is not None and now - self.cmd_t < 2.0:
            self._line_right(frame, k, yr,
                             f"CMD R{self.cmd_roll:+04.0f} "
                             f"P{self.cmd_pitch:+04.0f}", HUD_WHITE)
        # 10) стрелка ветра из трима активного стабилизатора (правый нижний
        # угол); поля wnp/wnr/wns есть только при активном источнике (ярусы
        # 0/1), протухший статус гасит виджет вместе с остальными
        if st_ok:
            self._draw_wind(frame, k)
