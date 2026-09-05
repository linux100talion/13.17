#!/usr/bin/env python3
"""VinsHandover — рантайм-передача стабилизации Flow→Vins по событию «VINS ready».

Тот самый hot-swap, ради которого существует ControlStack.switch_*. Пока VINS не
сошёлся — стабилизирует наш пре-VINS демпфер (Flow+Yaw); как только поток одометрии
устойчив → заменяет стабилизаторы стека на VinsHold (захватив vins-опору в этот
момент). Пилот (RcTransmitter) не трогается — меняется только источник опоры.

⚠️ Свап применяется К КАЖДОМУ НОВОМУ СТЕКУ: каждый Control-шаг миссии строит
СВЕЖИЙ стек с Dp-стабилизаторами, и однократный свап (до 2026-08-19) жил только
до конца текущего шага — все последующие шаги молча летали на демпфере (полёт
VINSHANDOVER: hover_1 держал 0.9 м, hover_4/7 болтало 4-7 м демпферным дрейфом).
Опора перезахватывается на каждом свапе — это и есть семантика шага («держи от
своей точки»), идентична поведению Dp-холдеров.

«VINS ready» = устойчивый поток /vins_estimator/odometry: VINS публикует одометрию
ТОЛЬКО после инициализации (solver NON_LINEAR), поэтому N сообщений + свежесть =
сходимость (как vins_converged() монолита).

Живёт в application (не в домене): это policy оркестрации стратегий, а не закон.
Тестируется оффлайн — синтетический рост vins_odom_count → switch срабатывает 1 раз.
"""
from ..domain.rc import RC_CENTER


class VinsHandover:
    _STICK_DZ = 40                # PWM: |стик − центр| ниже = стик в центре (висение)

    def __init__(self, vins_hold, min_count: int = 40, fresh_sec: float = 2.0,
                 v_max: float = 0.0, ipm_tol: float = 0.0, sane_n: int = 3,
                 hover_v: float = 0.0, hover_sec: float = 2.0,
                 trim_seed: bool = True, scale_ratio: float = 0.0,
                 scale_ipm_min: float = 2.0, scale_sec: float = 3.0,
                 scale_alt_max: float = 4.0, scale_hold: float = 30.0):
        self._vins = vins_hold
        # посев трима от демпфера на входе в ярус 1 (DpHold.trim_pwm →
        # DpVins.seed_trim): ветер, который демпфер уже держит, не учить заново
        self.trim_seed = trim_seed
        self.min_count = min_count
        self.fresh_sec = fresh_sec
        # ГЕЙТ ЗДОРОВЬЯ (санити): защита от разноса VINS (см. config.vins_v_max).
        # 0 = выкл. sane_n — сколько кадров подряд «болен» до вердикта.
        self.v_max = v_max
        self.ipm_tol = ipm_tol
        self.sane_n = sane_n
        # ФИЗИКА ВИСЕНИЯ (ловит МЕДЛЕННЫЙ разнос, потолок — только грубый): при
        # ЦЕНТРАЛЬНЫХ стиках дольше hover_sec истинная скорость ограничена ветром
        # (~1 м/с даже в 10 м/с — контур держит); |vins_v| выше hover_v = разнос.
        # VINS-независимо (стик+|vins_v|), надёжно (в отличие от IPM). 0 = выкл.
        self.hover_v = hover_v
        self.hover_sec = hover_sec
        self._center_since = None     # sim-время начала непрерывного висения (стик центр)
        # ЧЕК ЗАНИЖЕНИЯ |vins_v| (коллапс масштаба реборн-VINS, lv2_joy_20260905_
        # 114248: VINS «0.4–0.9» при истинных 3–5.5, IPM годен 100 % и видел 5.0):
        # потолок и физика висения ловят только ЗАВЫШЕНИЕ. Опорник — IPM-канал,
        # но ТОЛЬКО там, где он надёжен: висение (стики центр), низко
        # (≤ scale_alt_max), ipm_ok непрерывно ≥ scale_sec; на высоте/быстрой
        # прямой IPM сам мусорил (ab_tier2gate 7→155 м/с) — там чек молчит.
        # |ipm_v| > scale_ipm_min и |vins_v| < ratio·|ipm_v| дольше scale_sec →
        # не sane, латч на scale_hold (масштаб сам не починится; демпфер держит
        # по IPM). Латч снимает перерождение VINS (новая рама = новый масштаб).
        self.scale_ratio = scale_ratio
        self.scale_ipm_min = scale_ipm_min
        self.scale_sec = scale_sec
        self.scale_alt_max = scale_alt_max
        self.scale_hold = scale_hold
        self._ipm_ok_since = None     # sim-старт непрерывной годности IPM
        self._scale_bad_since = None  # sim-старт непрерывного «VINS занижает»
        self._scale_until = -1e9      # латч чека занижения до (sim-с)
        self.scale_trips = 0          # срабатываний чека занижения (лог/статус)
        self._trim_dirty = False      # был /restart VINS: мировой трим стаба недействителен
        self._bad = 0                # кадров подряд «болен» (IPM|hover)
        self._last_sane_t = None      # счётчик двигаем раз на новый sim-тик
        self._was_sane = True         # прошлый вердикт (для фронта sane→insane)
        self._restart_req = False     # одноразовый запрос /restart (нода опросит)
        self._done = False

    def vins_sane(self, s) -> bool:
        """VINS вменяем: скорость физически возможна И (если IPM жив) согласна с
        независимым оптическим каналом. Разнос (|v|→20 на неподвижном борте)
        проваливает оба. Модуль скорости инвариантен к системе координат, так
        что vins (мир) и ipm (тело) сравнимы напрямую. На ФРОНТЕ sane→insane
        заказывает /restart VINS (нода опрашивает pop_restart_request).
        Третий чек — ЗАНИЖЕНИЕ против IPM (коллапс масштаба), см. __init__."""
        import math
        vh = math.hypot(s.vins_vx, s.vins_vy)
        # Счётчик двигаем раз на новый sim-тик — метод зовут оба пути лесенки.
        new_tick = self._last_sane_t is None or s.now_sim > self._last_sane_t
        bad = False
        # IPM-кросс-чек (по умолчанию выкл — IPM ненадёжен, см. config.vins_ipm_tol)
        if self.ipm_tol > 0.0 and s.ipm_ok:
            iv = math.hypot(s.ipm_vfwd, s.ipm_vlat)
            bad = bad or abs(vh - iv) > self.ipm_tol
        # ФИЗИКА ВИСЕНИЯ: стик в центре дольше hover_sec + |vins_v| > hover_v
        centered = (abs(s.pilot_roll - RC_CENTER) < self._STICK_DZ
                    and abs(s.pilot_pitch - RC_CENTER) < self._STICK_DZ)
        if new_tick:
            self._center_since = (self._center_since or s.now_sim) if centered else None
        centered_long = (self._center_since is not None
                         and s.now_sim - self._center_since > self.hover_sec)
        hovering = self.hover_v > 0.0 and centered_long
        if hovering and vh > self.hover_v:
            bad = True                        # висим, а VINS «летит» = разнос
        if new_tick:
            self._bad = self._bad + 1 if bad else 0
        # ЧЕК ЗАНИЖЕНИЯ (см. __init__): VINS видит много меньше, чем стойко годный
        # IPM на висении низко → масштаб VINS схлопнулся, контур слеп.
        if new_tick:
            self._ipm_ok_since = ((self._ipm_ok_since or s.now_sim)
                                  if s.ipm_ok else None)
        if self.scale_ratio > 0.0:
            alt = s.perc_alt if s.perc_alt is not None else s.rel_alt
            low = alt is not None and alt <= self.scale_alt_max
            ipm_stable = (self._ipm_ok_since is not None
                          and s.now_sim - self._ipm_ok_since >= self.scale_sec)
            iv = math.hypot(s.ipm_vfwd, s.ipm_vlat)
            under = (centered_long and low and ipm_stable
                     and iv > self.scale_ipm_min and vh < self.scale_ratio * iv)
            if new_tick:
                self._scale_bad_since = ((self._scale_bad_since or s.now_sim)
                                         if under else None)
                if (self._scale_bad_since is not None
                        and s.now_sim - self._scale_bad_since >= self.scale_sec):
                    if s.now_sim >= self._scale_until:
                        self.scale_trips += 1           # новый латч (не продление)
                    self._scale_until = s.now_sim + self.scale_hold
        scale_bad = s.now_sim < self._scale_until
        cap_bad = self.v_max > 0.0 and vh > self.v_max   # физ. потолок (грубо, сразу)
        sane = (not cap_bad) and (self._bad < self.sane_n) and not scale_bad
        if new_tick:
            if self._was_sane and not sane:   # фронт: заказать сброс VINS
                self._restart_req = True
            self._was_sane = sane
            self._last_sane_t = s.now_sim
        return sane

    def pop_restart_request(self) -> bool:
        """One-shot: True если был фронт sane→insane с прошлого опроса (нода
        шлёт /restart, чтобы VINS переинициализировался после разноса)."""
        r = self._restart_req
        self._restart_req = False
        return r

    def note_vins_restart(self) -> None:
        """Нода ФАКТИЧЕСКИ послала /restart VINS: мировая рама одометрии
        перерождается (мир монокуляра рождается с курсом первого кадра) —
        ветровой трим стабилизатора, хранимый мировым вектором, в новой раме
        недействителен. Помечаем; сброс — на ближайшем входе в ярус
        (vins_stabs). Запрос-фронт гейта (pop_restart_request) сам по себе
        трим НЕ сбрасывает: рестарт может не состояться (кулдаун, выкл).
        Латч чека занижения снимается: новая рама = новый масштаб."""
        self._trim_dirty = True
        self._scale_until = -1e9
        self._scale_bad_since = None

    def vins_ready(self, s) -> bool:
        return (s.vins_odom_count >= self.min_count and
                (s.now_sim - s.vins_last_sim) < self.fresh_sec and
                self.vins_sane(s))

    def vins_stabs(self, base, s):
        """Состав яруса VinsHold: yaw-стабы из base + VinsHold ПОСЛЕДНИМ (порядок —
        см. maybe_switch: поздний перезаписывает свои оси у раннего; keep+[vins]
        верен и для композитов, и для раздельных стабов). VinsHold.enter здесь же:
        опора = текущая точка. Используется и одноразовым свапом (maybe_switch),
        и лесенкой SF-мастера (Freefly._ladder_apply — там переходы двусторонние)."""
        keep = [st for st in base if "yaw" in getattr(st, "axes", frozenset())]
        self.seed_vins(base, s)
        self._vins.enter(s)
        return keep + [self._vins]

    def seed_vins(self, base, s) -> None:
        """ПОСЕВ ветрового трима DpVins от демпферной базы (И-член демпфера =
        тот же ветер, снятый секунду назад — DpVins не учит заново, унос
        ~2 м → ~0). Вынесен из vins_stabs, потому что нужен НЕ только на входе
        в ярус 1: лесенка умеет прыгать 0→2 (LOITER латчится в тот же миг, что
        зреет VINS) МИНУЯ vins_stabs — тогда без явного посева здесь трим
        DpVins остаётся девственным, и стрелка ветра HUD в LOITER пуста
        (симптом: «пропадает при переключении в loiter, возвращается
        перещёлкиванием тумблера» — перещёлк проводит через ярус 1). Зовётся
        и из _ladder_apply на входе в ярус 2. Идемпотентен: seed_trim сам
        отказывает на НЕдевственном триме (начатое обучение / выученный ветер
        свежее демпферного), так что проход 0→1→2 не сеет дважды.
        /restart (_trim_dirty) сбрасывает трим здесь же — тогда сеем свежую
        раму. Валюта — PWM каналов (trim_pwm), знаки — забота seed_trim."""
        if self._trim_dirty:
            # рама VINS переродилась (/restart) — мировой трим недействителен;
            # у стабов без трима (VinsHold) метода нет — им сбрасывать нечего
            reset = getattr(self._vins, "reset_trim", None)
            if reset is not None:
                reset()
            self._trim_dirty = False
        if not self.trim_seed:
            return
        seed = getattr(self._vins, "seed_trim", None)
        if seed is None:
            return
        for st_ in base:
            tp = getattr(st_, "trim_pwm", None)
            v = tp() if tp is not None else None
            if v is not None:
                seed(v[0], v[1], s)
                break

    def maybe_switch(self, stack, s) -> bool:
        """Если VINS сошёлся, а ЭТОТ стек ещё на демпфере — заменить roll/pitch-
        стабилизаторы на VinsHold, СОХРАНИВ стабы других осей (yaw-холд остаётся
        жив — иначе после свапа рыскание замирало в центр, а живой пилот терял
        yaw-стик). Возвращает True на тике переключения (лог).

        ⚠️ VinsHold идёт ПОСЛЕДНИМ в списке — в per-axis композиции ControlStack
        поздний стабилизатор перезаписывает свои оси у раннего. Порядок
        [vins]+keep ломался на КОМПОЗИТЕ (DpHoldM: один стаб с осями
        roll+pitch+yaw): «есть yaw» сохранял его целиком, композит стоял после
        VinsHold и перезаписывал roll/pitch — VinsHold обезврежен, борт летел
        на голом демпфере (прогоны LV1/LV3 2026-08-19: дрейф 1.3 м/с до fence
        при ЗДОРОВОМ VINS). С keep+[vins] композит пишет все три оси, VinsHold
        поверх забирает roll/pitch, yaw остаётся демпферу — верно и для
        раздельных стабов (оси не пересекаются, порядок безразличен)."""
        if not self.vins_ready(s):
            return False
        if self._vins in stack.stabs:
            return False                           # этот стек уже на VinsHold
        stack.switch_stabilization(self.vins_stabs(stack.stabs, s))
        self._done = True
        return True

    @property
    def switched(self) -> bool:
        return self._done
