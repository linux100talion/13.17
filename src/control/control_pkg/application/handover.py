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


class VinsHandover:
    def __init__(self, vins_hold, min_count: int = 40, fresh_sec: float = 2.0,
                 v_max: float = 0.0, ipm_tol: float = 0.0, sane_n: int = 3):
        self._vins = vins_hold
        self.min_count = min_count
        self.fresh_sec = fresh_sec
        # ГЕЙТ ЗДОРОВЬЯ (санити): защита от разноса VINS (см. config.vins_v_max).
        # 0 = выкл. sane_n — сколько кадров подряд расхождения с IPM до вердикта
        # «болен» (защита от одиночного шумного кадра IPM; физ. потолок — сразу).
        self.v_max = v_max
        self.ipm_tol = ipm_tol
        self.sane_n = sane_n
        self._ipm_bad = 0             # кадров подряд |vins_v−ipm_v| > tol
        self._last_sane_t = None      # счётчик двигаем раз на новый sim-тик
        self._done = False

    def vins_sane(self, s) -> bool:
        """VINS вменяем: скорость физически возможна И (если IPM жив) согласна с
        независимым оптическим каналом. Разнос (|v|→20 на неподвижном борте)
        проваливает оба. Модуль скорости инвариантен к системе координат, так
        что vins (мир) и ipm (тело) сравнимы напрямую."""
        import math
        vh = math.hypot(s.vins_vx, s.vins_vy)
        if self.v_max > 0.0 and vh > self.v_max:
            return False                      # физически невозможно = мусор (грубо)
        # IPM-кросс-чек с защитой от одиночного шумного кадра (sane_n подряд).
        # Счётчик двигаем раз на новый sim-тик — метод зовут оба пути лесенки.
        new_tick = self._last_sane_t is None or s.now_sim > self._last_sane_t
        if self.ipm_tol > 0.0 and s.ipm_ok:
            iv = math.hypot(s.ipm_vfwd, s.ipm_vlat)
            bad = abs(vh - iv) > self.ipm_tol
            if new_tick:
                self._ipm_bad = self._ipm_bad + 1 if bad else 0
        elif new_tick:
            self._ipm_bad = 0                 # IPM слеп — чек не судит, не копим
        if new_tick:
            self._last_sane_t = s.now_sim
        return self._ipm_bad < self.sane_n

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
        self._vins.enter(s)
        return keep + [self._vins]

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
