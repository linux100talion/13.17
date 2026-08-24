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
    def __init__(self, vins_hold, min_count: int = 40, fresh_sec: float = 2.0):
        self._vins = vins_hold
        self.min_count = min_count
        self.fresh_sec = fresh_sec
        self._done = False

    def vins_ready(self, s) -> bool:
        return (s.vins_odom_count >= self.min_count and
                (s.now_sim - s.vins_last_sim) < self.fresh_sec)

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
