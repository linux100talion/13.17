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

    def maybe_switch(self, stack, s) -> bool:
        """Если VINS сошёлся, а ЭТОТ стек ещё на демпфере — заменить его
        стабилизаторы на VinsHold. Возвращает True на тике переключения (лог)."""
        if not self.vins_ready(s):
            return False
        if stack.stabs == [self._vins]:
            return False                           # этот стек уже на VinsHold
        self._vins.enter(s)                        # опора = текущая точка (семантика шага)
        stack.switch_stabilization(self._vins)     # Flow+Yaw → VinsHold
        self._done = True
        return True

    @property
    def switched(self) -> bool:
        return self._done
