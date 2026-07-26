#!/usr/bin/env python3
"""PlanRunner — обобщённый секвенсор полётного задания (список Step'ов).

Заменяет захардкоженный MissionRunner FSM: bootstrap теперь просто ОДИН план (список
шагов). Новое задание = данные, а не новый класс. Переходы: NEXT (следующий), GOTO
(прыжок по имени — напр. аборт climb → land), FINISH (конец). Зависит только от портов
(Clock/FlightMode/Logger) — ни строчки rclpy. Возвращает RcCommand, публикует нода.

ctx-хелперы (шаги их зовут): elapsed() — sim-время в текущем шаге (ленивое базирование);
try_cmd(fn) — троттлинг сервис-вызовов ~1/sim-сек; keep_mode() — ре-ассерт режима.
"""
from control_pkg.domain.rc import RcCommand

from .step import FINISH, GOTO, NEXT


class PlanRunner:
    def __init__(self, steps, clock, flight_mode, log):
        self.steps = steps
        self.clock = clock
        self.mode = flight_mode
        self.log = log
        self._by_name = {st.name: i for i, st in enumerate(steps)}
        self.i = 0
        self.finished = False
        self.result = "?"
        self._entered = False
        self._t0 = None                 # ленивое базирование времени шага
        self._last_cmd = -1e9
        self._last_mode_assert = -1e9

    # --- ctx-хелперы для шагов (sim-время) ---
    def now(self):
        return self.clock.now_sim()

    def elapsed(self):
        if self._t0 is None:            # первый тик шага с живым /clock задаёт точку отсчёта
            self._t0 = self.now()
            return 0.0
        return self.now() - self._t0

    def try_cmd(self, fn):
        if self.now() - self._last_cmd >= 1.0:
            self._last_cmd = self.now()
            fn()

    def keep_mode(self, s, mode):
        # ре-ассерт режима (страховка на смену полётником); '' — транзиент до heartbeat
        if s.mode not in (None, "", mode) and self.now() - self._last_mode_assert >= 2.0:
            self._last_mode_assert = self.now()
            self.log.warn(f"режим={s.mode} ≠ {mode} — ре-ассерт")
            self.mode.set_mode(mode)

    # --- секвенсор ---
    def _jump(self, idx):
        if idx >= len(self.steps):
            self._end()
            return
        self.i = idx
        self._entered = False           # перевход на следующем тике

    def _end(self):
        self.finished = True

    def tick(self, s) -> RcCommand:
        if self.finished:
            return RcCommand()
        st = self.steps[self.i]
        if not self._entered:
            self.log.info(f">>> шаг {self.i} · {st.name}")
            self._t0 = None
            self._last_cmd = -1e9
            self._last_mode_assert = -1e9
            st.enter(self, s)
            self._entered = True
        res = st.tick(self, s)
        if res.result:
            self.result = res.result
        if res.status == NEXT:
            self._jump(self.i + 1)
        elif res.status == GOTO:
            self._jump(self._by_name[res.goto])
        elif res.status == FINISH:
            self._end()
        if self.finished:
            self.log.info(f">>> ИТОГ: {self.result} (mode={s.mode}, armed={s.armed}, "
                          f"rel_alt={s.rel_alt}, odom={s.vins_odom_count})")
        return res.rc
