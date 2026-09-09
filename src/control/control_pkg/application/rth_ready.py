#!/usr/bin/env python3
"""RthReadiness — ЛАТЧ ДОВЕРИЯ К ВОЗВРАТУ ДОМОЙ (чистая политика, без ROS).

Возврат домой ведёт FCU по позиции EKF, а в LV=2 позиция EKF держится ТОЛЬКО
подтяжкой vision_pose от нашего моста. Значит «можно ли возвращаться» = «цела ли
рама, в которой записан дом». Полёты 044105 и 073004 показали оба способа её
порвать: ложный вердикт гейта посреди возврата и разнос VINS сразу после отрыва
(EKF уехал на 200 м, и SMART_RTL «вернул» борт в 52 м от старта — по своим,
уже неверным цифрам).

Отсюда правило пилота (2026-09-09): в первых метрах после отрыва можно ЧТО
УГОДНО — там VINS init'ится, перерождается, лечится; но как только борт вышел
из круга лечения, рама обязана остаться непрерывной. Порвалась — возврата нет,
и это ЧЕСТНО показано (🔴 в HUD), а не выясняется в воздухе.

Машина состояний:

    HEAL   — от арма. Круг лечения: |смещение| ≤ radius (считаем ПО IPM — VINS
             в этой фазе может врать) И время от отрыва ≤ heal_sec (таймаут не
             зависит от гейна канала и ловит «висим и не лечимся»).
      ↓ латч: VINS зрел (odom ≥ min_count) и здоров непрерывно ripe_sec
    READY  — дом ЗАПИСАН (поза EKF на момент латча) + пишется трек пути.
             Отсюда возврат разрешён.
      ↓ разрыв рамы
    LOST   — терминально (relatch=False): перерождение VINS, вердикт «болен»,
             закрытый мост ПОСЛЕ латча; либо вышли из круга/истёк таймаут, не
             успев залатчиться. Возврат запрещён, борт домой ведёт пилот.

Отдельный выход — `ripe`: «VINS доказал зрелость» БЕЗ привязки к кругу. По нему
лётная нода разрешает мосту публиковать vision_pose (гейт зрелости моста): пока
VINS не доказал себя, полётник не получает от него ни одной позы и отравить его
нечем (в 073004 хватило 2–4 с мусора, чтобы EKF уехал на 200 м).

Смещение по IPM считается тем же приёмом, что StationFrame: приращения
ipm_fwd/ipm_lat поворачиваются по курсу AHRS. Свой экземпляр, потому что рама
станции сбрасывается на входах в ярус, а нам нужен отсчёт ОТ АРМА.
"""
import math


class RthReadiness:
    HEAL, READY, LOST = 'heal', 'ready', 'lost'

    def __init__(self, radius: float = 5.0, heal_sec: float = 30.0,
                 ripe_sec: float = 5.0, min_count: int = 300,
                 fresh_sec: float = 2.0, track_m: float = 3.0,
                 track_max: int = 4000, relatch: bool = False):
        self.radius = float(radius)
        self.heal_sec = float(heal_sec)
        self.ripe_sec = float(ripe_sec)
        self.min_count = int(min_count)
        self.fresh_sec = float(fresh_sec)
        self.track_m = float(track_m)
        self.track_max = int(track_max)
        self.relatch = bool(relatch)
        self.reset()

    # --- состояние -----------------------------------------------------------
    def reset(self) -> None:
        self.state = self.HEAL
        self.why = ''
        self.ripe = False           # зрелость для моста (без привязки к кругу)
        self.home = None            # (x, y, z) поза EKF в момент латча
        self.track = []             # [(x, y, z)] от дома, шаг track_m
        self.track_full = False
        self.path_m = 0.0           # длина записанного трека, м
        self.dist = 0.0             # |смещение| от арма по IPM, м
        self.latch_sim = None       # sim-время латча
        self._x = self._y = 0.0     # смещение по IPM (мировые оси курса)
        self._seq = -1
        self._prev = None
        self._t_arm = None
        self._ripe_since = None
        self._reb = None            # снимок счётчика перерождений на латче

    # --- ядро ----------------------------------------------------------------
    def _advance_ipm(self, s) -> None:
        """Приращение пути IPM (тело) → мировые оси по курсу AHRS. Точный ноль
        пары = сброс пути перцепцией на новом сегменте, приращения не даёт."""
        seq = int(getattr(s, 'flow_seq', 0))
        if seq == self._seq:
            return
        self._seq = seq
        cur = (float(getattr(s, 'ipm_fwd', 0.0)), float(getattr(s, 'ipm_lat', 0.0)))
        if self._prev is not None and not (cur[0] == 0.0 and cur[1] == 0.0):
            df, dl = cur[0] - self._prev[0], cur[1] - self._prev[1]
            psi = float(getattr(s, 'att_yaw', 0.0))
            c, si = math.cos(psi), math.sin(psi)
            self._x += df * c - dl * si
            self._y += df * si + dl * c
            self.dist = math.hypot(self._x, self._y)
        self._prev = cur

    def _healthy(self, s, sane: bool) -> bool:
        fresh = (s.now_sim - getattr(s, 'vins_last_sim', -1e9)) <= self.fresh_sec
        mature = int(getattr(s, 'vins_odom_count', 0)) >= self.min_count
        return bool(sane and fresh and mature)

    def _bridge_dead(self, s) -> bool:
        """Мост точно не кормит EKF. «Сообщений моста не было» (bridge_seen=False)
        — не приговор: ray_tracer может не запускаться (голый Orin), судить нечем."""
        return bool(getattr(s, 'bridge_seen', False)
                    and not getattr(s, 'bridge_open', True))

    def _broke(self, s, sane: bool):
        """Что порвало раму ПОСЛЕ латча (None = цела)."""
        if int(getattr(s, 'vins_rebirths', 0)) != self._reb:
            return 'reborn'
        if not sane:
            return 'insane'
        if self._bridge_dead(s):
            return 'bridge'
        return None

    def _record(self, s) -> None:
        x, y = getattr(s, 'ekf_x', None), getattr(s, 'ekf_y', None)
        if x is None or y is None or self.track_full:
            return
        z = getattr(s, 'ekf_z', None) or 0.0
        px, py, _pz = self.track[-1]
        if math.hypot(x - px, y - py) < self.track_m:
            return
        if len(self.track) >= self.track_max:
            self.track_full = True      # 12 км при шаге 3 м — дальше не растём
            return
        self.path_m += math.hypot(x - px, y - py)
        self.track.append((float(x), float(y), z))

    def update(self, s, sane: bool = True) -> str:
        """Один лётный тик. sane — вердикт гейта здоровья (Handover.vins_sane),
        считается НОДОЙ один раз за тик и отдаётся сюда, чтобы не двигать его
        счётчики дважды."""
        if not getattr(s, 'armed', False):
            if self._t_arm is not None:     # дизарм = конец полёта, следующий чистый
                self.reset()
            return self.state
        if self._t_arm is None:
            self._t_arm = s.now_sim
        self._advance_ipm(s)

        healthy = self._healthy(s, sane)
        if healthy:
            if self._ripe_since is None:
                self._ripe_since = s.now_sim
        else:
            self._ripe_since = None
        self.ripe = (self._ripe_since is not None
                     and s.now_sim - self._ripe_since >= self.ripe_sec)

        if self.state == self.LOST:
            return self.state
        if self.state == self.READY:
            broke = self._broke(s, sane)
            if broke is not None:
                self.state, self.why = self.LOST, broke
            else:
                self._record(s)
            return self.state
        # --- HEAL: латч, пока мы в круге и в бюджете времени ---
        # Латчимся только когда цепочка РЕАЛЬНО живая: мало «VINS созрел» — мост
        # должен уже публиковать позу (BridgeGate держит закрытие ещё hold_sec
        # после последней причины, и латч в этот момент означал бы «доверяю раме,
        # которой FCU не видит» — и тут же дисквалификацию по brg=0).
        if self.ripe and not self._bridge_dead(s):
            x, y = getattr(s, 'ekf_x', None), getattr(s, 'ekf_y', None)
            if x is not None and y is not None:
                z = getattr(s, 'ekf_z', None) or 0.0
                self.home = (float(x), float(y), z)
                self.track = [self.home]
                self.path_m = 0.0
                self.latch_sim = s.now_sim
                self._reb = int(getattr(s, 'vins_rebirths', 0))
                self.state, self.why = self.READY, ''
                return self.state
            self.why = 'no-ekf'         # зрелость есть, а позы EKF нет — ждём
        elif self.ripe:
            self.why = 'bridge-wait'    # зрелость есть, мост ещё закрыт — ждём его
        if self.dist > self.radius:
            self.state, self.why = self.LOST, 'circle'
        elif s.now_sim - self._t_arm > self.heal_sec:
            self.state, self.why = self.LOST, 'timeout'
        return self.state

    # --- наружу --------------------------------------------------------------
    def home_dist(self, s):
        """Расстояние до дома по прямой (поза EKF), None — дома нет/нет позы."""
        x, y = getattr(s, 'ekf_x', None), getattr(s, 'ekf_y', None)
        if self.home is None or x is None or y is None:
            return None
        return math.hypot(x - self.home[0], y - self.home[1])

    def status(self, s) -> str:
        """Поле rth= в /mission/status: <состояние>[:причина]/<число>.
        HEAL — сколько уже отъехали от арма; READY — сколько до дома."""
        if self.state == self.HEAL:
            return f"heal/{self.dist:.1f}"
        if self.state == self.READY:
            d = self.home_dist(s)
            return f"ready/{'--' if d is None else f'{d:.0f}'}"
        return f"lost:{self.why or '-'}"
