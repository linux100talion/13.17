#!/usr/bin/env python3
"""Стратегии стабилизации — ДВА семейства + per-axis алиасы. ФАСАД-реэкспорт.

Код разъехался по модулям (2026-09-01, бывший монолит 1430 строк), импорты
`from ...stabilization import X` работают как раньше — ничего не переносить:

- `gz_hold.py` — **Gz\\***: держит ПОЗИЦИЮ по ground-truth Gazebo (sim-оракул для
  тюнинга). GzHold база + алиасы GzPosHold/GzRollHold/GzPitchHold/GzYawHold.
- `flow_damper.py` — **_FlowDamper1D**: общая база Dp* (покадровая интеграция,
  conf-blend, hold+fade, станция-кипинг BRAKE/RETURN, трим ветра, мягкость).
- `flow_axes.py` — **Dp\\*** оси по полнокадровому потоку: DpRollHold/DpPitchHold/
  DpYawHold + зонд DpPitchBack + композит DpHold (все три оси).
- `ipm_axes.py` — оси по МЕТРИЧЕСКОМУ каналу вида сверху: _IpmGated (гейт
  доверия) + DpPitchRate/DpRollRate.
- `alt_settled.py` — _AltSettled («высота успокоилась» без дифференциатора).
- `station_frame.py` — StationFrame (станция в осях курса, общая рама).
- `vins_hold.py` — VinsHold (position-hold по VINS после init, своя опора).
- `passthrough.py` — PilotPassthrough (легаси).

Незанятые оси раздаёт пилоту сам ControlStack (per-axis база = сырые стики).
"""
from ..rc import RC_CENTER, RcCommand, clamp                                # noqa: F401
from ..setpoint import Setpoint                                             # noqa: F401
from ..state import DroneState                                              # noqa: F401
from .alt_settled import _AltSettled                                        # noqa: F401
from .base import StabilizationStrategy                                     # noqa: F401
from .flow_axes import DpHold, DpPitchBack, DpPitchHold, DpRollHold, DpYawHold  # noqa: F401
from .flow_damper import _blend, _FlowDamper1D                              # noqa: F401
from .gz_hold import GzHold, GzPitchHold, GzPosHold, GzRollHold, GzYawHold  # noqa: F401
from .ipm_axes import DpPitchRate, DpRollRate, _IpmGated                    # noqa: F401
from .passthrough import PilotPassthrough                                   # noqa: F401
from .station_frame import StationFrame                                     # noqa: F401
from .vins_hold import VinsHold                                             # noqa: F401
