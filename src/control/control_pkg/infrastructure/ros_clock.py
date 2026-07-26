#!/usr/bin/env python3
"""RosClock — адаптер порта Clock: sim-время по /clock (use_sim_time)."""


class RosClock:
    def __init__(self, node):
        self._node = node

    def now_sim(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9
