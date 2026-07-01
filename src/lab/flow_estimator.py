#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FlowEstimator — чистая зрительная часть FLOW-DAMP: кадр + гироскоп → агрегаты потока.

БЕЗ ROS (только numpy + cv2) — нарочно изолирована (спека §8): шарится между боевой
нодой `flow_damp_node.py` и оффлайн-тестом `flow_derotation_check.py`, юнит-тестится сама.

Пайплайн (см. FAQ_vins.md 6-11):
  sparse LK (поток между соседними кадрами)
    → derotate: вычесть ВРАЩАТЕЛЬНЫЙ поток (ω_cam = R · ω_imu), формула Longuet-Higgins
    → остаточный ТРАНСЛЯЦИОННЫЙ поток
    → агрегаты: боковой (медиана горизонт.) + диагностика (RMS остатка/измерения, |ω|).

rotflow_sign: множитель вращательной поправки. +1/−1 — два знака; 0 — БЕЗ derotation
(baseline). Перебором {R, Rᵀ}×{±1} оффлайн-тест выбирает верный вариант по минимуму
остатка на кадрах с большим |ω| (чистое вращение → правильная derotation → остаток≈0).
"""

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


class FlowEstimator:
    def __init__(self, fx, fy, cx, cy, R_cam_imu, rotflow_sign=1.0, max_feats=200,
                 smooth_n=1, yaw_smooth_n=1):
        if cv2 is None:
            raise RuntimeError('cv2 не найден — FlowEstimator не работает')
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.R = np.asarray(R_cam_imu, dtype=np.float64).reshape(3, 3)
        self.rotflow_sign = float(rotflow_sign)
        self.max_feats = max_feats
        # ВРЕМЕННОЕ СГЛАЖИВАНИЕ lateral: медиана по N кадрам. Шум потока БЕЛЫЙ
        # (автокорр≈0, см. flow_calib) → усреднение по N режет пол как √N, а сигнал
        # (боковая скорость) на низкой частоте почти не смазывается. Лаг ~N/2 кадров
        # мал и петля к нему нечувствительна (τ-развёртка в flow_loop_sim). 1 = выкл.
        self.smooth_n = max(1, int(smooth_n))
        self._lat_buf = []
        self.yaw_smooth_n = max(1, int(yaw_smooth_n))   # сглаживание визуального yaw
        self._yaw_buf = []
        self.prev_gray = None
        self.prev_pts = None
        self.prev_stamp = None
        self._lk = dict(winSize=(21, 21), maxLevel=3,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        self._feat = dict(maxCorners=max_feats, qualityLevel=0.01, minDistance=8, blockSize=7)

    def _detect(self, gray):
        return cv2.goodFeaturesToTrack(gray, mask=None, **self._feat)

    def _rot_flow(self, p0, wx, wy, wz, dt):
        """Вращательный поток (пиксели/кадр) в точках p0 для ω камеры (rad/s)."""
        # нормированные координаты
        xn = (p0[:, 0] - self.cx) / self.fx
        yn = (p0[:, 1] - self.cy) / self.fy
        # Longuet-Higgins/Prazdny, нормир. плоскость, ×dt. TODO[sign]: знаки сверяет тест.
        u_rot_n = (xn * yn * wx - (1.0 + xn ** 2) * wy + yn * wz) * dt
        v_rot_n = ((1.0 + yn ** 2) * wx - xn * yn * wy - xn * wz) * dt
        u_rot = self.fx * u_rot_n
        v_rot = self.fy * v_rot_n
        return np.column_stack([u_rot, v_rot])

    def process(self, gray, stamp, omega_imu):
        """gray: uint8 HxW; stamp: сек; omega_imu: ω в FLU (rad/s). → dict | None."""
        out = None
        if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) > 0:
            dt = max(1e-3, stamp - self.prev_stamp)
            nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray,
                                                  self.prev_pts, None, **self._lk)
            st = st.reshape(-1).astype(bool)
            p0 = self.prev_pts.reshape(-1, 2)[st]
            p1 = nxt.reshape(-1, 2)[st]
            n = len(p0)
            if n >= 8:
                flow = p1 - p0                                  # измеренный поток (px/кадр)
                # ω в фрейме камеры: ω_cam = R · ω_imu
                w = self.R @ np.asarray(omega_imu, dtype=np.float64)
                if self.rotflow_sign != 0.0:
                    rot = self.rotflow_sign * self._rot_flow(p0, w[0], w[1], w[2], dt)
                else:
                    rot = np.zeros_like(flow)                   # baseline: без derotation
                tr = flow - rot                                 # трансляционный остаток
                lateral_raw = float(np.median(tr[:, 0]))        # v0: прокси бокового сноса
                # временное сглаживание (медиана по N кадрам) — режет белый шум ~√N
                self._lat_buf.append(lateral_raw)
                if len(self._lat_buf) > self.smooth_n:
                    self._lat_buf.pop(0)
                lateral = float(np.median(self._lat_buf)) if self.smooth_n > 1 else lateral_raw
                # --- ВИЗУАЛЬНЫЙ YAW (фаза 2): derotate ТОЛЬКО roll+pitch (гиро x,y —
                # гравитация-референс, НЕ дрейфуют), yaw гиро НЕ вычитаем → остаток =
                # yaw-вращение + трансляция. В ДАЛЬНЕЙ сцене трансляция ≈0 (тот самый
                # depth, что убил боковую ось) → остаток ≈ чистый визуальный yaw.
                oi = np.asarray(omega_imu, dtype=np.float64)
                w_ny = self.R @ np.array([oi[0], oi[1], 0.0])   # FLU: yaw (z) обнулён
                rot_ny = self._rot_flow(p0, w_ny[0], w_ny[1], w_ny[2], dt)
                yaw_flow_raw = float(np.median((flow - rot_ny)[:, 0]))  # px/кадр ∝ визуальный yaw
                self._yaw_buf.append(yaw_flow_raw)                  # сглаживание (медиана по N)
                if len(self._yaw_buf) > self.yaw_smooth_n:
                    self._yaw_buf.pop(0)
                yaw_flow = float(np.median(self._yaw_buf)) if self.yaw_smooth_n > 1 else yaw_flow_raw
                # TODO[phase2]: дивергенция (looming) из аффинного фита tr по (xn,yn) → PITCH.
                divergence = 0.0
                out = dict(
                    lateral=lateral, lateral_raw=lateral_raw, yaw_flow=yaw_flow,
                    divergence=divergence, n=n, dt=dt,
                    conf=float(n) / float(self.max_feats),
                    # --- диагностика для flow_derotation_check ---
                    resid_rms=float(np.sqrt(np.mean(np.sum(tr ** 2, axis=1)))),
                    meas_rms=float(np.sqrt(np.mean(np.sum(flow ** 2, axis=1)))),
                    omega_norm=float(np.linalg.norm(omega_imu)),
                )
        self.prev_gray = gray
        self.prev_pts = self._detect(gray)
        self.prev_stamp = stamp
        return out
