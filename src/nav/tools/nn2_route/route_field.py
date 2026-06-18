#!/usr/bin/env python3
# ============================================================================
# route_field.py — сборка поля f = −∇V из (s,e) + визуальный сервоинг.
#
# Теория — nn2_navigation_dream.txt (XIII–XV). Потенциал и поле:
#   V(x) = −α·s + ½β·e²
#   f = −∇V = α·∇s − β·e·∇e
# Ключ: ∇s и ∇e через route-геометрию = ФИЗИЧЕСКИЕ направления (касательная T и
# нормаль n маршрута), поэтому поле собирается БЕЗ дескрипторного якобиана:
#   v_route = α·T̂(σ) − β·e·n̂(σ),   σ = s·L            (route/world-рамка)
# Эта же геометрия даёт засечку p̂ = R(σ) + e·n(σ) (грань 2, см. relocalizer).
#
# RouteField — чистый numpy (тестируем). RouteCoords — torch-загрузчик голов
# s(f),e(f) из train_route_coords.pt. VisualServo — НАБРОСОК рулёжки, когда курса
# нет (потеря VINS): гомингуем к дескриптору следующей крошки.
# ============================================================================
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_geometry import Centerline                      # noqa: E402


# ============================================================================
# 1. ПОЛЕ −∇V в route-рамке (чистый numpy)
# ============================================================================
class RouteField:
    """Из (s,e) собирает желаемую скорость v = −∇V в рамке маршрута/мира.

    α — тяга «вперёд по маршруту», β — тяга «назад на нить». speed — модуль
    желаемой скорости (None -> без нормировки, вернёт «сырое» −∇V).
    """
    def __init__(self, centerline: Centerline, alpha=1.0, beta=0.3, speed=2.0):
        self.cl = centerline
        self.alpha, self.beta, self.speed = float(alpha), float(beta), float(speed) \
            if speed is not None else None

    def velocity_route(self, s, e):
        """(s,e) -> 2D скорость в рамке мира (где построена центрлиния)."""
        _, T, n = self.cl.frame_at(s * self.cl.L)
        v = self.alpha * T - self.beta * float(e) * n        # = −∇V
        if self.speed:
            nrm = float(np.linalg.norm(v))
            if nrm > 1e-9:
                v = v / nrm * self.speed
        return v

    def position(self, s, e):
        """(s,e) -> p̂ = R(σ) + e·n(σ): точка для засечки (грань 2)."""
        R, _, n = self.cl.frame_at(s * self.cl.L)
        return R + float(e) * n

    def command_body(self, s, e, yaw):
        """v_route -> body-рамка по известному курсу yaw (мир->тело, поворот −yaw).

        Применимо, когда курс есть (VINS/IMU-yaw/магнитометр). Без курса — см.
        VisualServo.
        """
        v = self.velocity_route(s, e)
        c, sn = np.cos(yaw), np.sin(yaw)
        return np.array([c * v[0] + sn * v[1], -sn * v[0] + c * v[1]])


# ============================================================================
# 2. ГОЛОВЫ s(f), e(f) из чекпойнта train_route_coords.pt (torch)
# ============================================================================
class RouteCoords:
    """Загружает обученные головы + центрлинию; даёт (s,e) по дескриптору и
    дескрипторные градиенты ∇s,∇e (для анализа; на руль их НАПРЯМУЮ не подашь —
    нужен image-якобиан, см. VisualServo)."""
    def __init__(self, ckpt_path, device="cpu"):
        import torch
        from nav_pkg.nn2.scene_descriptor import MetricHead
        ckpt = torch.load(str(ckpt_path), map_location=device)
        D, H = ckpt["backbone_dim"], ckpt["hidden"]
        self.torch = torch
        self.device = device
        self.s_head = MetricHead(D, H, 1).to(device).eval()
        self.e_head = MetricHead(D, H, 1).to(device).eval()
        self.s_head.load_state_dict(ckpt["s_head"])
        self.e_head.load_state_dict(ckpt["e_head"])
        self.centerline = Centerline(ckpt["centerline_verts"])

    def coords(self, descriptor):
        """Сырой CLS-дескриптор (D,) -> (s∈[0,1], e в метрах)."""
        t = self.torch.from_numpy(np.asarray(descriptor, np.float32)).to(self.device)[None]
        with self.torch.no_grad():
            s = self.torch.sigmoid(self.s_head(t)).item()
            e = self.e_head(t).item()
        return s, e

    def grad_coords(self, descriptor):
        """∇s,∇e по дескриптору (autograd). Анализ/диагностика, не команда руля."""
        t = self.torch.from_numpy(np.asarray(descriptor, np.float32)).to(self.device)[None]
        t.requires_grad_(True)
        s = self.torch.sigmoid(self.s_head(t)).sum()
        gs = self.torch.autograd.grad(s, t, retain_graph=True)[0][0].cpu().numpy()
        e = self.e_head(t).sum()
        ge = self.torch.autograd.grad(e, t)[0][0].cpu().numpy()
        return gs, ge


# ============================================================================
# 3. ВИЗУАЛЬНЫЙ СЕРВОИНГ без курса (НАБРОСОК; потеря VINS)
# ============================================================================
class VisualServo:
    """Когда абсолютного курса нет (VINS потерян, магнитометр под РЭБ): рулим
    ЧИСТО ВИЗУАЛЬНО — гонимся за дескриптором СЛЕДУЮЩЕЙ крошки маршрута.

    Несущая идея (XI): не «лети на азимут N», а «двигайся так, чтобы дескриптор
    кадра приближался к дескриптору крошки s+lookahead». Поле даёт ЦЕЛЬ (вперёд
    по s), реализация — снижение дескрипторной ошибки.

    ⚠ НАБРОСОК. Перевод «уменьшить ||desc − target||» в команду руля требует
    image-якобиана ∂desc/∂motion, которого аналитически нет. Рабочие пути:
      (a) bearing известного ориентира в кадре (NN1/фичи) -> P-регулятор по yaw;
      (b) онлайн-оценка якобиана пробными микродвижениями (observe Δdesc);
    здесь — каркас (a)+заглушка (b), числа калибруются на стенде/симе.
    """
    def __init__(self, breadcrumb_descs, breadcrumb_s, lookahead=0.03,
                 k_yaw=1.0, fwd_speed=2.0):
        self.D = np.asarray(breadcrumb_descs, np.float32)   # (M, dim) крошки
        self.s = np.asarray(breadcrumb_s, np.float64)       # (M,) их s*
        self.lookahead = float(lookahead)
        self.k_yaw, self.fwd_speed = float(k_yaw), float(fwd_speed)

    def next_target(self, s_now):
        """Крошка на lookahead впереди по маршруту -> её индекс."""
        return int(np.argmin(np.abs(self.s - (s_now + self.lookahead))))

    def command(self, descriptor, s_now, target_bearing=None):
        """-> (yaw_rate, forward_speed). target_bearing (рад) — пеленг крошки в
        кадре (из NN1/фич), если есть: путь (a). Нет -> только «вперёд» + TODO(b)."""
        j = self.next_target(s_now)
        err = float(np.linalg.norm(self.D[j] - np.asarray(descriptor, np.float32)))

        if target_bearing is not None:                # путь (a): по пеленгу
            yaw_rate = self.k_yaw * target_bearing
        else:                                         # путь (b): TODO probe-якобиан
            yaw_rate = 0.0
        # вперёд, пока не «съели» сходство; темп можно гасить по err
        forward = self.fwd_speed
        return yaw_rate, forward, {"target_idx": j, "desc_err": err}


# --- самопроверка поля (только numpy) ---------------------------------------
def _selftest():
    from route_geometry import build_centerline
    route = np.stack([np.linspace(0, 100, 50), np.zeros(50)], axis=1)   # вдоль +x
    cl = build_centerline(route, smooth_window=1, resample_ds=5.0)
    fld = RouteField(cl, alpha=1.0, beta=0.5, speed=None)

    v0 = fld.velocity_route(0.5, 0.0)            # на нити -> чистое «вперёд»
    assert np.allclose(v0, [1, 0], atol=1e-3), v0
    vL = fld.velocity_route(0.5, 2.0)            # слева (+y) -> тянет на −y
    assert vL[0] > 0 and vL[1] < 0, vL
    vR = fld.velocity_route(0.5, -2.0)           # справа -> тянет на +y
    assert vR[0] > 0 and vR[1] > 0, vR

    p = fld.position(0.5, 2.0)                   # засечка p̂ = R(50,0)+2·(0,1)
    assert np.allclose(p, [50, 2], atol=1e-3), p

    cb = fld.command_body(0.5, 0.0, 0.0)         # yaw=0 -> body==world
    assert np.allclose(cb, v0, atol=1e-3), cb
    print("route_field selftest: OK (v на нити/сбоку, p̂, body@yaw=0)")


if __name__ == "__main__":
    _selftest()
