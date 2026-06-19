#!/usr/bin/env python3
# ============================================================================
# route_heads.py — ШАГ 5 (c)-основного: РАНТАЙМ голов C (φ -> s,e) + OOD-страж (XXIII).
#
# Зачем: офлайн-пайплайн (ШАГ 1-4) выпёк головы s(φ),e(φ) (train_route_coords.pt).
# В рантайме (c)-основного route-слой = ОДИН forward-pass БЕЗ FAISS: φ -> головы ->
# s,e -> поле −∇V. Этот модуль и есть тот forward-pass (инференс-сторона
# fit_route_heads) + страж XXIII: метрика (φ-карта FAISS) НЕ фьюзится слепо, а
# служит OOD-ДЕТЕКТОРОМ — если route-поза далеко (много σ) от метрики, топограф
# либо поплыл вне 100 облётов, либо алиасинг -> не доверяем, флажок наверх.
#
# (c)-ОСНОВНОЙ РАЗВОД ПРОВОДОВ (XXIII): головы C — ПЕРВИЧНЫЙ s,e для ОБОИХ проводов
# (локализация + рулёжка). FAISS/метрика — ТОЛЬКО локализац. провод (страховка/
# восстановление), НЕ в route-рантайме. s_filter (XXII) остаётся мостом к VINS.
#
# Реальные головы = MetricHead (torch) из train_route_coords.pt — ленивым импортом.
# Numpy-стенд-ин LinearRouteHeads — для среды без torch и как явный fallback-контракт.
# Чистый numpy в стенд-ине/страже — тестируется.
# ============================================================================
import numpy as np


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.asarray(x, np.float64)))


class RouteHeads:
    """Рантайм голов C из train_route_coords.pt: φ -> (s∈[0,1], e м). torch ленивый.
    Веса грузим один раз; infer_feat — горячий путь (один forward-pass, без FAISS)."""
    def __init__(self, s_head, e_head, centerline_L, backbone_dim, device="cpu",
                 encoder=None, _torch=None):
        self.s_head = s_head
        self.e_head = e_head
        self.L = float(centerline_L)
        self.backbone_dim = int(backbone_dim)
        self.device = device
        self.encoder = encoder            # SceneEncoder для infer_frame (опц.)
        self._torch = _torch

    @classmethod
    def load(cls, ckpt_path, device="cpu", with_encoder=False):
        """Грузим обе головы + центрлинию из чекпойнта (format train_route_coords.main:
        s_head/e_head state_dict, backbone_dim, hidden, centerline_L/_verts, model)."""
        import torch                                              # noqa: E402
        from nav_pkg.nn2.scene_descriptor import MetricHead       # noqa: E402
        ck = torch.load(str(ckpt_path), map_location=device)
        din, hid = int(ck["backbone_dim"]), int(ck["hidden"])
        s_head = MetricHead(in_dim=din, hidden=hid, out_dim=1).to(device).eval()
        e_head = MetricHead(in_dim=din, hidden=hid, out_dim=1).to(device).eval()
        s_head.load_state_dict(ck["s_head"])
        e_head.load_state_dict(ck["e_head"])
        enc = None
        if with_encoder:
            from nav_pkg.nn2.scene_descriptor import SceneEncoder
            enc = SceneEncoder(model_name=ck.get("model", "dinov2_vits14"), device=device)
        return cls(s_head, e_head, float(ck["centerline_L"]), din, device, enc, torch)

    def infer_feat(self, feat):
        """φ (D,) -> (s, e). Горячий путь: НЕТ поиска по базе, чистый forward-pass."""
        t = self._torch
        with t.no_grad():
            f = t.from_numpy(np.asarray(feat, np.float32)[None, :]).to(self.device)
            s = float(t.sigmoid(self.s_head(f)).squeeze().cpu().numpy())
            e = float(self.e_head(f).squeeze().cpu().numpy())
        return s, e

    def infer_frame(self, frame):
        """Кадр -> φ (DINOv2) -> (s,e). Требует encoder (with_encoder=True при load).
        В nn2_scene φ уже считается для FAISS — там лучше переиспользовать его и
        звать infer_feat (без второго прогона DINOv2)."""
        if self.encoder is None:
            raise RuntimeError("infer_frame требует encoder (load(..., with_encoder=True))")
        feat = self.encoder.encode(frame, normalize=False)
        return self.infer_feat(feat)


class LinearRouteHeads:
    """Numpy-стенд-ин голов C (без torch): s=σ(φ·Ws+bs), e=φ·We+be. Тот же контракт
    infer_feat, что у RouteHeads. Для среды без torch и как явный fallback. Веса
    можно подогнать офлайн (lstsq) — здесь просто носитель готовых весов."""
    def __init__(self, Ws, bs, We, be, centerline_L=1.0):
        self.Ws = np.asarray(Ws, np.float64); self.bs = float(bs)
        self.We = np.asarray(We, np.float64); self.be = float(be)
        self.L = float(centerline_L)

    def infer_feat(self, feat):
        f = np.asarray(feat, np.float64)
        return float(_sigmoid(f @ self.Ws + self.bs)), float(f @ self.We + self.be)


def metric_ood_gate(p_route, metric, k_sigma=3.0):
    """OOD-страж XXIII: согласна ли route-поза с метрикой (φ-карта FAISS)?
    p_route (2,) — поза из голов C (p̂=R(σ)+e·n); metric=(p_m (2,), C_m (2,2), mconf).
    Махаланобис route-позы под ковариацией метрики. <=k_sigma -> согласны (фьюзить
    можно); > -> РАСХОЖДЕНИЕ: топограф поплыл вне распределения ИЛИ алиасинг ->
    метрику в фьюз НЕ берём, наверх флажок (раздуть σ_s в s_filter, сбить conf).
    Без метрики (None) или mconf низкого -> страж не судит (ok=True, maha=0)."""
    if metric is None:
        return True, 0.0
    p_m, C_m, mconf = metric
    d = np.asarray(p_route, np.float64) - np.asarray(p_m, np.float64)
    try:
        maha = float(np.sqrt(max(d @ np.linalg.solve(np.asarray(C_m, np.float64), d), 0.0)))
    except np.linalg.LinAlgError:
        return True, 0.0                                 # вырожденная cov -> не судим
    return maha <= float(k_sigma), maha


# --- самопроверка (numpy): рантайм-контракт + OOD-страж -----------------------
def _selftest():
    rng = np.random.default_rng(0)
    D = 16
    # стенд-ин голов: s растёт вдоль некой оси φ, e — вдоль другой
    Ws = rng.normal(0, 0.3, D); We = rng.normal(0, 0.5, D)
    heads = LinearRouteHeads(Ws, bs=0.0, We=We, be=0.0, centerline_L=100.0)
    feat = rng.normal(0, 1, D).astype(np.float32)
    s, e = heads.infer_feat(feat)
    assert 0.0 <= s <= 1.0, s
    assert np.isfinite(e)
    # батч-контракт: разные φ -> разные s (голова не константа)
    ss = [heads.infer_feat(rng.normal(0, 1, D))[0] for _ in range(50)]
    assert np.std(ss) > 0.05, np.std(ss)

    # OOD-страж: route-поза рядом с метрикой -> согласны; далеко -> расхождение
    C_m = np.array([[4.0, 0.0], [0.0, 4.0]])             # σ=2 м
    metric = (np.array([50.0, 30.0]), C_m, 0.9)
    ok_near, m_near = metric_ood_gate(np.array([51.0, 30.5]), metric, k_sigma=3.0)
    ok_far, m_far = metric_ood_gate(np.array([50.0 + 12.0, 30.0]), metric, k_sigma=3.0)
    assert ok_near and m_near < 3.0, (ok_near, m_near)
    assert (not ok_far) and m_far > 3.0, (ok_far, m_far)
    # без метрики страж не судит
    ok_none, m_none = metric_ood_gate(np.array([0.0, 0.0]), None)
    assert ok_none and m_none == 0.0

    print("route_heads selftest: OK")
    print(f"  стенд-ин голов: s={s:.3f}∈[0,1], e={e:.2f} м; std(s) по 50 φ = {np.std(ss):.2f}")
    print(f"  OOD-страж: рядом maha={m_near:.2f}<=3 -> фьюз; далеко maha={m_far:.2f}>3 -> метрику не берём")
    print("  -> рантайм голов C = forward-pass без FAISS; метрика XXIII = OOD-детектор, не слепой фьюз")
    print("  (реальные головы: RouteHeads.load(train_route_coords.pt) — torch, на борту/симуляции)")


if __name__ == "__main__":
    _selftest()
