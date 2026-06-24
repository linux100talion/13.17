#!/usr/bin/env python3
# ============================================================================
# device_util — авто-фолбэк выбора устройства torch (cuda -> cpu).
#
# Ноды NN1/NN2 по умолчанию просят device="cuda" (боевой Orin / GPU-sim). На
# машине без GPU (ветка nn2_c3_cpu, GPU-less прогон) такой запрос валит ноду на
# загрузке модели (torch: no CUDA device). resolve_device() повторяет приём из
# tools/nn2_route/train_route_coords.py: если cuda недоступна — молча уходим на
# cpu (с предупреждением), вместо краша. cpu остаётся cpu.
# ============================================================================


def resolve_device(requested, logger=None):
    """Вернуть рабочее имя устройства torch.

    requested — что просили ("cuda" / "cpu" / "cuda:0" ...). Если просили cuda,
    но torch.cuda недоступна — фолбэк на "cpu". Иначе — что просили.
    logger — опц. ROS-логгер для предупреждения о фолбэке.
    """
    requested = (requested or "cpu").strip()

    if not requested.startswith("cuda"):
        return requested

    try:
        import torch
        if torch.cuda.is_available():
            return requested
        reason = "CUDA недоступна (нет GPU/драйвера)"
    except Exception as e:                      # torch без CUDA-сборки и т.п.
        reason = f"проверка torch.cuda не удалась ({e})"

    msg = f"device={requested} запрошен, но {reason} — фолбэк на cpu"
    if logger is not None:
        logger.warn(msg)
    else:
        print(f"[device_util] {msg}")
    return "cpu"
