"""WB-WAM 20 Hz action contract and deterministic 50 Hz execution adapter."""

from __future__ import annotations

import numpy as np

MODEL_HORIZON = 32
MODEL_ACTION_DIM = 66
MODEL_HZ = 20
EXEC_HORIZON = 50
EXEC_HZ = 50


def validate_native_actions(value) -> np.ndarray:
    actions = np.asarray(value, dtype=np.float32)
    expected = (MODEL_HORIZON, MODEL_ACTION_DIM)
    if actions.shape != expected or not np.isfinite(actions).all():
        raise ValueError(f"expected finite native actions {expected}, got {actions.shape}")
    return actions


def execution_source_indices() -> np.ndarray:
    """Map each 50 Hz tick to the latest available 20 Hz model row."""
    ticks = np.arange(EXEC_HORIZON, dtype=np.int64)
    indices = np.floor(ticks * MODEL_HZ / EXEC_HZ).astype(np.int64)
    if indices[-1] >= MODEL_HORIZON:
        raise AssertionError("execution horizon exceeds native model horizon")
    return indices


def zero_order_hold_20hz_to_50hz(value) -> tuple[np.ndarray, np.ndarray]:
    """Hold latent/Dex1 rows on the physical timeline; never interpolate tokens."""
    native = validate_native_actions(value)
    indices = execution_source_indices()
    return np.ascontiguousarray(native[indices]), indices


def validate_execution_actions(value) -> np.ndarray:
    actions = np.asarray(value, dtype=np.float32)
    expected = (EXEC_HORIZON, MODEL_ACTION_DIM)
    if actions.shape != expected or not np.isfinite(actions).all():
        raise ValueError(f"expected finite execution actions {expected}, got {actions.shape}")
    return actions
