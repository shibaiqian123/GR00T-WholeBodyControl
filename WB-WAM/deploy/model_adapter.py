"""Load and run a native WB-WAM SONIC latent64 + Dex1 continuous checkpoint."""

from __future__ import annotations

import base64
import hashlib
import io
import os
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image

from protocol import (
    DEX1_OUTPUT_MAX_RAD,
    DEX1_OUTPUT_MIN_RAD,
    EXEC_HORIZON,
    EXEC_HZ,
    MODEL_ACTION_DIM,
    MODEL_HORIZON,
    MODEL_HZ,
    clip_dex1_targets,
    validate_execution_actions,
    zero_order_hold_20hz_to_50hz,
)
from latency_trace import mark

CONTRACT = {
    "representation": "sonic_latent64_dex1_continuous_v1",
    "state_dim": 31,
    "action_dim": 66,
    "state_order": ["sonic_body_joints_29", "left_dex1_position", "right_dex1_position"],
    "action_order": ["sonic_motion_token_64", "left_dex1_target", "right_dex1_target"],
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _resolve_checkpoint(run_dir: Path, checkpoint: str) -> Path:
    candidates = [
        run_dir / "checkpoints" / "weights" / checkpoint,
        run_dir / "weights" / checkpoint,
        run_dir / checkpoint,
    ]
    for candidate in candidates:
        if candidate.is_file() and candidate.stat().st_size > 0:
            return candidate.resolve()
    raise FileNotFoundError(f"checkpoint not found: {checkpoint} under {run_dir}")


def _load_text_cache(path: Path, torch):
    payload = torch.load(path, map_location="cpu", weights_only=True)
    if not isinstance(payload, dict) or set(payload) < {"context", "mask"}:
        raise ValueError("text cache must contain context and mask")
    context = payload["context"].clone()
    mask = payload["mask"].to(dtype=torch.bool)
    if tuple(context.shape) != (192, 4096) or tuple(mask.shape) != (192,):
        raise ValueError(f"invalid text cache shapes: {tuple(context.shape)} {tuple(mask.shape)}")
    if not torch.isfinite(context).all():
        raise ValueError("text cache contains NaN/Inf")
    context[~mask] = 0
    # Training exposes an all-valid mask after zeroing padded embeddings.
    manifest_path = path.parent / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"text cache manifest missing: {manifest_path}")
    import json
    manifest = json.loads(manifest_path.read_text())
    prompt = manifest.get("prompt")
    prompt_hash = manifest.get("sha256")
    if not isinstance(prompt, str) or not prompt.startswith("The task is: "):
        raise ValueError("text cache manifest has no training prompt")
    if hashlib.sha256(prompt.encode("utf-8")).hexdigest() != prompt_hash:
        raise ValueError("text cache manifest prompt hash mismatch")
    filename_hash = path.name.split(".", 1)[0]
    if filename_hash != prompt_hash:
        raise ValueError("text cache filename does not match manifest prompt hash")
    task = prompt.split("\n", 1)[0].removeprefix("The task is: ").strip()
    if not task:
        raise ValueError("training task is empty in text cache manifest")
    return context, torch.ones_like(mask), task, manifest_path


def _decode_image(observation: dict) -> np.ndarray:
    encoding = observation.get("image_encoding", "png")
    if encoding == "rgb8_base64":
        if observation.get("image_shape") != [224, 224, 3]:
            raise ValueError("expected RGB uint8 shape [224,224,3]")
        raw = base64.b64decode(observation["image_rgb"], validate=True)
        if len(raw) != 224 * 224 * 3:
            raise ValueError("invalid RGB payload size")
        image = np.frombuffer(raw, dtype=np.uint8).reshape(224, 224, 3)
    elif encoding == "png":
        raw = base64.b64decode(observation["image_png"], validate=True)
        image = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB").resize((224, 224), Image.Resampling.BILINEAR))
    else:
        raise ValueError(f"unsupported image encoding: {encoding}")
    if image.shape != (224, 224, 3) or image.dtype != np.uint8:
        raise ValueError("invalid decoded image")
    return image


class Policy:
    def __init__(
        self,
        *,
        run_dir: str,
        checkpoint: str,
        wbwam_root: str,
        text_cache: str,
        num_inference_steps: int,
        seed: int,
    ):
        import torch
        from hydra.utils import instantiate
        from omegaconf import OmegaConf

        self.torch = torch
        self.run_dir = Path(run_dir).resolve()
        self.checkpoint = _resolve_checkpoint(self.run_dir, checkpoint)
        self.config_path = self.run_dir / "config.yaml"
        self.stats_path = self.run_dir / "dataset_stats.json"
        self.text_cache_path = Path(text_cache).resolve()
        self.wbwam_root = Path(wbwam_root).resolve()
        for path in (self.config_path, self.stats_path, self.text_cache_path):
            if not path.is_file():
                raise FileNotFoundError(path)
        source_root = self.wbwam_root / "training" / "src"
        if not source_root.is_dir():
            raise FileNotFoundError(f"WB-WAM training source missing: {source_root}")
        sys.path.insert(0, str(source_root))
        cfg = OmegaConf.load(self.config_path)
        data = cfg.data
        model_cfg = OmegaConf.create(OmegaConf.to_container(cfg.model, resolve=True))
        if str(model_cfg.get("_target_")) != "wbwam.models.wan22.sonic_latent.create_sonic_latent":
            raise ValueError("training config is not the native SONIC latent model")
        expected = (int(data.state_dim), int(data.action_dim), int(data.num_frames) - 1)
        if expected != (31, 66, MODEL_HORIZON):
            raise ValueError(f"expected training state/action/horizon=(31,66,{MODEL_HORIZON}), got {expected}")
        if str(data.action_representation) != "native" or int(data.action_video_freq_ratio) != 4:
            raise ValueError("training config is not the validated native 20 Hz SONIC data contract")
        checkpoint_payload = torch.load(self.checkpoint, map_location="cpu", weights_only=True, mmap=True)
        if checkpoint_payload.get("sonic_contract") != CONTRACT:
            raise ValueError("checkpoint SONIC contract mismatch")
        del checkpoint_payload

        from wbwam.datasets.normalization import load_dataset_stats_from_json
        from wbwam.datasets.wb_stats import WBFlatNormalizer

        stats = load_dataset_stats_from_json(str(self.stats_path))
        self.normalizer = WBFlatNormalizer(
            stats,
            norm_default_mode=str(data.get("norm_default_mode", "min/max")),
            norm_exception_mode=data.get("norm_exception_mode"),
            use_stepwise_action_norm=bool(data.get("use_stepwise_action_norm", True)),
            norm_missing_key_mode=str(data.get("norm_missing_key_mode", "error")),
            dummy_clip_default=tuple(data.get("dummy_clip_default", (-5.0, 5.0))),
            norm_tail_scale=float(data.get("norm_tail_scale", 0.075)),
        )
        self.context, self.context_mask, self.expected_task, self.text_manifest_path = _load_text_cache(
            self.text_cache_path, torch
        )
        model_cfg.load_text_encoder = False
        model_cfg.skip_dit_load_from_pretrain = True
        model_cfg.action_dit_pretrained_path = None
        model_cfg.compile_training_denoise = False
        device = "cuda"
        dtype = torch.bfloat16
        self.model = instantiate(model_cfg, model_dtype=dtype, device=device)
        self.model.load_checkpoint(str(self.checkpoint), strict=True)
        self.model = self.model.to(device).eval().requires_grad_(False)
        self.num_inference_steps = int(num_inference_steps)
        self.seed = int(seed)
        self.info = {
            "checkpoint": str(self.checkpoint),
            "checkpoint_sha256": sha256(self.checkpoint),
            "config_sha256": sha256(self.config_path),
            "statistics_sha256": sha256(self.stats_path),
            "text_cache_sha256": sha256(self.text_cache_path),
            "text_manifest_sha256": sha256(self.text_manifest_path),
            "task_sha256": hashlib.sha256(self.expected_task.encode("utf-8")).hexdigest(),
            "state_dim": 31,
            "action_shape": [EXEC_HORIZON, MODEL_ACTION_DIM],
            "token_shape": [EXEC_HORIZON, 64],
            "native_action_shape": [MODEL_HORIZON, MODEL_ACTION_DIM],
            "native_control_hz": MODEL_HZ,
            "control_hz": EXEC_HZ,
            "time_adapter": "zero_order_hold_latest_20hz_row_on_50hz_ticks",
            "token_interpolation": False,
            "rtc_mode": "disabled",
            "rtc_enabled": False,
            "inference_steps": self.num_inference_steps,
            "seed": self.seed,
            "gripper_output_units": "motor_output_radians",
            "gripper_label_mode": "continuous",
            "gripper_output_clip_rad": [DEX1_OUTPUT_MIN_RAD, DEX1_OUTPUT_MAX_RAD],
            "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES", ""),
        }

    def predict(self, observation: dict, *, trace_id: str = "unbound") -> dict:
        torch = self.torch
        state = np.asarray(observation.get("state"), dtype=np.float32)
        if state.shape != (31,) or not np.isfinite(state).all():
            raise ValueError("expected finite raw state[31]")
        if not isinstance(observation.get("observation_id"), str):
            raise ValueError("observation_id required")
        observed_at = observation.get("observed_at")
        if not isinstance(observed_at, (int, float)) or not np.isfinite(observed_at):
            raise ValueError("finite observed_at required")
        task = observation.get("lang")
        if task != self.expected_task:
            raise ValueError("task instruction does not exactly match the cached training prompt")
        image = _decode_image(observation)
        image_tensor = torch.from_numpy(image.copy()).permute(2, 0, 1).unsqueeze(0).to(
            device=self.model.device, dtype=self.model.torch_dtype
        )
        image_tensor = image_tensor * (2.0 / 255.0) - 1.0
        proprio = self.normalizer.normalize_proprio(torch.from_numpy(state).unsqueeze(0))
        mark(trace_id, "preprocess_done")
        start = time.monotonic()
        with torch.no_grad():
            prediction = self.model.infer_action(
                prompt=None,
                input_image=image_tensor,
                action_horizon=MODEL_HORIZON,
                proprio=proprio,
                context=self.context,
                context_mask=self.context_mask,
                num_inference_steps=self.num_inference_steps,
                seed=self.seed,
                rand_device="cpu",
                tiled=False,
                compile_action_infer=False,
                action_infer_mode="first_frame",
            )["action"]
        if tuple(prediction.shape) != (MODEL_HORIZON, MODEL_ACTION_DIM) or not torch.isfinite(prediction).all():
            raise ValueError(f"invalid normalized model output: {tuple(prediction.shape)}")
        normalized_native = prediction.float().cpu().numpy()
        normalized_execution, source_indices = zero_order_hold_20hz_to_50hz(normalized_native)
        native = self.normalizer.denormalize_action(prediction.unsqueeze(0)).float().cpu().numpy()[0]
        native = clip_dex1_targets(native)
        execution, _ = zero_order_hold_20hz_to_50hz(native)
        execution = validate_execution_actions(execution)
        if np.max(np.abs(execution[:, :64])) > 1.25:
            raise ValueError("predicted SONIC token exceeds fail-closed absolute limit 1.25")
        mark(trace_id, "predict_done")
        return {
            "normalized": normalized_execution.tolist(),
            "motion_tokens": execution[:, :64].tolist(),
            "gripper_actions": execution[:, 64:66].tolist(),
            "native_actions": native.tolist(),
            "execution_source_indices": source_indices.tolist(),
            "inference_seconds": time.monotonic() - start,
            "max_abs_token": float(np.max(np.abs(execution[:, :64]))),
        }

    def predict_video(self, observation: dict):
        """Independent diagnostic video generation; never called by action service."""
        torch = self.torch
        state = np.asarray(observation.get("state"), dtype=np.float32)
        if state.shape != (31,) or not np.isfinite(state).all():
            raise ValueError("expected finite raw state[31]")
        if observation.get("lang") != self.expected_task:
            raise ValueError("task instruction does not exactly match cached training prompt")
        image = _decode_image(observation)
        image_tensor = torch.from_numpy(image.copy()).permute(2, 0, 1).unsqueeze(0).to(
            device=self.model.device, dtype=self.model.torch_dtype
        )
        image_tensor = image_tensor * (2.0 / 255.0) - 1.0
        proprio = self.normalizer.normalize_proprio(torch.from_numpy(state).unsqueeze(0))
        start = time.monotonic()
        with torch.no_grad():
            result = self.model.infer_joint(
                prompt=None,
                input_image=image_tensor,
                num_video_frames=9,
                action_horizon=MODEL_HORIZON,
                proprio=proprio,
                context=self.context,
                context_mask=self.context_mask,
                num_inference_steps=self.num_inference_steps,
                seed=self.seed,
                rand_device="cpu",
                tiled=False,
                test_action_with_infer_action=False,
                compile_action_infer=False,
            )
        frames = result.get("video")
        if not isinstance(frames, list) or len(frames) != 9:
            raise ValueError("WB-WAM preview expected 9 decoded video frames")
        return Image.fromarray(image), frames, time.monotonic() - start
