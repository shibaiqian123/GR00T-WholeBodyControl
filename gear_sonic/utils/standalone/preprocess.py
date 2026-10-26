"""Extracted deployment preprocessing; see SOURCES.md for versions and licenses."""
from __future__ import annotations
import torch
from .math_ops import *
IL2MJ = [
    0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18,
    2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28,
]
MJ2IL = [
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17,
    24, 18, 25, 19, 26, 20, 27, 21, 28,
]
SMPL_FUTURE_STEPS = 10
SMPL_FRAME_SKIP = 1  # 0.02 s @ 50 fps (sonic_release smpl_dt_future_ref_frames)
SMPL_WRIST_MJ_IDX = tuple(IL2MJ.index(i) for i in range(23, 29))
def sonic_smpl_tokenizer(
    env: ManagerBasedRlEnv,
    command_name: str = "motion",
    future_steps: int = SMPL_FUTURE_STEPS,
    frame_skip: int = SMPL_FRAME_SKIP,
) -> torch.Tensor:
    """SONIC smpl-encoder input: (num_envs, future_steps * 84).

    Per-frame layout mirrors sonic_release.yaml's smpl encoder inputs, in order:
      [smpl_joints_multi_future_local_nonflat (72) |
       smpl_root_ori_b_multi_future (6) |
       joint_pos_multi_future_wrist_for_smpl (6)]

    Op-chain verbatim from gear_sonic (commands.py smpl properties +
    observations.py smpl terms): joints rotated into each future frame's OWN
    smpl-root frame; root ori = 6D of quat_inv(robot_root) * smpl_root; wrist
    refs = the retargeted G1 wrist joint targets (zeros when unavailable).

    Requires ``command.motion`` to carry ``smpl_joints (T, 24, 3)`` RAW as in
    SONIC's smpl pkl (**z-up**, root-centered, no transl — the motion lib
    never converts joints, only the root quat) and ``smpl_root_quat (T, 4)``
    z-up, wxyz, SMPL base rot removed.

    **Both z-up, and that is load-bearing.** The op-chain below is
    ``quat_apply_inverse(root_q, joints)``, so a frame mismatch does not error —
    it silently redefines the leading 72 dims as a body frame that rotates with
    heading. gear_sonic's ``smpl_y_up`` flag converts the ROOT ONLY
    (``motion_lib_base.py`` loads ``smpl_joints`` untouched), so a pkl with a
    y-up ``pose_aa`` still ships z-up joints. Zero-shot tracking reward on 63
    GRAIL curb clips: z-up **4.735** vs y-up **0.348** (g1 encoder 5.677).
    """
    command = env.command_manager.get_term(command_name)
    num_envs = env.num_envs
    idx = command.future_frames(future_steps, frame_skip)  # (B, F)

    joints = command.motion.smpl_joints[idx]      # (B, F, 24, 3)
    root_q = command.motion.smpl_root_quat[idx]   # (B, F, 4)

    q = root_q[:, :, None, :].expand(*joints.shape[:-1], 4)
    joints_local = quat_apply_inverse(
        q.reshape(-1, 4), joints.reshape(-1, 3)
    ).reshape(num_envs, future_steps, -1)          # (B, F, 72)

    robot_quat = env.scene["robot"].data.root_link_quat_w
    rot_dif = quat_mul(
        quat_inv(robot_quat[:, None, :].expand(num_envs, future_steps, 4).reshape(-1, 4)),
        root_q.reshape(-1, 4),
    )
    ori = matrix_from_quat(rot_dif)[..., :2].reshape(num_envs, future_steps, 6)

    wrist = command.motion.joint_pos[idx][:, :, SMPL_WRIST_MJ_IDX]  # (B, F, 6)

    return torch.cat([joints_local, ori, wrist], dim=-1).reshape(num_envs, -1)
__all__ = [
    "G1_SMPL_BODY_MAP",
    "infer_morphology_scale",
    "scaled_smpl_targets",
]
G1_SMPL_BODY_MAP: tuple[tuple[str, int], ...] = (
    ("pelvis", 0),
    ("left_hip_roll_link", 1),
    ("left_knee_link", 4),
    ("left_ankle_roll_link", 10),
    ("right_hip_roll_link", 2),
    ("right_knee_link", 5),
    ("right_ankle_roll_link", 11),
    ("torso_link", 9),
    ("left_shoulder_roll_link", 16),
    ("left_elbow_link", 18),
    ("left_wrist_yaw_link", 20),
    ("right_shoulder_roll_link", 17),
    ("right_elbow_link", 19),
    ("right_wrist_yaw_link", 21),
)
_LEFT_LEG = (0, 1, 4, 10)
_RIGHT_LEG = (0, 2, 5, 11)
def _chain_length(points: torch.Tensor, indices: tuple[int, ...]) -> torch.Tensor:
    chain = points[..., list(indices), :]
    return torch.linalg.vector_norm(
        chain[..., 1:, :] - chain[..., :-1, :], dim=-1
    ).sum(-1)
def infer_morphology_scale(
    smpl_joints_w: torch.Tensor,
    robot_body_pos_w: torch.Tensor,
    robot_body_names: tuple[str, ...] | list[str],
) -> float:
    """Infer one G1/human scale from median left/right leg-chain lengths."""
    name_to_idx = {name: i for i, name in enumerate(robot_body_names)}
    robot_chains = (
        ("pelvis", "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link"),
        ("pelvis", "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link"),
    )
    robot_lengths = []
    for names in robot_chains:
        p = robot_body_pos_w[..., [name_to_idx[name] for name in names], :]
        robot_lengths.append(
            torch.linalg.vector_norm(p[..., 1:, :] - p[..., :-1, :], dim=-1).sum(-1)
        )
    robot_length = torch.stack(robot_lengths).mean()

    human_lengths = torch.stack(
        (
            _chain_length(smpl_joints_w, _LEFT_LEG),
            _chain_length(smpl_joints_w, _RIGHT_LEG),
        ),
        dim=-1,
    ).mean(-1)
    human_length = human_lengths.median()
    if not torch.isfinite(human_length) or human_length <= 1e-6:
        raise ValueError("cannot infer morphology scale from a degenerate SMPL skeleton")
    return float((robot_length / human_length).item())
def scaled_smpl_targets(
    smpl_joints_w: torch.Tensor,
    scale: float | torch.Tensor,
    origins: torch.Tensor,
) -> torch.Tensor:
    """Map every homologous SMPL point to a G1-sized world target.

    Root xy and the lowest source ankle z remain in the authored scene frame;
    body-relative offsets are uniformly scaled around the pelvis.  ``scale``
    may be one scalar or one value per leading batch element.
    """
    pelvis = smpl_joints_w[..., 0, :]
    foot_z = smpl_joints_w[..., (10, 11), 2].amin(dim=-1)
    scale_t = torch.as_tensor(
        scale, dtype=smpl_joints_w.dtype, device=smpl_joints_w.device
    )
    while scale_t.ndim < pelvis.ndim - 1:
        scale_t = scale_t.unsqueeze(-1)

    root = pelvis.clone()
    root[..., 2] = foot_z + scale_t * (pelvis[..., 2] - foot_z)
    mapped = smpl_joints_w[..., [j for _, j in G1_SMPL_BODY_MAP], :]
    targets = root[..., None, :] + scale_t[..., None, None] * (
        mapped - pelvis[..., None, :]
    )
    return targets + origins[..., None, :]
