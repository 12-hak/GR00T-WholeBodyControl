"""Convert Meta Quest / WebXR poses into the VR 3-point format used by gear_sonic_deploy."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as sRot

# SMPL wrist/neck frame offsets (same as pico_manager_thread_server)
_WRIST_L_OFFSET = sRot.from_euler("xyz", [90, 0, 0], degrees=True)
_WRIST_R_OFFSET = sRot.from_euler("xyz", [-90, 0, 180], degrees=True)
_NECK_OFFSET = sRot.from_euler("xyz", [0, 0, -90], degrees=True)

# Quest grip pose -> anatomical wrist (Unity local frame, metres)
_GRIP_TO_WRIST_L = np.array([0.0, -0.04, 0.07])
_GRIP_TO_WRIST_R = np.array([0.0, -0.04, 0.07])

_WEBXR_TO_UNITY = np.diag([1.0, 1.0, -1.0])
_Q_UNITY_TO_ROBOT = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0.0]])

JOYSTICK_DEADZONE = 0.08
_PELVIS_BACK_Z = 0.08


def webxr_to_unity_pose7(pose7: np.ndarray) -> np.ndarray:
    pos = _WEBXR_TO_UNITY @ pose7[:3]
    rot = sRot.from_quat(pose7[3:7])
    rot_u = sRot.from_matrix(_WEBXR_TO_UNITY @ rot.as_matrix() @ _WEBXR_TO_UNITY.T)
    out = np.zeros(7, dtype=np.float64)
    out[:3] = pos
    out[3:7] = rot_u.as_quat()
    return out


def _unity_to_robot_pose7(pose7: np.ndarray) -> np.ndarray:
    pose = pose7.copy()
    world = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float64)
    pose[:3] = _Q_UNITY_TO_ROBOT @ pose[:3]
    world[:3] = _Q_UNITY_TO_ROBOT @ world[:3]
    rot_base = sRot.from_quat(world[3:7]).as_matrix()
    rot = sRot.from_quat(pose[3:7]).as_matrix()
    rel_rot = sRot.from_matrix(_Q_UNITY_TO_ROBOT @ (rot_base.T @ rot) @ _Q_UNITY_TO_ROBOT.T)
    rel_pos = sRot.from_matrix(_Q_UNITY_TO_ROBOT @ rot_base.T @ _Q_UNITY_TO_ROBOT.T).apply(
        pose[:3] - world[:3]
    )
    out = np.zeros(7, dtype=np.float32)
    out[:3] = rel_pos
    out[3:7] = rel_rot.as_quat(scalar_first=True)
    return out


def _grip_to_wrist_pose(pose_u: np.ndarray, shift_local: np.ndarray) -> np.ndarray:
    out = pose_u.copy()
    rot = sRot.from_quat(out[3:7])
    out[:3] = out[:3] + rot.apply(shift_local)
    return out


def estimate_torso_yaw_rad(left_u: np.ndarray, right_u: np.ndarray) -> float:
    shoulder = right_u[:3] - left_u[:3]
    shoulder[1] = 0.0
    norm = np.linalg.norm(shoulder)
    if norm < 1e-4:
        return 0.0
    shoulder /= norm
    fwd = np.cross([0.0, 1.0, 0.0], shoulder)
    fn = np.linalg.norm(fwd)
    if fn < 1e-4:
        return 0.0
    fwd /= fn
    return float(np.arctan2(fwd[0], fwd[2]))


def estimate_pelvis_position(
    hmd_u: np.ndarray,
    left_u: np.ndarray,
    right_u: np.ndarray,
    pelvis_height: float = 1.15,
) -> np.ndarray:
    mid = 0.5 * (left_u[:3] + right_u[:3])
    pelvis_y = float(hmd_u[1] - pelvis_height)
    return np.array([mid[0], pelvis_y, mid[2] - _PELVIS_BACK_Z], dtype=np.float64)


def extract_facing_from_controllers(left: np.ndarray, right: np.ndarray) -> list[float]:
    left_u = webxr_to_unity_pose7(np.asarray(left, dtype=np.float64))
    right_u = webxr_to_unity_pose7(np.asarray(right, dtype=np.float64))
    yaw = estimate_torso_yaw_rad(left_u, right_u)
    fwd_u = sRot.from_euler("y", yaw, degrees=False).apply([0.0, 0.0, 1.0])
    fwd_r = _Q_UNITY_TO_ROBOT @ fwd_u
    xy = fwd_r[:2]
    norm = np.linalg.norm(xy)
    if norm < 1e-6:
        return [1.0, 0.0, 0.0]
    return [float(xy[0] / norm), float(xy[1] / norm), 0.0]


def apply_stick_deadzone(x: float, y: float, deadzone: float = JOYSTICK_DEADZONE) -> tuple[float, float]:
    mag = np.hypot(x, y)
    if mag < deadzone:
        return 0.0, 0.0
    scale = (mag - deadzone) / (1.0 - deadzone) / mag
    return float(x * scale), float(y * scale)


class QuestLocomotion:
    """Left stick = move/strafe, right stick X = turn (planner facing)."""

    def __init__(self, yaw_gain: float = 2.5) -> None:
        self.yaw_gain = yaw_gain
        self.yaw_angle_rad = 0.0
        self.heading = [1.0, 0.0, 0.0]
        self.is_turning = False

    def sync_from_controllers(self, left: np.ndarray, right: np.ndarray) -> None:
        facing = extract_facing_from_controllers(left, right)
        self.heading = facing
        self.yaw_angle_rad = float(np.arctan2(facing[1], facing[0]))

    def step(
        self,
        left_stick: tuple[float, float],
        right_stick: tuple[float, float],
        dt: float,
    ) -> tuple[list[float], float, list[float], bool]:
        lx_raw, ly_raw = float(left_stick[0]), float(left_stick[1])
        rx_raw = float(right_stick[0])
        lx, ly = apply_stick_deadzone(lx_raw, ly_raw)
        rx, _ry = apply_stick_deadzone(rx_raw, float(right_stick[1]))

        self.is_turning = abs(rx_raw) >= JOYSTICK_DEADZONE
        if self.is_turning:
            self.yaw_angle_rad += self.yaw_gain * (-rx) * max(dt, 1e-3)

        self.heading = [
            float(np.cos(self.yaw_angle_rad)),
            float(np.sin(self.yaw_angle_rad)),
            0.0,
        ]

        local_x = -lx
        local_y = -ly
        mag = np.hypot(local_x, local_y)
        if mag < 1e-6:
            return [0.0, 0.0, 0.0], -1.0, self.heading, self.is_turning

        scale = min(mag, 1.0) / mag
        local_x *= scale
        local_y *= scale
        fx, fy = self.heading[0], self.heading[1]
        perp_x, perp_y = -fy, fx
        rot = np.array([[perp_x, perp_y], [fx, fy]])
        global_xy = rot @ np.array([local_x, local_y])
        speed = float(min(mag, 1.0))
        return [float(global_xy[0]), float(global_xy[1]), 0.0], speed, self.heading, self.is_turning


def quest_poses_to_vr3pt_raw(
    hmd: np.ndarray,
    left: np.ndarray,
    right: np.ndarray,
    pelvis_height: float = 1.15,
    frozen_torso_yaw: float | None = None,
) -> np.ndarray:
    """
    Quest HMD + controllers -> VR 3PT (L-wrist, R-wrist, neck).

    Arms track controllers in a torso frame frozen at calibration (frozen_torso_yaw).
    Neck uses full HMD orientation for upper-body / head look.
    """
    hmd_u = webxr_to_unity_pose7(np.asarray(hmd, dtype=np.float64))
    left_u = _grip_to_wrist_pose(webxr_to_unity_pose7(np.asarray(left, dtype=np.float64)), _GRIP_TO_WRIST_L)
    right_u = _grip_to_wrist_pose(webxr_to_unity_pose7(np.asarray(right, dtype=np.float64)), _GRIP_TO_WRIST_R)

    live_yaw = estimate_torso_yaw_rad(left_u, right_u)
    torso_yaw = float(frozen_torso_yaw) if frozen_torso_yaw is not None else live_yaw
    pelvis_pos = estimate_pelvis_position(hmd_u, left_u, right_u, pelvis_height)

    root_u = np.zeros(7, dtype=np.float64)
    root_u[:3] = pelvis_pos
    root_u[3:7] = sRot.from_euler("y", torso_yaw, degrees=False).as_quat()

    kp = np.zeros((4, 7), dtype=np.float32)
    for rel_i, pose_u, offset in (
        (0, root_u, sRot.from_euler("xyz", [0, 0, -90], degrees=True)),
        (1, left_u, _WRIST_L_OFFSET),
        (2, right_u, _WRIST_R_OFFSET),
        (3, hmd_u, _NECK_OFFSET),
    ):
        body = _unity_to_robot_pose7(pose_u)
        quat = body[3:7]
        rot_quat = (sRot.from_quat(quat, scalar_first=True) * offset).as_quat(scalar_first=False)
        kp[rel_i, :3] = body[:3]
        kp[rel_i, 3:] = rot_quat

    root_pos = kp[0, :3].copy()
    root_quat = kp[0, 3:].copy()
    root_inv = sRot.from_quat(root_quat, scalar_first=True).inv()
    for i in range(1, 4):
        kp[i, :3] = root_inv.apply(kp[i, :3] - root_pos)
        kp[i, 3:] = (root_inv * sRot.from_quat(kp[i, 3:], scalar_first=True)).as_quat(scalar_first=True)

    return kp[1:]
