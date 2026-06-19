"""Convert Meta Quest / WebXR poses into the VR 3-point format used by gear_sonic_deploy."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as sRot

# SMPL wrist/neck frame offsets (same as pico_manager_thread_server)
_WRIST_L_OFFSET = sRot.from_euler("xyz", [90, 0, 0], degrees=True)
_WRIST_R_OFFSET = sRot.from_euler("xyz", [-90, 0, 180], degrees=True)
_NECK_OFFSET = sRot.from_euler("xyz", [0, 0, -90], degrees=True)

# Quest grip pose -> anatomical wrist (Unity local frame, metres).
_GRIP_TO_WRIST_L = np.array([0.0, -0.04, 0.07])
_GRIP_TO_WRIST_R = np.array([0.0, -0.04, 0.07])

_WEBXR_TO_UNITY = np.diag([1.0, 1.0, -1.0])
_Q_UNITY_TO_ROBOT = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0.0]])

JOYSTICK_DEADZONE = 0.08
_PELVIS_BACK_Z = 0.08

# Axis remap: Quest WebXR chain yields a left-handed wrist frame. Measured on right
# hand: up->-Y, forward->-Z, right->+X. Left hand measured after 1st fix:
# left->+Z, up->+Y (Y/Z swapped) — same remap as right resolves it.
# Target deploy frame: X=fwd, Y=left, Z=up.
QUEST_AXIS_FIX_RIGHT = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64
)
# Left wrist: same remap as right, then mirror across robot Y (left side is +Y).
_QUEST_Y_MIRROR = np.diag([1.0, -1.0, 1.0])
QUEST_AXIS_FIX_NECK = QUEST_AXIS_FIX_RIGHT
DEFAULT_ARM_REACH_SCALE = 0.88


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


def controller_robot_orientation(pose_webxr: np.ndarray) -> sRot:
    """Controller orientation expressed in the robot world frame (scalar-first).

    No root-relative step and no SMPL OFFSET — this is the raw physical orientation
    of the controller in robot coordinates, used for convention-independent
    world-frame rotation deltas.
    """
    u = webxr_to_unity_pose7(np.asarray(pose_webxr, dtype=np.float64))
    body = _unity_to_robot_pose7(u)
    return sRot.from_quat(body[3:7], scalar_first=True)


def hmd_yaw_rad(hmd_u: np.ndarray) -> float:
    """Horizontal yaw of the HMD in the Unity frame (about the up axis)."""
    fwd = sRot.from_quat(hmd_u[3:7]).apply([0.0, 0.0, 1.0])
    fwd[1] = 0.0
    n = np.linalg.norm(fwd)
    if n < 1e-6:
        return 0.0
    fwd /= n
    return float(np.arctan2(fwd[0], fwd[2]))


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
    torso_yaw: float = 0.0,
    pelvis_height: float = 0.65,
) -> np.ndarray:
    """
    Pelvis anchored to the HEAD (HMD), like Pico anchors wrists to the real pelvis.

    XZ tracks the head (with a small lean back along torso forward), Y is the head
    height minus a fixed head->pelvis distance. The absolute offset is absorbed by
    calibration; what matters is the pelvis follows the head, not the hands.
    """
    back = sRot.from_euler("y", torso_yaw, degrees=False).apply([0.0, 0.0, -1.0])
    return np.array(
        [
            float(hmd_u[0] + back[0] * _PELVIS_BACK_Z),
            float(hmd_u[1] - pelvis_height),
            float(hmd_u[2] + back[2] * _PELVIS_BACK_Z),
        ],
        dtype=np.float64,
    )


def extract_facing_from_hmd(hmd: np.ndarray) -> list[float]:
    hmd_u = webxr_to_unity_pose7(np.asarray(hmd, dtype=np.float64))
    yaw = hmd_yaw_rad(hmd_u)
    fwd_u = sRot.from_euler("y", yaw, degrees=False).apply([0.0, 0.0, 1.0])
    fwd_r = _Q_UNITY_TO_ROBOT @ fwd_u
    xy = fwd_r[:2]
    norm = np.linalg.norm(xy)
    if norm < 1e-6:
        return [1.0, 0.0, 0.0]
    return [float(xy[0] / norm), float(xy[1] / norm), 0.0]


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

    def sync_from_hmd(self, hmd: np.ndarray) -> None:
        facing = extract_facing_from_hmd(hmd)
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
    pelvis_height: float = 0.65,
    frozen_torso_yaw: float | None = None,
) -> np.ndarray:
    """
    Quest HMD + controllers -> VR 3PT (L-wrist, R-wrist, neck).

    Pelvis is anchored to the HEAD (HMD), like Pico anchors to the real pelvis —
    so moving the hands does NOT move the body reference frame. Torso yaw is frozen
    at calibration so turning the head does not swing the arms. Neck uses full HMD
    orientation for upper-body / head look.
    """
    hmd_u = webxr_to_unity_pose7(np.asarray(hmd, dtype=np.float64))
    left_u = _grip_to_wrist_pose(webxr_to_unity_pose7(np.asarray(left, dtype=np.float64)), _GRIP_TO_WRIST_L)
    right_u = _grip_to_wrist_pose(webxr_to_unity_pose7(np.asarray(right, dtype=np.float64)), _GRIP_TO_WRIST_R)

    live_yaw = hmd_yaw_rad(hmd_u)
    torso_yaw = float(frozen_torso_yaw) if frozen_torso_yaw is not None else live_yaw
    pelvis_pos = estimate_pelvis_position(hmd_u, torso_yaw, pelvis_height)

    root_u = np.zeros(7, dtype=np.float64)
    root_u[:3] = pelvis_pos
    root_u[3:7] = sRot.from_euler("y", torso_yaw, degrees=False).as_quat()

    # Neck keypoint: use a TORSO-STABLE (yaw-only) orientation, like Pico's SMPL neck
    # joint — NOT the live head tilt. The neck orientation at calibration becomes the
    # frame that ThreePointPose rotates all wrist positions by; using head pitch here
    # corrupts arm tracking (e.g. calibrating while looking down couples vertical hand
    # motion into the robot's X axis). Yaw-only also keeps head turns from moving arms.
    neck_u = hmd_u.copy()
    neck_u[3:7] = sRot.from_euler("y", torso_yaw, degrees=False).as_quat()

    kp = np.zeros((4, 7), dtype=np.float32)
    for rel_i, pose_u, offset in (
        (0, root_u, sRot.from_euler("xyz", [0, 0, -90], degrees=True)),
        (1, left_u, _WRIST_L_OFFSET),
        (2, right_u, _WRIST_R_OFFSET),
        (3, neck_u, _NECK_OFFSET),
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


def _apply_axis_fix_row(row: np.ndarray, P: np.ndarray) -> None:
    row[:3] = P @ row[:3]
    rm = sRot.from_quat(row[3:7], scalar_first=True).as_matrix()
    row[3:7] = sRot.from_matrix(P @ rm @ P.T).as_quat(scalar_first=True)


def _mirror_robot_y_row(row: np.ndarray) -> None:
    """Mirror wrist pose across robot Y=0 (right-side remap -> left-side +Y)."""
    row[1] = -row[1]
    rm = sRot.from_quat(row[3:7], scalar_first=True).as_matrix()
    row[3:7] = sRot.from_matrix(_QUEST_Y_MIRROR @ rm @ _QUEST_Y_MIRROR).as_quat(
        scalar_first=True
    )


def apply_quest_vr3pt_axis_fix(
    vr_3pt: np.ndarray,
    reach_scale: float = 1.0,
    reach_origin: np.ndarray | None = None,
) -> np.ndarray:
    """Remap Quest wrist frames to deploy robot frame; optional reach scaling."""
    out = vr_3pt.copy()
    _apply_axis_fix_row(out[0], QUEST_AXIS_FIX_RIGHT)
    _mirror_robot_y_row(out[0])
    _apply_axis_fix_row(out[1], QUEST_AXIS_FIX_RIGHT)
    _apply_axis_fix_row(out[2], QUEST_AXIS_FIX_NECK)
    if reach_scale != 1.0 and reach_origin is not None:
        for i in (0, 1):
            out[i, :3] = reach_origin[i] + reach_scale * (out[i, :3] - reach_origin[i])
    return out
