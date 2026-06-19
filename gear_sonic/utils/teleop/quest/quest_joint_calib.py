"""Interactive Quest controller / HMD joint alignment calibration."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from scipy.spatial.transform import Rotation as sRot

# Robot pelvis frame: X forward, Y left, Z up
_LEFT_DIRS = {
    "out": np.array([0.0, 1.0, 0.0]),
    "fwd": np.array([1.0, 0.0, 0.0]),
    "up": np.array([0.0, 0.0, 1.0]),
    "back": np.array([-1.0, 0.0, 0.0]),
    "down": np.array([0.0, 0.0, -1.0]),
}
_RIGHT_DIRS = {
    "out": np.array([0.0, -1.0, 0.0]),
    "fwd": np.array([1.0, 0.0, 0.0]),
    "up": np.array([0.0, 0.0, 1.0]),
    "back": np.array([-1.0, 0.0, 0.0]),
    "down": np.array([0.0, 0.0, -1.0]),
}

# Quest grip: ray / point direction in grip local frame
_GRIP_POINT_AXIS = np.array([0.0, 0.0, -1.0])


@dataclass
class QuestJointOffsets:
    left_rot_fix: sRot = field(default_factory=lambda: sRot.identity())
    right_rot_fix: sRot = field(default_factory=lambda: sRot.identity())
    left_pos_fix: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    right_pos_fix: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    neck_rot_fix: sRot = field(default_factory=lambda: sRot.identity())
    neck_pitch_gain: float = 1.8
    neck_roll_gain: float = 1.2
    tpose_neck_quat_wxyz: np.ndarray | None = None
    active: bool = False

    def apply(self, vr_3pt: np.ndarray) -> np.ndarray:
        if not self.active:
            return vr_3pt
        out = vr_3pt.copy()
        for row, rot_fix, pos_fix in (
            (0, self.left_rot_fix, self.left_pos_fix),
            (1, self.right_rot_fix, self.right_pos_fix),
        ):
            r = sRot.from_quat(out[row, 3:7], scalar_first=True)
            out[row, 3:7] = (rot_fix * r).as_quat(scalar_first=True)
            out[row, :3] = out[row, :3] + pos_fix

        nr = sRot.from_quat(out[2, 3:7], scalar_first=True)
        out[2, 3:7] = (self.neck_rot_fix * nr).as_quat(scalar_first=True)

        if self.tpose_neck_quat_wxyz is not None:
            ref = sRot.from_quat(self.tpose_neck_quat_wxyz, scalar_first=True)
            cur = sRot.from_quat(out[2, 3:7], scalar_first=True)
            rel = ref.inv() * cur
            e = rel.as_euler("YXZ", degrees=False)
            e[1] *= self.neck_pitch_gain
            e[2] *= self.neck_roll_gain
            out[2, 3:7] = (ref * sRot.from_euler("YXZ", e, degrees=False)).as_quat(scalar_first=True)
        return out


@dataclass
class _CalibStepDef:
    key: str
    prompt: str
    limb: str  # left | right | neck | both


_CALIB_STEPS: list[_CalibStepDef] = [
    _CalibStepDef("tpose", "T-pose: BOTH arms straight OUT horizontally. Look straight ahead.", "both"),
    _CalibStepDef("left_out", "LEFT arm straight OUT to your LEFT side.", "left"),
    _CalibStepDef("left_fwd", "LEFT arm straight FORWARD (point ahead).", "left"),
    _CalibStepDef("left_up", "LEFT arm straight UP toward ceiling.", "left"),
    _CalibStepDef("left_back", "LEFT arm pointing BACK behind you.", "left"),
    _CalibStepDef("left_down", "LEFT arm straight DOWN at your side.", "left"),
    _CalibStepDef("right_out", "RIGHT arm straight OUT to your RIGHT side.", "right"),
    _CalibStepDef("right_fwd", "RIGHT arm straight FORWARD.", "right"),
    _CalibStepDef("right_up", "RIGHT arm straight UP.", "right"),
    _CalibStepDef("right_back", "RIGHT arm pointing BACK.", "right"),
    _CalibStepDef("right_down", "RIGHT arm straight DOWN.", "right"),
    _CalibStepDef("head_fwd", "Look STRAIGHT ahead (neutral head).", "neck"),
    _CalibStepDef("head_down", "Look DOWN at your feet.", "neck"),
    _CalibStepDef("head_up", "Look UP at the ceiling.", "head_up"),
]


class QuestJointCalibWizard:
    COUNTDOWN_SEC = 3.0
    CAPTURE_SEC = 0.6

    def __init__(self) -> None:
        self.offsets = QuestJointOffsets()
        self._step_idx = 0
        self._phase = "idle"
        self._countdown_at = 0.0
        self._capture_at = 0.0
        self._capture_buf: list[np.ndarray] = []
        self._left_samples: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._right_samples: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._neck_samples: dict[str, np.ndarray] = {}
        self._announced = -1

    @property
    def running(self) -> bool:
        return self._phase != "idle"

    def start(self) -> None:
        self._step_idx = 0
        self._phase = "countdown"
        self._countdown_at = 0.0
        self._capture_buf.clear()
        self._left_samples.clear()
        self._right_samples.clear()
        self._neck_samples.clear()
        self._announced = -1
        self.offsets = QuestJointOffsets()
        print("\n[Quest] === Joint calibration (j) ===")
        self._print_step_prompt()

    def _print_step_prompt(self) -> None:
        step = _CALIB_STEPS[self._step_idx]
        print(f"[Quest] Step {self._step_idx + 1}/{len(_CALIB_STEPS)}: {step.prompt}")

    def _avg_pose(self, buf: list[np.ndarray]) -> np.ndarray:
        arr = np.stack(buf, axis=0)
        out = np.mean(arr, axis=0)
        quats = arr[:, 3:7]
        quats /= np.linalg.norm(quats, axis=1, keepdims=True)
        mean_q = np.mean(quats, axis=0)
        mean_q /= np.linalg.norm(mean_q)
        out[3:7] = mean_q
        return out.astype(np.float32)

    def _finish_capture(self, avg: np.ndarray) -> None:
        step = _CALIB_STEPS[self._step_idx]
        if step.limb in ("left", "both"):
            self._left_samples[step.key] = (avg[0, :3].copy(), avg[0, 3:7].copy())
        if step.limb in ("right", "both"):
            self._right_samples[step.key] = (avg[1, :3].copy(), avg[1, 3:7].copy())
        if step.limb in ("neck", "both", "head_up"):
            self._neck_samples[step.key] = avg[2, 3:7].copy()
        print(f"[Quest] Captured step '{step.key}'")

    def _fit_wrist(self, samples: dict, prefix: str) -> sRot:
        dirs = _LEFT_DIRS if prefix == "left" else _RIGHT_DIRS
        sources: list[np.ndarray] = []
        targets: list[np.ndarray] = []
        for pose_name, target in dirs.items():
            key = "tpose" if pose_name == "out" and "tpose" in samples else f"{prefix}_{pose_name}"
            if key not in samples:
                continue
            _pos, quat_wxyz = samples[key]
            rot = sRot.from_quat(quat_wxyz, scalar_first=True)
            sources.append(rot.apply(_GRIP_POINT_AXIS))
            targets.append(target / np.linalg.norm(target))
        if len(sources) < 2:
            return sRot.identity()
        src = np.stack(sources)
        tgt = np.stack(targets)
        rot, _ = sRot.align_vectors(tgt, src)
        return rot

    def _finalize(self) -> None:
        self.offsets.left_rot_fix = self._fit_wrist(self._left_samples, "left")
        self.offsets.right_rot_fix = self._fit_wrist(self._right_samples, "right")

        if "tpose" in self._neck_samples:
            self.offsets.tpose_neck_quat_wxyz = self._neck_samples["tpose"].copy()
        if "head_fwd" in self._neck_samples:
            nf = sRot.from_quat(self._neck_samples["head_fwd"], scalar_first=True)
            nt = (
                sRot.from_quat(self._neck_samples["tpose"], scalar_first=True)
                if "tpose" in self._neck_samples
                else sRot.identity()
            )
            self.offsets.neck_rot_fix = nt * nf.inv()

        if "head_down" in self._neck_samples and "head_fwd" in self._neck_samples:
            ref = sRot.from_quat(self._neck_samples["head_fwd"], scalar_first=True)
            down = sRot.from_quat(self._neck_samples["head_down"], scalar_first=True)
            rel = ref.inv() * down
            pitch = abs(rel.as_euler("YXZ", degrees=False)[1])
            if pitch > 0.05:
                self.offsets.neck_pitch_gain = min(3.0, np.radians(45) / pitch)

        self.offsets.active = True
        print(
            f"[Quest] Joint calibration DONE — pitch gain={self.offsets.neck_pitch_gain:.2f}, "
            f"roll gain={self.offsets.neck_roll_gain:.2f}. Press v for arms."
        )

    def tick(self, now: float, raw_vr_3pt: np.ndarray | None) -> None:
        if self._phase == "idle":
            return
        if raw_vr_3pt is None:
            return

        if self._phase == "countdown":
            if self._countdown_at <= 0.0:
                self._countdown_at = now + self.COUNTDOWN_SEC
            remaining = int(np.ceil(self._countdown_at - now))
            if remaining != self._announced:
                self._announced = remaining
                if remaining > 0:
                    print(f"[Quest] Hold pose… {remaining}")
            if now >= self._countdown_at:
                self._phase = "capture"
                self._capture_at = now + self.CAPTURE_SEC
                self._capture_buf.clear()
                self._announced = -1
            return

        if self._phase == "capture":
            self._capture_buf.append(raw_vr_3pt.copy())
            if now >= self._capture_at:
                if self._capture_buf:
                    self._finish_capture(self._avg_pose(self._capture_buf))
                self._step_idx += 1
                if self._step_idx >= len(_CALIB_STEPS):
                    self._phase = "idle"
                    self._finalize()
                else:
                    self._phase = "countdown"
                    self._countdown_at = 0.0
                    self._print_step_prompt()
            return


def compute_waist_from_neck_quat(
    neck_quat_wxyz: np.ndarray, scale: float = 1.0
) -> np.ndarray:
    """Map calibrated neck quat -> G1 waist [yaw, roll, pitch]."""
    quat_xyzw = np.array(
        [neck_quat_wxyz[1], neck_quat_wxyz[2], neck_quat_wxyz[3], neck_quat_wxyz[0]]
    )
    euler_zyx = sRot.from_quat(quat_xyzw).as_euler("ZYX", degrees=False)
    return np.array(
        [euler_zyx[0] * scale, euler_zyx[2] * scale, euler_zyx[1] * scale],
        dtype=np.float64,
    )


def build_upper_body_with_waist(
    base_upper_body: list[float] | np.ndarray | None,
    neck_quat_wxyz: np.ndarray,
    pitch_gain: float = 1.0,
) -> list[float]:
    """17-DOF upper body target with waist driven from neck orientation."""
    ub = list(base_upper_body) if base_upper_body is not None else [0.0] * 17
    if len(ub) < 17:
        ub = ub + [0.0] * (17 - len(ub))
    waist = compute_waist_from_neck_quat(neck_quat_wxyz, scale=pitch_gain)
    ub[0] = float(waist[0])
    ub[1] = float(waist[1])
    ub[2] = float(waist[2])
    return ub
