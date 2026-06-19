#!/usr/bin/env python3
"""
Quest 3 arm teleop bridge for GR00T sim2sim.

Streams Meta Quest HMD + controller poses over WebSocket, converts to VR 3-point
targets, and publishes ZMQ planner messages (same wire format as Pico).

Arms: VR_3PT mode (encoder mode teleop). Legs: planner idle (no locomotion yet).

Usage (from repo root, .venv_teleop active):
  python gear_sonic/scripts/quest3_manager_server.py

Quest browser (same Wi-Fi as PC, HTTPS required for WebXR):
  https://<PC_IP>:8766/webxr_client.html?host=<PC_IP>
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import ssl
import sys
import threading
import time
from enum import Enum
from pathlib import Path

import msgpack
import numpy as np
import zmq
from scipy.spatial.transform import Rotation as sRot

from gear_sonic.utils.teleop.quest.quest_joint_calib import QuestJointCalibWizard, QuestJointOffsets
from gear_sonic.utils.teleop.quest.quest_pose import (
    DEFAULT_ARM_REACH_SCALE,
    QuestLocomotion,
    apply_quest_vr3pt_axis_fix,
    hmd_yaw_rad,
    quest_poses_to_vr3pt_raw,
    webxr_to_unity_pose7,
)
from gear_sonic.utils.teleop.quest.quest_tls import ensure_tls_cert, make_server_ssl_context
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
)
from gear_sonic.utils.teleop.zmq.zmq_poller import ZMQPoller

try:
    from gear_sonic.scripts.pico_manager_thread_server import (
        FeedbackReader,
        LocomotionMode,
        ThreePointPose,
    )
except ImportError as exc:
    raise SystemExit(
        "Could not import pico_manager_thread_server. "
        "Install teleop deps: pip install -e 'gear_sonic[teleop]'"
    ) from exc


class StreamMode(Enum):
    OFF = 0
    PLANNER = 2
    PLANNER_VR_3PT = 5


class QuestFrame:
    __slots__ = ("hmd", "left", "right", "buttons", "thumbstick", "t")

    def __init__(self, data: dict):
        self.hmd = np.array(data["hmd"], dtype=np.float64)
        self.left = np.array(data["left"], dtype=np.float64)
        self.right = np.array(data["right"], dtype=np.float64)
        self.buttons = data.get("buttons", {})
        self.thumbstick = data.get("thumbstick", {})
        self.t = float(data.get("t", time.time()))


class QuestPoseStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: QuestFrame | None = None
        self._frame_count = 0
        self._ping_count = 0
        self._last_frame_time = 0.0
        self._last_ping_time = 0.0
        self._last_in_vr = False
        self._last_dbg: dict | None = None

    def update(self, data: dict) -> None:
        with self._lock:
            if data.get("dbg"):
                self._last_dbg = data["dbg"]
                if data["dbg"].get("in_vr"):
                    self._last_in_vr = True
            if data.get("ping"):
                self._ping_count += 1
                self._last_ping_time = time.time()
                self._last_in_vr = bool(data.get("in_vr"))
                return
            if "hmd" not in data or "left" not in data or "right" not in data:
                return
            self._latest = QuestFrame(data)
            self._frame_count += 1
            self._last_frame_time = time.time()

    def get(self) -> QuestFrame | None:
        with self._lock:
            return self._latest

    def stats(self) -> tuple[int, int, float, float, bool, dict | None]:
        with self._lock:
            return (
                self._frame_count,
                self._ping_count,
                self._last_frame_time,
                self._last_ping_time,
                self._last_in_vr,
                self._last_dbg,
            )


WEB_DIR = Path(__file__).resolve().parents[1] / "utils" / "teleop" / "quest"
_MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript",
    ".css": "text/css",
}
_WSS_PROBE_HTML = (
    "<!DOCTYPE html><html><body><h1>GR00T Quest WSS</h1>"
    "<p>Certificate trusted. Close this tab and open the WebXR page on port 8766.</p>"
    "</body></html>"
)


def _guess_mime(path: str) -> str:
    for ext, mime in _MIME.items():
        if path.endswith(ext):
            return mime
    return "text/plain; charset=utf-8"


def _make_static_process_request(web_dir: Path):
    root = web_dir.resolve()

    async def process_request(connection, request):
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None

        path = request.path.split("?", 1)[0]
        if path in ("", "/"):
            path = "/webxr_client.html"

        rel = path.lstrip("/")
        if not rel or ".." in rel.split("/"):
            response = connection.respond(404, "Not found\n")
            response.headers["Content-Type"] = "text/plain; charset=utf-8"
            return response

        file_path = (root / rel).resolve()
        if not str(file_path).startswith(str(root)) or not file_path.is_file():
            response = connection.respond(404, "Not found\n")
            response.headers["Content-Type"] = "text/plain; charset=utf-8"
            return response

        body = file_path.read_text(encoding="utf-8")
        response = connection.respond(200, body)
        response.headers["Content-Type"] = _guess_mime(rel)
        return response

    return process_request


async def _ws_probe_process_request(connection, request):
    if request.headers.get("Upgrade", "").lower() == "websocket":
        return None
    response = connection.respond(200, _WSS_PROBE_HTML)
    response.headers["Content-Type"] = "text/html; charset=utf-8"
    return response


async def _ws_handler(websocket, store: QuestPoseStore) -> None:
    peer = websocket.remote_address
    print(f"[Quest] WebSocket client connected: {peer}")
    got_pose = False
    try:
        async for message in websocket:
            data = json.loads(message)
            store.update(data)
            if not data.get("ping") and "hmd" in data and not got_pose:
                got_pose = True
                print(f"[Quest] First VR pose frame from {peer}")
    except Exception as exc:
        print(f"[Quest] WebSocket client error: {exc}")
    finally:
        print(f"[Quest] WebSocket client disconnected: {peer}")


async def _run_https_ws_server(
    port: int, store: QuestPoseStore, ssl_context: ssl.SSLContext
) -> None:
    import websockets

    async def handler(websocket):
        await _ws_handler(websocket, store)

    async with websockets.serve(
        handler,
        "0.0.0.0",
        port,
        ssl=ssl_context,
        process_request=_make_static_process_request(WEB_DIR),
    ):
        print(f"[Quest] HTTPS + WSS on port {port}")
        print(f"[Quest] Open: https://<PC_IP>:{port}/webxr_client.html?host=<PC_IP>")
        await asyncio.Future()


async def _run_ws_only_server(
    port: int, store: QuestPoseStore, ssl_context: ssl.SSLContext
) -> None:
    import websockets

    async def handler(websocket):
        await _ws_handler(websocket, store)

    async with websockets.serve(
        handler,
        "0.0.0.0",
        port,
        ssl=ssl_context,
        process_request=_ws_probe_process_request,
    ):
        print(f"[Quest] Legacy WSS on port {port} (HTTP GET returns cert-trust page)")
        await asyncio.Future()


def _start_network_servers(
    http_port: int, ws_port: int, store: QuestPoseStore, ssl_context: ssl.SSLContext
) -> None:
    if http_port == ws_port:
        target = lambda: asyncio.run(_run_https_ws_server(http_port, store, ssl_context))
        threading.Thread(target=target, daemon=True).start()
        return

    threading.Thread(
        target=lambda: asyncio.run(_run_https_ws_server(http_port, store, ssl_context)),
        daemon=True,
    ).start()
    threading.Thread(
        target=lambda: asyncio.run(_run_ws_only_server(ws_port, store, ssl_context)),
        daemon=True,
    ).start()


def _keyboard_listener(state: dict) -> None:
    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while state["running"]:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1).lower()
                if ch == "s":
                    state["cmd"] = "start"
                elif ch == "v":
                    state["cmd"] = "vr3pt"
                elif ch == "c":
                    state["cmd"] = "calibrate"
                elif ch == "o":
                    state["cmd"] = "stop"
                elif ch == "p":
                    state["cmd"] = "planner"
                elif ch == "j":
                    state["cmd"] = "joint_calib"
                elif ch in (" ", "\r", "\n"):
                    state["cmd"] = "calib_capture"
                elif ch == "k":
                    state["cmd"] = "calib_abort"
                elif ch == "d":
                    state["cmd"] = "debug_toggle"
                elif ch == "f":
                    state["cmd"] = "freeze_orient_toggle"
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    parser = argparse.ArgumentParser(description="Quest 3 arm teleop ZMQ bridge")
    parser.add_argument("--zmq-port", type=int, default=5556)
    parser.add_argument("--ws-port", type=int, default=8766)
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--zmq-feedback-port", type=int, default=5557)
    parser.add_argument("--rate", type=int, default=20, help="Planner send rate Hz")
    parser.add_argument("--host-ip", type=str, default=os.environ.get("QUEST_HOST_IP", "192.168.1.235"))
    parser.add_argument("--cert-dir", type=str, default=os.path.expanduser("~/.gear_sonic_quest_certs"))
    parser.add_argument("--vis", action="store_true", help="VR 3-point PyVista debug view")
    parser.add_argument(
        "--arm-scale",
        type=float,
        default=DEFAULT_ARM_REACH_SCALE,
        help="Scale wrist reach from calibration pose (default: %(default)s)",
    )
    args = parser.parse_args()

    cert_path, key_path = ensure_tls_cert(Path(args.cert_dir), args.host_ip)
    ssl_context = make_server_ssl_context(cert_path, key_path)

    store = QuestPoseStore()
    three_point = ThreePointPose(enable_vis_vr3pt=args.vis, log_prefix="Quest3PT")
    locomotion = QuestLocomotion()
    joint_wizard = QuestJointCalibWizard()
    frozen_torso_yaw: float | None = None

    def _freeze_torso(frame: QuestFrame) -> None:
        nonlocal frozen_torso_yaw
        hmd_u = webxr_to_unity_pose7(frame.hmd)
        frozen_torso_yaw = hmd_yaw_rad(hmd_u)
        locomotion.sync_from_hmd(frame.hmd)
        print(f"[Quest] Torso frame frozen (yaw={np.degrees(frozen_torso_yaw):.1f}°)")

    def _raw_pose(frame: QuestFrame) -> np.ndarray:
        return quest_poses_to_vr3pt_raw(
            frame.hmd, frame.left, frame.right, frozen_torso_yaw=frozen_torso_yaw
        )

    def _recapture_wrist_ref() -> None:
        """Re-learn wrist position/orientation reference without FK recalibration."""
        keep_freeze = wrist_ref.get("freeze", True)
        wrist_ref.clear()
        wrist_ref.update(active=False, capture=True, freeze=keep_freeze)

    def _enable_arm_tracking() -> None:
        nonlocal mode
        mode = StreamMode.PLANNER_VR_3PT
        socket.send(build_command_message(start=True, stop=False, planner=True))
        _recapture_wrist_ref()

    def _recalibrate_quest(frame: QuestFrame, *, start_tracking: bool = True) -> bool:
        """Full reset: torso frame, FK offsets (default robot pose), wrist tracking."""
        _freeze_torso(frame)
        raw = _raw_pose(frame)
        if not three_point.calibrate_from_vr3pt(raw, full_reset=True):
            print("[Quest] Calibration FAILED — check robot model / URDF loaded")
            return False
        joint_wizard.offsets = QuestJointOffsets()
        if joint_wizard.running:
            joint_wizard.abort()
        if start_tracking:
            _enable_arm_tracking()
            print("[Quest] Calibrated — arms tracking (forearms-forward neutral)")
        else:
            _recapture_wrist_ref()
            print("[Quest] Calibrated to default arm pose")
        return True

    # Position-first wrist tracking; press f to toggle live wrist rotation.
    wrist_ref: dict = {"active": False, "capture": False, "freeze": True}
    feedback = FeedbackReader(zmq_feedback_port=args.zmq_feedback_port)

    ctx = zmq.Context()
    socket = ctx.socket(zmq.PUB)
    try:
        socket.bind(f"tcp://*:{args.zmq_port}")
    except zmq.error.ZMQError as exc:
        if exc.errno == zmq.EADDRINUSE:
            raise SystemExit(
                f"[Quest] ZMQ port {args.zmq_port} already in use — another teleop instance is running.\n"
                f"  Kill it: wsl -d Ubuntu-22.04 -- bash -lc 'fuser -k {args.zmq_port}/tcp'\n"
                f"  Or check: wsl -d Ubuntu-22.04 -- bash -lc 'ss -tlnp | grep {args.zmq_port}'"
            ) from exc
        raise

    _start_network_servers(args.http_port, args.ws_port, store, ssl_context)

    kb_state = {"running": True, "cmd": None}
    threading.Thread(target=_keyboard_listener, args=(kb_state,), daemon=True).start()

    print(
        "\n[Quest] Keyboard: c=calibrate+track arms | s=start | v=arms | p=idle | o=stop\n"
        "[Quest] Calibrate (c): forearms forward, elbows bent ~90°, look straight ahead.\n"
        "[Quest] f=toggle wrist rotation (default: position-only). d=axis debug.\n"
        "[Quest] Sticks: left=move/strafe, right X=turn.\n"
        "[Quest] Deploy must use: ./deploy.sh --input-type zmq_manager sim\n"
    )

    mode = StreamMode.OFF
    dt = 1.0 / max(1, args.rate)
    prev_stick_click = False
    prev_stop = False
    warned_no_data = False
    calibrate_at: float | None = None
    calibrate_announced = 0
    last_feedback_poll = 0.0
    debug_axes = False
    last_debug = 0.0

    try:
        while True:
            frame = store.get()
            cmd = kb_state.pop("cmd", None)

            now = time.time()

            if cmd == "calibrate":
                calibrate_at = now + 3.0
                calibrate_announced = 0
                print("[Quest] Calibration in 3s — forearms forward, elbows bent, look ahead")
            elif cmd == "start":
                if frame is not None:
                    _recalibrate_quest(frame, start_tracking=False)
                else:
                    print("[Quest] No headset data yet — starting policy without VR calibration")
                mode = StreamMode.PLANNER
                socket.send(build_command_message(start=True, stop=False, planner=True))
                print("[Quest] Policy started (planner idle) — check deploy terminal / MuJoCo")
            elif cmd == "vr3pt":
                if frame is None:
                    print(
                        "[Quest] Arm VR_3PT requested but no Quest data — "
                        "fix WebSocket on headset first, then press v again"
                    )
                elif not three_point.is_calibrated:
                    if frame is not None:
                        _recalibrate_quest(frame, start_tracking=True)
                    else:
                        print("[Quest] Press c to calibrate first")
                else:
                    _enable_arm_tracking()
                    print("[Quest] Arm VR_3PT mode active")
            elif cmd == "planner":
                mode = StreamMode.PLANNER
                socket.send(build_command_message(start=True, stop=False, planner=True))
                print("[Quest] Planner idle (arms not tracked)")
            elif cmd == "joint_calib":
                if frame is None:
                    print("[Quest] Joint calib needs Quest tracking — enter VR first")
                else:
                    _freeze_torso(frame)
                    joint_wizard.start()
            elif cmd == "calib_capture":
                if joint_wizard.running:
                    joint_wizard.request_capture()
            elif cmd == "calib_abort":
                joint_wizard.abort()
            elif cmd == "debug_toggle":
                debug_axes = not debug_axes
                wrist_ref["dbg_ref_L"] = None
                wrist_ref["dbg_ref_R"] = None
                print(
                    f"[Quest] Axis debug {'ON' if debug_axes else 'OFF'} — "
                    "hold still, then move ONE axis; read 'dominant'"
                )
            elif cmd == "freeze_orient_toggle":
                wrist_ref["freeze"] = not wrist_ref.get("freeze", False)
                print(
                    f"[Quest] Wrist orientation {'FROZEN (position-only)' if wrist_ref.get('freeze') else 'LIVE'}"
                )
            elif cmd == "stop":
                mode = StreamMode.OFF
                socket.send(build_command_message(start=False, stop=True, planner=True))
                print("[Quest] Stop sent")

            if calibrate_at is not None:
                remaining = int(np.ceil(calibrate_at - now))
                if remaining > 0:
                    if remaining != calibrate_announced:
                        calibrate_announced = remaining
                        print(f"[Quest] Calibrating in {remaining}...")
                elif frame is not None:
                    if _recalibrate_quest(frame, start_tracking=True):
                        print("[Quest] Ready — move arms")
                    calibrate_at = None
                else:
                    print("[Quest] Calibration failed — no Quest pose data")
                    calibrate_at = None

            if frame is not None:
                buttons = frame.buttons or {}
                stick_click = bool(buttons.get("left_stick_click"))
                stop_combo = bool(buttons.get("stop_combo"))

                if stop_combo and not prev_stop and mode != StreamMode.OFF:
                    mode = StreamMode.OFF
                    socket.send(build_command_message(start=False, stop=True, planner=True))
                    print("[Quest] Emergency stop (Quest buttons)")
                elif stick_click and not prev_stick_click and mode == StreamMode.PLANNER:
                    if frame is not None and (
                        three_point.is_calibrated or _recalibrate_quest(frame, start_tracking=True)
                    ):
                        _enable_arm_tracking()
                        print("[Quest] Arm VR_3PT (left stick click)")

                prev_stick_click = stick_click
                prev_stop = stop_combo

            if joint_wizard.running and frame is not None:
                raw_for_wizard = _raw_pose(frame)
                joint_wizard.tick(now, raw_for_wizard)
                if not joint_wizard.running and joint_wizard.offsets.active:
                    _freeze_torso(frame)
                    three_point.calibrate_from_vr3pt(
                        joint_wizard.offsets.apply(_raw_pose(frame)),
                        full_reset=True,
                    )
                    _enable_arm_tracking()
                    print("[Quest] FK recalibrated after joint wizard")

            # VR 3-point arm target — same fields Pico sends in VR_3PT mode.
            vr_pos = None
            vr_orn = None
            if mode == StreamMode.PLANNER_VR_3PT and frame is not None:
                raw = _raw_pose(frame)
                raw = joint_wizard.offsets.apply(raw)
                calibrated = three_point.process_vr3pt_pose(raw)

                cl = sRot.from_quat(raw[0, 3:7], scalar_first=True)
                cr = sRot.from_quat(raw[1, 3:7], scalar_first=True)
                on_capture = bool(wrist_ref.get("capture") and three_point.is_calibrated)

                if wrist_ref.get("active") and not on_capture and "r0_l" in wrist_ref:
                    if wrist_ref.get("freeze"):
                        wl = wrist_ref["r0_l"]
                        wr = wrist_ref["r0_r"]
                    else:
                        wl = (cl * wrist_ref["c0_l"].inv()) * wrist_ref["r0_l"]
                        wr = (cr * wrist_ref["c0_r"].inv()) * wrist_ref["r0_r"]
                    calibrated[0, 3:7] = wl.as_quat(scalar_first=True)
                    calibrated[1, 3:7] = wr.as_quat(scalar_first=True)

                reach_origin = (
                    wrist_ref.get("p0")
                    if wrist_ref.get("active") and not on_capture
                    else None
                )
                reach_scale = args.arm_scale if reach_origin is not None else 1.0
                calibrated = apply_quest_vr3pt_axis_fix(
                    calibrated, reach_scale=reach_scale, reach_origin=reach_origin
                )

                if on_capture or (
                    wrist_ref.get("active") and wrist_ref.get("p0") is None
                ):
                    wrist_ref["r0_l"] = sRot.from_quat(calibrated[0, 3:7], scalar_first=True)
                    wrist_ref["r0_r"] = sRot.from_quat(calibrated[1, 3:7], scalar_first=True)
                    wrist_ref["p0"] = calibrated[:2, :3].copy()

                if on_capture:
                    wrist_ref["c0_l"] = cl
                    wrist_ref["c0_r"] = cr
                    wrist_ref["active"] = True
                    wrist_ref["capture"] = False
                    print("[Quest] Wrist reference captured — tracking from neutral pose")

                vr_pos = calibrated[:, :3].flatten().tolist()
                vr_orn = calibrated[:, 3:].flatten().tolist()

                if debug_axes and now - last_debug > 0.4:
                    last_debug = now
                    for hand, idx in (("L", 0), ("R", 1)):
                        pos = np.array(calibrated[idx, :3], dtype=np.float64)
                        key = f"dbg_ref_{hand}"
                        ref = wrist_ref.get(key)
                        if ref is None:
                            wrist_ref[key] = pos.copy()
                            ref = pos
                        d = pos - ref
                        labels = ["X(fwd)", "Y(left)", "Z(up)"]
                        i = int(np.argmax(np.abs(d)))
                        dom = f"{'+' if d[i] >= 0 else '-'}{labels[i]}"
                        print(
                            f"[dbg] {hand} pos=[{pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f}] "
                            f"dFromRef=[{d[0]:+.2f},{d[1]:+.2f},{d[2]:+.2f}] dominant={dom}"
                        )

            if mode in (StreamMode.PLANNER, StreamMode.PLANNER_VR_3PT):
                if frame is not None:
                    left_stick = tuple((frame.thumbstick.get("left", [0, 0]) or [0, 0])[:2])
                    right_stick = tuple((frame.thumbstick.get("right", [0, 0]) or [0, 0])[:2])
                    movement, speed, facing, turning = locomotion.step(left_stick, right_stick, dt)
                    if turning and speed < 0:
                        loco_mode = LocomotionMode.SLOW_WALK
                        speed = 0.15
                    elif speed > 0:
                        loco_mode = LocomotionMode.WALK
                    else:
                        loco_mode = LocomotionMode.IDLE
                else:
                    facing = [1.0, 0.0, 0.0]
                    movement = [0.0, 0.0, 0.0]
                    speed = -1.0
                    loco_mode = LocomotionMode.IDLE
                socket.send(
                    build_planner_message(
                        mode=int(loco_mode),
                        movement=movement,
                        facing=facing,
                        speed=speed if speed > 0 else -1.0,
                        height=-1.0,
                        vr_3pt_position=vr_pos,
                        vr_3pt_orientation=vr_orn,
                    )
                )

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[Quest] Shutting down…")
    finally:
        kb_state["running"] = False
        three_point.close()
        socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
