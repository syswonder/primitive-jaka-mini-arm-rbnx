#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""jaka_arm — Robonix arm primitive for the JAKA (jkrc) controller.

Owns `robonix/primitive/arm/*`:

  joint_states  topic_out  ROS 2 /jaka/joint_states (arm joints rad + gripper m)
  end_pose      topic_out  ROS 2 /jaka/end_pose      (TCP pose, base frame)
  joint_command topic_in   ROS 2 /jaka/joint_command (position-only write)
  pos_command   topic_in   ROS 2 /jaka/pos_command   (TCP target; IK on provider)

Frame, units and writable-field conventions are documented in CAPABILITY.md.
Motion commands are rejected while `motion_enabled` is false (default) —
a real safety mode, not a fake disable; see config.spec.
"""
from __future__ import annotations

import queue
import threading
import time

from robonix_api import Primitive, Ok, Err
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from .driver import (
    JakaDriver,
    JOINT_COUNT,
    JOINT_NAMES,
    GRIPPER_JOINT_NAME,
    GRIPPER_STROKE_M,
)

provider = Primitive(id="jaka_arm", namespace="robonix/primitive/arm")

ALL_JOINT_NAMES = JOINT_NAMES + [GRIPPER_JOINT_NAME]

# ---- module state -----------------------------------------------------------
_driver: JakaDriver | None = None
_cfg: dict = {}
_stop_event = threading.Event()
_state_thread: threading.Thread | None = None
_cmd_queue: "queue.Queue[tuple[str, object] | None]" = queue.Queue()
_cmd_thread: threading.Thread | None = None
_last_motion_warn_s = 0.0
_last_gripper_open_m: float | None = None
_shutdown_done = False


# ---- lifecycle --------------------------------------------------------------
@provider.on_init
def init(cfg: dict):
    global _cfg
    ip = cfg.get("ip")
    if not ip:
        return Err("config `ip` is required (JAKA controller address)")
    _cfg = dict(cfg)

    topics = cfg.get("topics") or {}
    provider.create_publisher(
        contract_id="robonix/primitive/arm/joint_states",
        topic=topics.get("joint_states", "/jaka/joint_states"),
        msg_type=JointState,
        qos="reliable",
    )
    provider.create_publisher(
        contract_id="robonix/primitive/arm/end_pose",
        topic=topics.get("end_pose", "/jaka/end_pose"),
        msg_type=Pose,
        qos="reliable",
    )
    provider.create_subscription(
        contract_id="robonix/primitive/arm/joint_command",
        topic=topics.get("joint_command", "/jaka/joint_command"),
        msg_type=JointState,
        callback=_on_joint_command,
        qos="reliable",
    )
    provider.create_subscription(
        contract_id="robonix/primitive/arm/pos_command",
        topic=topics.get("pos_command", "/jaka/pos_command"),
        msg_type=Pose,
        callback=_on_pos_command,
        qos="reliable",
    )
    return Ok()


@provider.on_activate
def activate():
    global _driver, _state_thread, _cmd_thread
    if _driver is None:
        _driver = JakaDriver(
            ip=_cfg["ip"],
            joint_speed_rad_s=float(_cfg.get("joint_speed_rad_s", 1.0)),
            linear_speed_mm_s=float(_cfg.get("linear_speed_mm_s", 100.0)),
            gripper_force=int(_cfg.get("gripper_force", 50)),
            gripper_speed=int(_cfg.get("gripper_speed", 1100)),
            reach_mse_threshold_deg2=float(_cfg.get("reach_mse_threshold_deg2", 1.0)),
            move_timeout_s=float(_cfg.get("move_timeout_s", 10.0)),
        )
        try:
            _driver.connect()
        except Exception as exc:  # noqa: BLE001
            _driver = None
            return Err(f"connect failed: {exc}")
    if _state_thread is None or not _state_thread.is_alive():
        _stop_event.clear()
        _state_thread = threading.Thread(target=_state_loop, daemon=True)
        _state_thread.start()
    if _cmd_thread is None or not _cmd_thread.is_alive():
        _cmd_thread = threading.Thread(target=_command_worker, daemon=True)
        _cmd_thread.start()
    return Ok()


@provider.on_deactivate
def deactivate():
    """Stop publishing and abort motion; keep the controller connection."""
    _stop_event.set()
    if _state_thread is not None:
        _state_thread.join(timeout=2.0)
    if _driver is not None:
        _driver.stop_motion()
    return Ok()


@provider.on_shutdown
def shutdown():
    """Final cleanup: stop threads, optional homing, power off, logout.

    The framework may invoke this twice on one teardown (Driver CMD_SHUTDOWN,
    then the process-signal fallback); the first call wins.
    """
    global _shutdown_done
    if _shutdown_done:
        return Ok()
    _shutdown_done = True
    _stop_event.set()
    if _cmd_thread is not None:
        # Sentinel before join: the worker blocks on get() until it sees None,
        # so joining first would stall for the full timeout.
        _cmd_queue.put(None)
    for thread in (_state_thread, _cmd_thread):
        if thread is not None:
            thread.join(timeout=2.0)
    if _driver is not None:
        _home_before_shutdown()
        _driver.disconnect()
    return Ok()


def _home_before_shutdown() -> None:
    """Best-effort gripper release + move to `shutdown_home_joints` before
    the power-off sequence.

    The gripper opens first so a held object is not dragged by the joint
    move. Deployment opt-in: the key absent = no motion on exit. Respects
    the motion_enabled safety gate; bounded by the driver's move_timeout_s
    and never raises — power-off always proceeds.
    """
    home = _cfg.get("shutdown_home_joints")
    if not home:
        return
    if not bool(_cfg.get("motion_enabled", False)):
        print("[jaka_arm] shutdown: motion_enabled is false; skipping homing",
              flush=True)
        return
    try:
        target = [float(v) for v in home]
    except (TypeError, ValueError):
        print(f"[jaka_arm] shutdown: invalid shutdown_home_joints {home!r}; "
              "skipping homing", flush=True)
        return
    print(f"[jaka_arm] shutdown: opening gripper, homing to "
          f"{[round(v, 3) for v in target]}", flush=True)
    try:
        # Open first (drops any held object); a failed release is logged but
        # does not block the homing move.
        if not _driver.set_gripper_open_m(GRIPPER_STROKE_M):
            print("[jaka_arm] shutdown: gripper open failed", flush=True)
        if not _driver.move_joints_rad(target):
            print("[jaka_arm] shutdown: homing failed or timed out", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[jaka_arm] shutdown: homing failed: {exc}", flush=True)


# ---- ROS 2 callbacks (executor thread) --------------------------------------
def _on_joint_command(msg: JointState) -> None:
    _cmd_queue.put(("joint", msg))


def _on_pos_command(msg: Pose) -> None:
    _cmd_queue.put(("pose", msg))


# ---- command worker ---------------------------------------------------------
def _command_worker() -> None:
    while True:
        item = _cmd_queue.get()
        if item is None:
            return
        kind, payload = item
        try:
            if kind == "joint":
                _exec_joint_command(payload)
            else:
                _exec_pos_command(payload)
        except Exception as exc:  # noqa: BLE001
            print(f"[jaka_arm] command failed: {exc}", flush=True)


def _motion_denied() -> bool:
    """Motion gate with a rate-limited warning. True = command rejected."""
    if bool(_cfg.get("motion_enabled", False)):
        return False
    global _last_motion_warn_s
    now = time.monotonic()
    if now - _last_motion_warn_s > 10.0:
        _last_motion_warn_s = now
        print("[jaka_arm] motion command rejected: motion_enabled is false",
              flush=True)
    return True


def _exec_joint_command(msg: JointState) -> None:
    if _motion_denied():
        return
    names = list(msg.name)
    positions = list(msg.position)
    if len(names) != len(positions) or not names:
        print(f"[jaka_arm] joint_command: name/position length mismatch "
              f"({len(names)} vs {len(positions)})", flush=True)
        return
    unknown = set(names) - set(ALL_JOINT_NAMES)
    if unknown:
        print(f"[jaka_arm] joint_command: unknown joints {sorted(unknown)}",
              flush=True)
        return

    # Fill missing arm joints from current feedback so a subset is allowed.
    current = _driver.get_joint_positions_rad()
    if current is None:
        print("[jaka_arm] joint_command: no current joint feedback", flush=True)
        return
    target = dict(zip(JOINT_NAMES, current, strict=True))
    gripper_open_m = None
    for name, value in zip(names, positions, strict=True):
        if name == GRIPPER_JOINT_NAME:
            gripper_open_m = float(max(0.0, min(float(value), GRIPPER_STROKE_M)))
        else:
            target[name] = float(value)

    if gripper_open_m is not None:
        _driver.set_gripper_open_m(gripper_open_m)
    if not _driver.move_joints_rad([target[n] for n in JOINT_NAMES]):
        print("[jaka_arm] joint_command: joint move failed", flush=True)


def _exec_pos_command(msg: Pose) -> None:
    if _motion_denied():
        return
    from scipy.spatial.transform import Rotation as R

    rpy = R.from_quat([msg.orientation.x, msg.orientation.y,
                       msg.orientation.z, msg.orientation.w]).as_euler("xyz")
    if not _driver.move_pose([msg.position.x, msg.position.y, msg.position.z],
                             [float(a) for a in rpy]):
        print("[jaka_arm] pos_command: move failed", flush=True)


# ---- state publisher ---------------------------------------------------------
def _state_loop() -> None:
    hz = float(_cfg.get("state_publish_hz", 50.0))
    period = max(0.01, 1.0 / hz)
    global _last_gripper_open_m
    while not _stop_event.wait(period):
        joints = _driver.get_joint_positions_rad()
        gripper = _driver.get_gripper_open_m()
        pose = _driver.get_tcp_pose()
        if joints is None:
            continue
        if gripper is not None:
            _last_gripper_open_m = gripper
        state = JointState()
        state.header.frame_id = str(_cfg.get("base_frame", "world"))
        if _last_gripper_open_m is None:
            # No gripper reading yet (first RS485 read still pending, or the
            # channel is down): publish the arm joints alone so arm feedback
            # never depends on gripper health. Once a valid gripper reading
            # arrives the full name set (arm + gripper_joint) resumes on
            # every publish.
            state.name = JOINT_NAMES
            state.position = list(joints)
        else:
            state.name = ALL_JOINT_NAMES
            state.position = list(joints) + [_last_gripper_open_m]
        provider.emit("robonix/primitive/arm/joint_states", state)
        if pose is not None:
            from scipy.spatial.transform import Rotation as R

            pos, rpy = pose
            quat = R.from_euler("xyz", rpy).as_quat()
            end = Pose()
            end.position.x, end.position.y, end.position.z = pos
            end.orientation.x, end.orientation.y = quat[0], quat[1]
            end.orientation.z, end.orientation.w = quat[2], quat[3]
            provider.emit("robonix/primitive/arm/end_pose", end)


if __name__ == "__main__":
    provider.run()
