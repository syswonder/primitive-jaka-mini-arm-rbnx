# SPDX-License-Identifier: MIT
# -*- coding: utf-8 -*-
# Adapted from AIIT-Roboarm (arm/jaka_ctrl_by_sdk.py), MIT License,
# Copyright (c) 2025 Xu Jinyang & Hou Yunlong. See NOTICE.
"""JakaDriver — minimal jkrc SDK adapter for the Robonix jaka_arm primitive.

Surface: connect/disconnect/stop_motion, joint & TCP feedback, joint move,
Cartesian move with controller-side IK fallback, binary adaptive gripper.

Units: joints in rad (SDK native), TCP pose in m + rpy rad, gripper opening
in m (TG-9801 stroke, normalized). All RC calls are serialized by one RLock
so the state-publish thread can interleave with motion commands.
"""
from __future__ import annotations

import threading
import time

import numpy as np

from . import jaka_sdk  # noqa: F401  (platform loader; must precede jkrc)
import jkrc  # noqa: E402

from .gripper import JakaGripper

ABS = 0  # jkrc absolute motion

JOINT_COUNT = 6
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
GRIPPER_JOINT_NAME = "gripper_joint"
# TG-9801 total stroke (datasheet: 可调行程 0~98 mm)
GRIPPER_STROKE_M = 0.098

# JAKA Mini soft limits (user manual table 5-1, degrees). Verified against
# the real arm before enabling motion; placeholder limits are from Zu3 docs.
JOINT_LIMITS_DEG = (
    (-360.0, 360.0), (-125.0, 125.0), (-130.0, 130.0),
    (-360.0, 360.0), (-120.0, 120.0), (-360.0, 360.0),
)
# j5 near 0 (wrist singularity) or near its limit cannot start Cartesian moves
SINGULAR_J5_EPS_DEG = 5.0
# Known-good grasp configuration (measured on the real arm), fallback IK seed
GRASP_IK_SEED_DEG = [46.2, -20.6, -76.1, 0.0, -83.4, 76.2]


class JakaDriver:
    """Blocking, poll-to-wait adapter over jkrc.RC. Not thread-safe by itself;
    every RC call is guarded by the internal lock."""

    def __init__(
        self,
        ip: str,
        joint_speed_rad_s: float = 1.0,
        linear_speed_mm_s: float = 100.0,
        gripper_force: int = 50,
        gripper_speed: int = 1100,
        reach_mse_threshold_deg2: float = 1.0,
        move_timeout_s: float = 10.0,
    ):
        self._ip = ip
        self._joint_speed_rad_s = float(joint_speed_rad_s)
        self._linear_speed_mm_s = float(linear_speed_mm_s)
        self._gripper_force = int(gripper_force)
        self._gripper_speed = int(gripper_speed)
        self._reach_mse_threshold_deg2 = float(reach_mse_threshold_deg2)
        self._move_timeout_s = float(move_timeout_s)
        self._lock = threading.RLock()
        self._rc = None
        self._gripper = None
        self._gripper_close_pos = 1000  # fallback: 1000 = closed (reverse travel)
        self._gripper_open_pos = 0      # fallback: 0 = open
        self._connected = False

    # ---------- lifecycle ----------

    def connect(self) -> None:
        """Login, power on, enable, and bring up the RS485 gripper channel."""
        with self._lock:
            if self._connected:
                return
            self._rc = jkrc.RC(self._ip)
            ret = self._rc.login()
            if ret[0] != 0:
                raise ConnectionError(f"JAKA login failed ({self._ip}): {ret}")
            self._gripper = JakaGripper(robot=self._rc)
            self._gripper.power_on()  # re-arms RS485 channel after power cycle
            self._rc.enable_robot()
            self._gripper.set_target_force(self._gripper_force)

            # Cache travel endpoints for opening normalization; controller
            # reads are occasionally bogus (0/0), fall back to measured travel.
            open_pos = self._gripper.read_open_position()
            close_pos = self._gripper.read_close_position()
            endpoints_valid = (
                open_pos is not None and close_pos is not None
                and open_pos != close_pos
                and 0 <= open_pos <= 1000 and 0 <= close_pos <= 1000
            )
            if endpoints_valid:
                self._gripper_open_pos = open_pos
                self._gripper_close_pos = close_pos
            print(
                f"[jaka_arm] TG-9801 travel: close={self._gripper_close_pos} "
                f"open={self._gripper_open_pos}"
                f"{'' if endpoints_valid else ' (fallback, reads invalid)'}",
                flush=True,
            )
            self._connected = True

    def disconnect(self) -> None:
        """Idempotent teardown: leave drag mode, stop motion, power off, logout.

        Mirrors AIIT disconnect_arm; every step is best-effort so a partially
        connected controller still releases cleanly.
        """
        with self._lock:
            if not self._connected:
                return
            for step in (
                lambda: self._rc.drag_mode_enable(False),
                lambda: self._rc.enable_robot(),
                lambda: self._rc.motion_abort(),
                lambda: self._rc.disable_robot(),
                lambda: self._rc.power_off(),
                lambda: self._rc.logout(),
            ):
                try:
                    step()
                except Exception as exc:  # noqa: BLE001
                    print(f"[jaka_arm] disconnect step failed: {exc}", flush=True)
            self._connected = False
            self._gripper = None
            self._rc = None

    def stop_motion(self) -> None:
        with self._lock:
            if self._connected:
                try:
                    self._rc.motion_abort()
                except Exception as exc:  # noqa: BLE001
                    print(f"[jaka_arm] motion_abort failed: {exc}", flush=True)

    # ---------- feedback ----------

    def get_joint_positions_rad(self) -> list[float] | None:
        with self._lock:
            if not self._connected:
                return None
            ret = self._rc.get_actual_joint_position()
            if ret[0] != 0:
                print(f"[jaka_arm] get_actual_joint_position error {ret[0]}", flush=True)
                return None
            return [float(a) for a in ret[1]]

    def get_gripper_open_m(self) -> float | None:
        with self._lock:
            if not self._connected:
                return None
            pos = self._gripper.read_position()
            if pos is None:
                return None
            lo, hi = self._gripper_close_pos, self._gripper_open_pos
            if hi == lo:
                return 0.0
            value = (pos - lo) / (hi - lo)  # 1 = fully open (both travel dirs)
            return float(np.clip(value, 0, 1)) * GRIPPER_STROKE_M

    def get_tcp_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
        """TCP pose as ((x, y, z) m, (rx, ry, rz) rad, R = Rz·Ry·Rx order)."""
        with self._lock:
            if not self._connected:
                return None
            ret = self._rc.get_actual_tcp_position()
            if ret[0] != 0:
                print(f"[jaka_arm] get_actual_tcp_position error {ret[0]}", flush=True)
                return None
            x, y, z, rx, ry, rz = ret[1]
            return (x / 1000.0, y / 1000.0, z / 1000.0), (float(rx), float(ry), float(rz))

    # ---------- motion ----------

    @staticmethod
    def _within_limits(angles_rad: list[float]) -> bool:
        return all(
            lo - 1e-6 <= np.degrees(a) <= hi + 1e-6
            for a, (lo, hi) in zip(angles_rad, JOINT_LIMITS_DEG, strict=True)
        )

    def move_joints_rad(self, positions_rad: list[float]) -> bool:
        """Start a joint move (non-blocking SDK call) and wait until reached.

        State reads keep flowing between polls, so Soma's freshness window is
        not held up by a long blocking SDK call.
        """
        if len(positions_rad) != JOINT_COUNT:
            raise ValueError(f"expected {JOINT_COUNT} joint positions")
        if not all(np.isfinite(positions_rad)):
            raise ValueError("joint positions must be finite")
        if not self._within_limits(positions_rad):
            print(f"[jaka_arm] joint target out of limits: "
                  f"{[round(np.degrees(a), 1) for a in positions_rad]}", flush=True)
            return False
        with self._lock:
            if not self._connected:
                return False
            ret = self._rc.joint_move(positions_rad, ABS, False, self._joint_speed_rad_s)
            if ret[0] != 0:
                print(f"[jaka_arm] joint_move error {ret[0]}", flush=True)
                return False
        return self._wait_reached(positions_rad)

    def move_pose(self, pos_m: list[float], rpy_rad: list[float]) -> bool:
        """Cartesian move; on linear failure or singular start, fall back to
        controller-side IK plus a joint move (AIIT hybrid strategy)."""
        pose = [pos_m[0] * 1000, pos_m[1] * 1000, pos_m[2] * 1000, *rpy_rad]

        joints = self.get_joint_positions_rad()
        if joints is None:
            return False
        j5_deg = np.degrees(joints[4])
        near_singular = abs(j5_deg) < SINGULAR_J5_EPS_DEG or abs(j5_deg) > (
            JOINT_LIMITS_DEG[4][1] - SINGULAR_J5_EPS_DEG
        )
        if not near_singular:
            with self._lock:
                if not self._connected:
                    return False
                ret = self._rc.linear_move(pose, ABS, False, self._linear_speed_mm_s)
            if ret[0] == 0 and self._wait_pose_reached(tuple(pos_m)):
                return True
            print(f"[jaka_arm] linear_move failed ({ret[0]}), falling back to IK",
                  flush=True)

        solution = self._solve_ik_deg(pose, joints)
        if solution is None:
            print(f"[jaka_arm] no IK solution within limits for pose "
                  f"{[round(v, 2) for v in pose]}", flush=True)
            return False
        return self.move_joints_rad([float(np.radians(a)) for a in solution])

    def set_gripper_open_m(self, open_m: float) -> bool:
        """TG-9801 adaptive mode is binary: >= half stroke = release, else grip."""
        with self._lock:
            if not self._connected:
                return False
            if open_m >= 0.5 * GRIPPER_STROKE_M:
                return self._gripper.release(self._gripper_speed) == 0
            return self._gripper.grip(self._gripper_speed) == 0

    # ---------- internals ----------

    def _wait_reached(self, target_rad: list[float]) -> bool:
        """Poll joint feedback until MSE vs target drops below threshold."""
        deadline = time.monotonic() + self._move_timeout_s
        while time.monotonic() < deadline:
            current = self.get_joint_positions_rad()
            if current is not None and len(current) == len(target_rad):
                mse_deg2 = sum(
                    (np.degrees(c) - np.degrees(t)) ** 2
                    for c, t in zip(current, target_rad, strict=True)
                ) / len(target_rad)
                if mse_deg2 <= self._reach_mse_threshold_deg2:
                    return True
            time.sleep(0.05)
        print(f"[jaka_arm] wait_until_reached timeout ({self._move_timeout_s}s)",
              flush=True)
        return False

    def _wait_pose_reached(self, target_m: tuple[float, float, float]) -> bool:
        deadline = time.monotonic() + self._move_timeout_s
        while time.monotonic() < deadline:
            pose = self.get_tcp_pose()
            if pose is not None:
                pos = pose[0]
                dist = sum((p - t) ** 2 for p, t in zip(pos, target_m, strict=True)) ** 0.5
                if dist <= 0.002:  # 2 mm
                    return True
            time.sleep(0.05)
        print(f"[jaka_arm] wait_pose_reached timeout ({self._move_timeout_s}s)",
              flush=True)
        return False

    def _solve_ik_deg(self, pose_mm_rpy: list[float], ref_rad: list[float]) -> list[float] | None:
        """Controller IK with limit repair: nearest branch, then wrist flip,
        then the known-good grasp seed (AIIT strategy)."""
        for seed in (ref_rad, [np.radians(a) for a in GRASP_IK_SEED_DEG]):
            with self._lock:
                if not self._connected:
                    return None
                ret = self._rc.kine_inverse(seed, pose_mm_rpy)
            if ret[0] != 0:
                continue
            solution = [float(np.degrees(a)) for a in ret[1]]
            if self._within_limits([float(np.radians(a)) for a in solution]):
                return solution
            flipped = self._wrist_flip(solution)
            if self._within_limits([float(np.radians(a)) for a in flipped]):
                return flipped
        return None

    @staticmethod
    def _wrist_flip(angles_deg: list[float]) -> list[float]:
        """Wrist-equivalent configuration (j4∓180, -j5, j6±180); same TCP pose."""
        flip = 180.0 if angles_deg[3] > 0 else -180.0
        return [
            angles_deg[0], angles_deg[1], angles_deg[2],
            angles_deg[3] - flip, -angles_deg[4], angles_deg[5] + flip,
        ]
