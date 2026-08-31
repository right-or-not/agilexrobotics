"""GELLO Robot-protocol adapter for a six-axis PiPER-X."""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from typing import Any

import numpy as np

from agilexrobotics.driver import PiperXDriver
from agilexrobotics.exceptions import FeedbackError


def _rpy_to_xyzw(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert XYZ fixed-axis roll/pitch/yaw to an XYZW quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )


class GelloPiperXRobot:
    """Duck-typed implementation of ``gello.robots.robot.Robot``.

    适配器向 GELLO 暴露六个机械臂关节和一个归一化夹爪自由度。夹爪使用
    ``0=全闭、1=全开``，再正向映射到 AGX 夹爪的实际开口宽度。
    """

    def __init__(
        self,
        *,
        channel: str = "can0",
        interface: str = "socketcan",
        firmware: Any = "default",
        configure_motion_limits: bool = True,
        startup_wait: float = 1.0,
        max_command_delta_rad: float = 1.0,
        gripper_max_width_m: float = 0.1,
        gripper_force_n: float = 1.0,
        driver: Any | None = None,
    ) -> None:
        if startup_wait < 0:
            raise ValueError("startup wait must be non-negative")
        if not math.isfinite(gripper_max_width_m) or gripper_max_width_m <= 0:
            raise ValueError("gripper max width must be a positive finite value")
        if not math.isfinite(gripper_force_n) or gripper_force_n < 0:
            raise ValueError("gripper force must be a non-negative finite value")
        self._driver = driver or PiperXDriver(
            channel=channel,
            interface=interface,
            firmware=firmware,
            max_command_delta_rad=max_command_delta_rad,
        )
        self._configure_motion_limits = configure_motion_limits
        self._startup_wait = startup_wait
        self._gripper_max_width_m = gripper_max_width_m
        self._gripper_force_n = gripper_force_n
        self._started = False
        self._last_positions: np.ndarray | None = None
        self._last_timestamp: float | None = None
        self._last_velocities = np.zeros(7, dtype=np.float64)
        # 夹爪反馈可能比机械臂反馈晚到；首次缺失时按 GELLO 的全闭值 0 处理。
        self._gripper_position = 0.0
        self.motion_limits_confirmed = False

    def start(self) -> None:
        if self._started:
            return
        self._driver.connect()
        try:
            if self._startup_wait:
                time.sleep(self._startup_wait)
            if self._configure_motion_limits:
                self.motion_limits_confirmed = (
                    self._driver.maximize_joint_motion_limits(require_confirmation=True)
                )
            self._driver.enable()
            time.sleep(0.5)
            state = self._driver.get_state()
            # 启动阶段只记录当前反馈，不发送 set_speed_percent 或普通
            # move_j，避免上一次 GELLO 退出后控制器仍保留 JS 模式时
            # 被启动流程切换模式。第一帧客户端目标到达后才开始 JS 跟随。
            positions = self._joint_state_with_gripper(state.joint_positions_rad)
            self._remember_state(positions, state.feedback_timestamp)
        except Exception:
            self._driver.close()
            raise
        self._started = True

    def close(self, *, disable: bool = False) -> None:
        if not self._started:
            return
        try:
            if disable:
                self._driver.disable()
        finally:
            self._driver.close()
            self._started = False

    def num_dofs(self) -> int:
        """返回六个机械臂关节加一个夹爪自由度。"""
        return 7

    def get_joint_state(self) -> np.ndarray:
        """返回六轴弧度和归一化夹爪位置组成的七维状态。"""
        state = self._get_state()
        positions = self._joint_state_with_gripper(state.joint_positions_rad)
        self._remember_state(positions, state.feedback_timestamp)
        return positions

    def command_joint_state(
        self, joint_state: Sequence[float] | np.ndarray[Any, Any]
    ) -> None:
        target = np.asarray(joint_state, dtype=np.float64)
        if target.shape != (7,):
            raise ValueError(
                "PiPER-X GELLO command requires 6 arm joints and 1 gripper, "
                f"got {target.shape}"
            )
        if not np.all(np.isfinite(target)):
            raise ValueError("PiPER-X GELLO command must contain finite values")
        self._require_started()
        # 第一帧客户端目标到达时才进入 JS 流式会话，避免服务端启动后
        # 在没有跟随目标的情况下长时间停留在 JS 模式。会话内只设置一次模式，
        # 后续帧直接使用 move_js 刷新六轴目标。
        self._driver.begin_fast_response_mode()
        self._driver.command_joints(target[:6].tolist(), fast_response=True)
        # GELLO 使用 0=全闭、1=全开；AGX 同样随数值增大而张开，因此直接映射。
        gripper = float(np.clip(target[6], 0.0, 1.0))
        width_m = gripper * self._gripper_max_width_m
        self._driver.command_gripper(width_m, self._gripper_force_n)
        # 命令成功后立即缓存目标；在下一帧真实反馈到达前保持观测连续。
        self._gripper_position = gripper

    def get_observations(self) -> dict[str, np.ndarray]:
        state = self._get_state()
        positions = self._joint_state_with_gripper(state.joint_positions_rad)
        self._remember_state(positions, state.feedback_timestamp)
        ee_pos_quat = np.zeros(7, dtype=np.float64)
        if state.flange_pose_m_rad is not None:
            pose = state.flange_pose_m_rad
            ee_pos_quat[:3] = pose[:3]
            ee_pos_quat[3:] = _rpy_to_xyzw(*pose[3:])
        return {
            "joint_positions": positions,
            "joint_velocities": self._last_velocities.copy(),
            "ee_pos_quat": ee_pos_quat,
            "gripper_position": np.array(positions[6], dtype=np.float64),
        }

    def _joint_state_with_gripper(
        self, arm_positions: Sequence[float] | np.ndarray[Any, Any]
    ) -> np.ndarray:
        """将六轴状态与正向归一化后的夹爪状态合并为七维数组。"""
        try:
            status = self._driver.get_gripper_status()
        except FeedbackError:
            # 夹爪反馈缺失不应阻止机械臂服务启动；沿用最近一次有效状态或命令。
            gripper = self._gripper_position
        else:
            width_m = float(status["value"])
            normalized = width_m / self._gripper_max_width_m
            gripper = float(np.clip(normalized, 0.0, 1.0))
            self._gripper_position = gripper
        return np.concatenate(
            (np.asarray(arm_positions, dtype=np.float64), np.array([gripper]))
        )

    def _get_state(self) -> Any:
        self._require_started()
        return self._driver.get_state()

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("start() must succeed before using the GELLO robot")

    def _remember_state(
        self,
        positions: Sequence[float] | np.ndarray[Any, Any],
        timestamp: float,
    ) -> None:
        current = np.asarray(positions, dtype=np.float64)
        if self._last_positions is not None and self._last_timestamp is not None:
            elapsed = timestamp - self._last_timestamp
            if elapsed > 0:
                self._last_velocities = (current - self._last_positions) / elapsed
        self._last_positions = current.copy()
        self._last_timestamp = timestamp
