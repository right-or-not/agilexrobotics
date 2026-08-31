"""在官方 pyAgxArm SDK 外提供状态校验与运动安全保护。"""

from __future__ import annotations

import math
import re
import time
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from typing import Any

from agilexrobotics.exceptions import (
    ArmStateError,
    CommunicationError,
    FeedbackError,
    JointCommandError,
    NotEnabledError,
    PiperXError,
)
from agilexrobotics.piper_x import Firmware, create_piper_x_arm

# PiPER-X SDK 配置中的标称关节范围，单位为弧度。
# 真实设备的零点反馈可能略微超出标称端点，因此实际校验时还会额外加入一个较小的标定余量
PIPER_X_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-2.617994, 2.617994),
    (0.0, 3.141593),
    (-2.967060, 0.0),
    (-1.553344, 1.553344),
    (-1.553344, 1.553344),
    (-3.141593, 3.141593),
)
PIPER_X_MAX_JOINT_SPEED_RAD_S = 3.0
PIPER_X_MAX_JOINT_ACCELERATION_RAD_S2 = 5.0

_ERROR_FLAG_NAMES = (
    "joint_1_angle_limit",
    "joint_2_angle_limit",
    "joint_3_angle_limit",
    "joint_4_angle_limit",
    "joint_5_angle_limit",
    "joint_6_angle_limit",
    "communication_status_joint_1",
    "communication_status_joint_2",
    "communication_status_joint_3",
    "communication_status_joint_4",
    "communication_status_joint_5",
    "communication_status_joint_6",
)


@dataclass(frozen=True)
class ArmErrorFlags:
    joint_1_angle_limit: bool = False
    joint_2_angle_limit: bool = False
    joint_3_angle_limit: bool = False
    joint_4_angle_limit: bool = False
    joint_5_angle_limit: bool = False
    joint_6_angle_limit: bool = False
    communication_status_joint_1: bool = False
    communication_status_joint_2: bool = False
    communication_status_joint_3: bool = False
    communication_status_joint_4: bool = False
    communication_status_joint_5: bool = False
    communication_status_joint_6: bool = False

    @property
    def any(self) -> bool:
        """只要存在任意关节限位或通信错误，就返回 `True`"""

        # 将 dataclass 转为字典后统一检查所有布尔标志
        return any(asdict(self).values())


@dataclass(frozen=True)
class PiperXState:
    joint_positions_rad: tuple[float, ...]
    flange_pose_m_rad: tuple[float, ...] | None
    control_mode: int
    arm_status: int
    motion_status: int
    enabled_joints: tuple[bool, ...]
    errors: ArmErrorFlags
    feedback_timestamp: float
    feedback_age: float
    receive_hz: float
    communication_ok: bool

    def as_dict(self) -> dict[str, Any]:
        """将不可变状态对象转换为便于 JSON 序列化的字典"""

        # asdict() 会递归转换内部的 ArmErrorFlags
        return asdict(self)


def _message_values(message: Any, expected_length: int, name: str) -> tuple[float, ...]:
    """从 SDK 消息中提取指定数量的有限浮点数"""

    # 没有消息通常表示周期反馈尚未到达，不能继续使用空数据。
    if message is None:
        raise FeedbackError(f"{name} feedback is not available")
    # 将 SDK 自定义数值统一转换为 Python float，并把结构错误包装成反馈异常。
    try:
        values = tuple(float(value) for value in message.msg)
    except (AttributeError, TypeError, ValueError) as exc:
        raise FeedbackError(f"{name} feedback is malformed") from exc
    # 数量不符、NaN 或无穷大都会使后续运动计算不可靠。
    if len(values) != expected_length or not all(math.isfinite(v) for v in values):
        raise FeedbackError(
            f"{name} feedback must contain {expected_length} finite values"
        )
    return values


class PiperXDriver:
    """在调用 pyAgxArm 运动 API 前统一验证连接、状态和目标"""

    def __init__(
        self,
        *,
        channel: str = "can0",
        interface: str = "socketcan",
        firmware: Firmware = "default",
        feedback_timeout: float = 0.5,
        motor_confirmation_timeout: float = 2.0,
        motor_confirmation_interval: float = 0.01,
        max_command_delta_rad: float = 0.2,
        joint_limit_margin_rad: float = math.radians(5.0),
        sdk_arm: Any | None = None,
        clock: Callable[[], float] = time.time,
    ) -> None:
        """创建驱动并保存连接、超时和运动安全参数"""

        # 所有等待时间和运动阈值必须有效，否则循环可能无法正确结束。
        if feedback_timeout <= 0:
            raise ValueError("feedback_timeout must be positive")
        if motor_confirmation_timeout <= 0:
            raise ValueError("motor_confirmation_timeout must be positive")
        if motor_confirmation_interval <= 0:
            raise ValueError("motor_confirmation_interval must be positive")
        if max_command_delta_rad <= 0:
            raise ValueError("max_command_delta_rad must be positive")
        if joint_limit_margin_rad < 0:
            raise ValueError("joint_limit_margin_rad must be non-negative")
        # 测试可直接注入 sdk_arm；正常运行时根据 CAN 参数创建官方 SDK 对象。
        self._arm = (
            sdk_arm
            if sdk_arm is not None
            else create_piper_x_arm(
                channel=channel, interface=interface, firmware=firmware
            )
        )
        self._clock = clock
        self._feedback_timeout = feedback_timeout
        self._motor_confirmation_timeout = motor_confirmation_timeout
        self._motor_confirmation_interval = motor_confirmation_interval
        self._max_command_delta_rad = max_command_delta_rad
        self._joint_limit_margin_rad = joint_limit_margin_rad
        # 这两个标志记录本驱动自身完成的连接与使能，不能替代硬件反馈。
        self._connected = False
        self._enabled_by_driver = False
        # JS/MIT 流式会话期间只设置一次运动模式，后续只发送关节目标。
        self._fast_response_mode_active = False
        self._previous_auto_motion_mode: bool | None = None
        # pyAgxArm 规定同一个机械臂实例只能初始化一次同类末端执行器。
        # 连接时会在 CAN 读取线程启动前初始化夹爪，后续状态读取和
        # 控制命令共同复用该对象。
        self._gripper: Any | None = None

    def connect(self) -> None:
        """连接机械臂并启动 SDK 的后台反馈读取线程"""

        # 重复调用 connect() 时直接返回，防止启动多个读取线程。
        if self._connected:
            return
        # SDK 异常统一转换为项目自己的 CommunicationError。
        try:
            self._initialize_gripper()
            self._arm.connect(start_read_thread=True)
        except Exception as exc:
            raise CommunicationError(f"failed to connect to PiPER-X: {exc}") from exc
        self._connected = True

    def close(self) -> None:
        """断开 SDK 连接并清除驱动内部的连接与使能记录。"""

        # 未连接时无需调用 SDK，保证 close() 可以安全重复执行。
        if not self._connected:
            return
        # 即使 SDK 断开失败，finally 也会恢复内部状态，避免误认为仍可控制。
        try:
            if self._fast_response_mode_active:
                self.end_fast_response_mode()
            self._arm.disconnect()
        finally:
            self._connected = False
            self._enabled_by_driver = False
            self._fast_response_mode_active = False
            self._previous_auto_motion_mode = None

    def get_state(self) -> PiperXState:
        """读取并验证一份完整、实时的机械臂状态快照。"""

        # 先确认连接及 CAN 通信正常，再读取关节角和控制器状态。
        self._require_connected()
        if self._arm.has_comm_error():
            raise CommunicationError(
                f"CAN communication error: {self._arm.get_comm_error()}"
            )

        # 关节角消息必须包含六轴有限数值，状态消息也必须存在。
        joints_message = self._arm.get_joint_angles()
        joints = _message_values(joints_message, 6, "joint angle")
        status_message = self._arm.get_arm_status()
        if status_message is None:
            raise FeedbackError("arm status feedback is not available")
        status = status_message.msg
        # 采用两条核心消息中较旧的时间戳，保守计算整份状态的反馈年龄。
        timestamp = min(
            float(joints_message.timestamp),
            float(status_message.timestamp),
        )
        age = max(0.0, self._clock() - timestamp)
        if not math.isfinite(timestamp) or age > self._feedback_timeout:
            raise FeedbackError(
                f"feedback is stale ({age:.3f}s > {self._feedback_timeout:.3f}s)"
            )

        # 将 SDK 的各个错误位复制到明确的数据结构，便于统一安全检查。
        err_status = status.err_status
        errors = ArmErrorFlags(
            **{
                name: bool(getattr(err_status, name, False))
                for name in _ERROR_FLAG_NAMES
            }
        )
        # 末端位姿可能尚未到达，因此允许 flange 为 None；关节使能状态必须有六项。
        flange_message = self._arm.get_flange_pose()
        flange = (
            None
            if flange_message is None
            else _message_values(flange_message, 6, "flange pose")
        )
        enabled = tuple(
            bool(value) for value in self._arm.get_joints_enable_status_list()
        )
        if len(enabled) != 6:
            raise FeedbackError("joint enable feedback must contain 6 values")

        return PiperXState(
            joint_positions_rad=joints,
            flange_pose_m_rad=flange,
            control_mode=int(status.ctrl_mode),
            arm_status=int(status.arm_status),
            motion_status=int(status.motion_status),
            enabled_joints=enabled,
            errors=errors,
            feedback_timestamp=timestamp,
            feedback_age=age,
            receive_hz=float(status_message.hz),
            communication_ok=True,
        )

    def get_motor_diagnostics(self) -> list[dict[str, Any]]:
        """读取六个关节的电机、驱动器及 FOC 状态"""

        # 诊断信息来自真实硬件反馈，因此必须先建立连接。
        self._require_connected()
        diagnostics: list[dict[str, Any]] = []
        status_names = (
            "voltage_too_low",
            "motor_overheating",
            "driver_overcurrent",
            "driver_overheating",
            "collision_status",
            "driver_error_status",
            "driver_enable_status",
            "stall_status",
        )
        # SDK 的关节编号从 1 开始，逐轴读取电机与驱动器两类消息。
        for joint_index in range(1, 7):
            motor_message = self._arm.get_motor_states(joint_index)
            driver_message = self._arm.get_driver_states(joint_index)
            if motor_message is None or driver_message is None:
                raise FeedbackError(
                    f"motor/driver feedback for joint {joint_index} is not available"
                )
            motor = motor_message.msg
            drive = driver_message.msg
            foc_status = drive.foc_status
            # 将 SDK 字段整理成带单位的普通字典，供 CLI 直接输出 JSON。
            diagnostics.append(
                {
                    "joint": joint_index,
                    "motor_position_rad": float(motor.position),
                    "motor_velocity_rad_s": float(motor.velocity),
                    "motor_current_a": float(motor.current),
                    "motor_torque_nm": float(motor.torque),
                    "driver_voltage_v": float(drive.vol),
                    "driver_temperature_c": float(drive.foc_temp),
                    "motor_temperature_c": float(drive.motor_temp),
                    "bus_current_a": float(drive.bus_current),
                    "driver_status": {
                        name: bool(getattr(foc_status, name)) for name in status_names
                    },
                    "motor_feedback_hz": float(motor_message.hz),
                    "driver_feedback_hz": float(driver_message.hz),
                }
            )
        return diagnostics

    def enable(self) -> None:
        """确认初始状态安全后，使能全部六个关节"""

        # 必须先收到完整的新鲜反馈，防止在未知状态下直接使能。
        state = self.wait_for_initial_state()
        self._require_safe_state(state)
        # 电机反馈是异步的，因此重复发送使能请求直到控制器确认或超时。
        if not self._confirm_motor_command(self._arm.enable, "enable"):
            raise ArmStateError("PiPER-X did not confirm that all joints are enabled")
        self._enabled_by_driver = True

    def wait_for_initial_state(self, timeout: float = 5.0) -> PiperXState:
        """等待连接后的第一份完整且未过期的状态反馈"""
        if timeout <= 0:
            raise ValueError("initial state timeout must be positive")
        self._require_connected()
        # 在截止时间前轮询；暂时缺少反馈属于启动阶段的正常情况。
        deadline = self._clock() + timeout
        last_error: FeedbackError | None = None
        while self._clock() < deadline:
            try:
                return self.get_state()
            except FeedbackError as exc:
                last_error = exc
                time.sleep(self._motor_confirmation_interval)
        # 超时后附带最后一次反馈错误，帮助区分无消息和消息格式问题。
        detail = "feedback did not become available"
        if last_error is not None:
            detail = str(last_error)
        raise FeedbackError(
            f"timed out after {timeout:.1f}s waiting for initial PiPER-X state: "
            f"{detail}"
        )

    def disable(self) -> None:
        """请求失能全部关节，并等待控制器反馈确认"""

        # 先清除本地使能标志，确保后续运动命令立即被驱动拒绝。
        self._require_connected()
        self._enabled_by_driver = False
        if not self._confirm_motor_command(self._arm.disable, "disable"):
            raise ArmStateError("PiPER-X did not confirm that all joints are disabled")

    def reset(self) -> None:
        """复位控制器运动状态，但不发送任何关节位置目标。"""
        self._require_connected()
        # 复位后原有使能状态不再可信，先阻止本驱动继续发送运动命令。
        self._enabled_by_driver = False
        try:
            self._arm.reset()
        except Exception as exc:
            raise CommunicationError(
                f"failed to send PiPER-X motion-controller reset: {exc}"
            ) from exc

    def emergency_stop(self) -> None:
        """请求 SDK 执行带阻尼的电子急停。"""
        self._require_connected()
        # 急停后立即清除本地使能标志，必须重新使能才能恢复运动。
        self._enabled_by_driver = False
        try:
            self._arm.electronic_emergency_stop()
        except Exception as exc:
            raise CommunicationError(
                f"failed to send PiPER-X emergency stop: {exc}"
            ) from exc

    def clear_errors(self, joint_index: int = 255) -> None:
        """清除指定关节错误；``255`` 表示同时处理全部关节"""
        self._require_connected()
        # 只接受 J1～J6 或 SDK 约定的广播编号 255。
        if joint_index not in (*range(1, 7), 255):
            raise ValueError("joint index must be in [1, 6] or 255")
        if not self._arm.clear_joint_error(joint_index=joint_index):
            raise ArmStateError(f"joint {joint_index} did not confirm error clear")

    def restore_default_motion_limits(self) -> None:
        """恢复固件默认的关节角度、速度和加速度限制"""
        self._require_connected()
        # SDK 返回 False 代表固件没有确认写入，不能把它当作成功。
        if not self._arm.set_joint_angle_vel_acc_limits_to_default():
            raise ArmStateError("firmware did not confirm default joint limits")

    def set_payload(self, payload: str) -> None:
        """设置控制器用于动力学补偿的负载档位"""

        self._require_connected()
        # 固件只接受 empty、half、full 三个预定义档位。
        if payload not in ("empty", "half", "full"):
            raise ValueError("payload must be empty, half, or full")
        if not self._arm.set_payload(payload=payload):
            raise ArmStateError(f"firmware did not confirm {payload} payload")

    def set_installation_position(self, position: str) -> None:
        """设置机械臂底座的安装方向"""

        self._require_connected()
        # 安装方向影响重力补偿，必须限制为 SDK 支持的三种取值。
        if position not in ("horizontal", "left", "right"):
            raise ValueError("installation position must be horizontal, left, or right")
        self._arm.set_installation_pos(position)

    def get_motion_limits(self, timeout: float = 1.0) -> dict[str, Any]:
        """通过官方 SDK 读取关节及末端的运动限制"""
        self._require_connected()
        joints: list[dict[str, float]] = []
        # 每个关节分别查询角度/速度限制与加速度限制，再合并为一项。
        for joint_index in range(1, 7):
            angle_speed = self._arm.get_joint_angle_vel_limits(
                joint_index, timeout=timeout
            )
            acceleration = self._arm.get_joint_acc_limits(joint_index, timeout=timeout)
            if angle_speed is None or acceleration is None:
                raise FeedbackError(f"joint {joint_index} limits are unavailable")
            joints.append(
                {
                    "joint": joint_index,
                    "min_angle_rad": float(angle_speed.msg.min_angle_limit),
                    "max_angle_rad": float(angle_speed.msg.max_angle_limit),
                    "max_speed_rad_s": float(angle_speed.msg.max_joint_spd),
                    "max_acceleration_rad_s2": float(acceleration.msg.max_joint_acc),
                }
            )
        # 末端限制反馈是可选的；没有反馈时保留 None，而不是伪造默认值。
        flange = self._arm.get_flange_vel_acc_limits(timeout=timeout)
        flange_values = None
        if flange is not None:
            flange_values = {
                "max_linear_velocity_m_s": float(flange.msg.end_max_linear_vel),
                "max_angular_velocity_rad_s": float(flange.msg.end_max_angular_vel),
                "max_linear_acceleration_m_s2": float(flange.msg.end_max_linear_acc),
                "max_angular_acceleration_rad_s2": float(
                    flange.msg.end_max_angular_acc
                ),
            }
        return {"joints": joints, "flange": flange_values}

    def get_ratings(self, timeout: float = 1.0) -> dict[str, list[int]]:
        """读取六轴碰撞保护等级和关节助力等级"""

        self._require_connected()
        # 两类反馈必须同时存在，才能返回含义完整的等级配置。
        protection = self._arm.get_crash_protection_rating(timeout=timeout)
        assistance = self._arm.get_joint_assistance_rating(timeout=timeout)
        if protection is None or assistance is None:
            raise FeedbackError("protection/assistance ratings are unavailable")
        return {
            "crash_protection": [int(value) for value in protection.msg],
            "joint_assistance": [int(value) for value in assistance.msg],
        }

    def set_rating(self, kind: str, rating: int, joint_index: int = 255) -> None:
        """设置单轴或全部关节的碰撞保护/助力等级"""

        self._require_connected()
        # joint_index=255 是固件定义的全部关节广播地址。
        if joint_index not in (*range(1, 7), 255):
            raise ValueError("joint index must be in [1, 6] or 255")
        # 两种等级使用不同的合法范围和 SDK 写入接口。
        if kind == "protect":
            if not 0 <= rating <= 8:
                raise ValueError("protection rating must be in [0, 8]")
            ok = self._arm.set_crash_protection_rating(
                joint_index=joint_index, rating=rating
            )
        elif kind == "assist":
            if not 0 <= rating <= 10:
                raise ValueError("assistance rating must be in [0, 10]")
            ok = self._arm.set_joint_assistance_rating(
                joint_index=joint_index, rating=rating
            )
        else:
            raise ValueError("rating kind must be protect or assist")
        if not ok:
            raise ArmStateError(f"firmware did not confirm {kind} rating")

    def command_pose(self, pose: Sequence[float], *, mode: str = "p") -> None:
        """使用 P 或 L 模式发送包含六个值的末端位姿"""
        # 只有经过本驱动成功使能后才允许发送运动指令。
        if not self._enabled_by_driver:
            raise NotEnabledError("enable() must succeed before commanding motion")
        # 将输入统一转换为 float，并验证六个值都可用于数值计算。
        values = tuple(float(value) for value in pose)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("pose must contain six finite values")
        # 发送前再次确认机械臂无错误，再根据 mode 调用对应 SDK 轨迹接口。
        self._require_safe_state(self.get_state())
        if mode == "p":
            self._arm.move_p(list(values))
        elif mode == "l":
            self._arm.move_l(list(values))
        else:
            raise ValueError("pose mode must be p or l")

    def command_circle(
        self,
        start_pose: Sequence[float],
        mid_pose: Sequence[float],
        end_pose: Sequence[float],
    ) -> None:
        """发送由起点、途经点和终点定义的末端圆弧轨迹"""

        # 圆弧也是运动命令，因此要求本驱动已成功使能机械臂。
        if not self._enabled_by_driver:
            raise NotEnabledError("enable() must succeed before commanding motion")
        # 将三个输入位姿转换为统一、不可变的浮点元组，方便一次性校验。
        poses = [
            tuple(float(value) for value in pose)
            for pose in (
                start_pose,
                mid_pose,
                end_pose,
            )
        ]
        if any(
            len(pose) != 6 or not all(math.isfinite(value) for value in pose)
            for pose in poses
        ):
            raise ValueError("circle poses must each contain six finite values")
        # 状态安全且三个位姿有效后，才把圆弧轨迹交给 SDK。
        self._require_safe_state(self.get_state())
        self._arm.move_c(*[list(pose) for pose in poses])

    def get_receive_fps(self) -> float:
        """返回 SDK 当前统计的 CAN 反馈接收频率"""

        self._require_connected()
        # 转为普通 float，便于 JSON 序列化和测试比较。
        return float(self._arm.get_fps())

    def _initialize_gripper(self) -> Any:
        """在 CAN 读取线程启动前注册 AGX 夹爪反馈解析器。"""

        # 重复调用 init_effector() 会被 pyAgxArm 拒绝，所以仅初始化一次。
        if self._gripper is None:
            self._gripper = self._arm.init_effector(
                self._arm.OPTIONS.EFFECTOR.AGX_GRIPPER
            )
        return self._gripper

    def _get_gripper(self) -> Any:
        """返回连接时初始化并缓存的 AGX 夹爪对象。"""

        self._require_connected()
        return self._initialize_gripper()

    def get_gripper_status(self) -> dict[str, Any]:
        """读取可选 AGX 夹爪的位置、力和驱动状态"""

        # 获取本驱动缓存的 AGX_GRIPPER 末端对象，再请求其最新反馈。
        gripper = self._get_gripper()
        status = gripper.get_gripper_status()
        if status is None:
            raise FeedbackError("gripper feedback is unavailable")
        # 将嵌套的 FOC 状态展开为 CLI 容易理解和序列化的字段。
        foc = status.msg.foc_status
        return {
            "value": float(status.msg.value),
            "force_n": float(status.msg.force),
            "mode": str(status.msg.mode),
            "enabled": bool(foc.driver_enable_status),
            "homed": bool(foc.homing_status),
            "error": bool(foc.driver_error_status),
            "receive_hz": float(status.hz),
        }

    def command_gripper(self, width_m: float, force_n: float = 1.0) -> None:
        """按目标开口宽度和夹持力控制 AGX 夹爪"""

        self._require_connected()
        # 宽度和力必须是非负有限值，避免向夹爪发送无意义数据。
        if not math.isfinite(width_m) or width_m < 0:
            raise ValueError("gripper width must be a non-negative finite value")
        if not math.isfinite(force_n) or force_n < 0:
            raise ValueError("gripper force must be a non-negative finite value")
        # 参数通过校验后，使用缓存的夹爪对象发送以米、牛顿为单位的命令。
        gripper = self._get_gripper()
        gripper.move_gripper_m(width_m, force_n)

    def reset_gripper(self) -> None:
        """复位 AGX 夹爪，并确认夹爪接受了该请求"""

        self._require_connected()
        # False 表示夹爪没有确认复位，不能静默忽略。
        gripper = self._get_gripper()
        if not gripper.reset_gripper():
            raise ArmStateError("gripper did not confirm reset")

    def calibrate_gripper_zero(self, timeout: float = 1.0) -> None:
        """将 AGX 夹爪当前位置标定为零点并等待确认"""

        self._require_connected()
        # 标定会写入夹爪设置，因此必须等待 timeout 内的明确反馈。
        gripper = self._get_gripper()
        if not gripper.calibrate_gripper(timeout=timeout):
            raise ArmStateError("gripper did not confirm zero calibration")

    def set_speed_percent(self, percent: int) -> None:
        """验证百分比后设置规划运动速度"""
        self._require_connected()
        # bool 是 int 的子类，但不能作为速度百分比，因此需要显式排除。
        if isinstance(percent, bool) or not isinstance(percent, int):
            raise TypeError("speed percent must be an integer")
        if not 1 <= percent <= 100:
            raise ValueError("speed percent must be in [1, 100]")
        # 将底层 SDK 异常包装成统一的项目异常
        # 便于 CLI 捕获
        try:
            self._arm.set_speed_percent(percent)
        except Exception as exc:
            raise PiperXError(f"failed to set PiPER-X speed: {exc}") from exc

    def maximize_joint_motion_limits(
        self, *, require_confirmation: bool = True
    ) -> bool:
        """将 J1～J6 的速度和加速度设为固件允许的最大值。

        新版固件会确认写入并支持读回；部分旧版固件不提供 pyAgxArm 使用的
        限值读回，此时接受匹配的 CAN ``0x476`` 应答。调用者可通过
        ``require_confirmation`` 决定缺少确认是否应当报错。
        """
        self._require_connected()
        # 非强制模式下，若固件明确不支持反馈，则跳过写入以免大量无效请求。
        if not require_confirmation and not self._supports_motion_limit_feedback():
            return False
        all_confirmed = True
        # 六个关节分别写入速度和加速度，并独立检查两项确认。
        for joint_index in range(1, 7):
            try:
                speed_ok = bool(
                    self._arm.set_joint_angle_vel_limits(
                        joint_index=joint_index,
                        max_joint_spd=PIPER_X_MAX_JOINT_SPEED_RAD_S,
                    )
                )
                # 新固件直接返回成功；旧固件可通过缓存的 0x476 应答确认。
                speed_confirmed = speed_ok or self._motion_limit_write_acknowledged(
                    0x74
                )
                acceleration_ok = bool(
                    self._arm.set_joint_acc_limits(
                        joint_index=joint_index,
                        max_joint_acc=PIPER_X_MAX_JOINT_ACCELERATION_RAD_S2,
                    )
                )
                acceleration_confirmed = (
                    acceleration_ok or self._motion_limit_write_acknowledged(0x75)
                )
            except Exception as exc:
                raise PiperXError(
                    f"failed to maximize joint {joint_index} motion limits: {exc}"
                ) from exc
            # 强制确认时立即报告具体关节和项目；否则记录总体未完全确认。
            if not speed_confirmed:
                all_confirmed = False
                if require_confirmation:
                    raise ArmStateError(
                        f"joint {joint_index} did not confirm maximum speed "
                        f"{PIPER_X_MAX_JOINT_SPEED_RAD_S:.1f}rad/s"
                    )
            if not acceleration_confirmed:
                all_confirmed = False
                if require_confirmation:
                    raise ArmStateError(
                        f"joint {joint_index} did not confirm maximum acceleration "
                        f"{PIPER_X_MAX_JOINT_ACCELERATION_RAD_S2:.1f}rad/s^2"
                    )
        return all_confirmed

    def _supports_motion_limit_feedback(self) -> bool:
        """判断固件是否支持运动限值写入反馈，避免发送无效请求"""
        # 固件信息必须包含符合 S-V主.次-修订 格式的软件版本字符串。
        firmware = self._arm.get_firmware(timeout=1.0)
        if not isinstance(firmware, dict):
            return False
        software_version = firmware.get("software_version")
        if not isinstance(software_version, str):
            return False
        match = re.fullmatch(r"S-V(\d+)\.(\d+)-(\d+)", software_version)
        if match is None:
            return False
        # 将版本各段转成整数元组，以便进行可靠的版本大小比较。
        version = tuple(int(part) for part in match.groups())
        return version >= (1, 8, 3)

    def _motion_limit_write_acknowledged(self, instruction_index: int) -> bool:
        """检查 pyAgxArm 缓存的 ``0x476`` 应答以兼容旧固件"""
        # 该检查器是 SDK 的可选兼容接口，不存在时只能视为未确认。
        checker = getattr(self._arm, "_is_resp_set_instruction", None)
        if not callable(checker):
            return False
        return bool(checker(instruction_index))

    def command_joints(
        self, joints_rad: Sequence[float], *, fast_response: bool = False
    ) -> None:
        """验证六轴目标后，通过 ``move_j`` 或 ``move_js`` 发送运动指令"""

        # 未经本驱动使能时禁止运动，防止绕过 enable() 中的安全检查。
        if not self._enabled_by_driver:
            raise NotEnabledError("enable() must succeed before commanding motion")
        # 同时检查目标范围、当前状态和六个关节的实际使能反馈。
        target = self._validate_target(joints_rad)
        state = self.get_state()
        self._require_safe_state(state)
        if not all(state.enabled_joints):
            raise ArmStateError("not all PiPER-X joints report enabled")

        # 计算每轴从当前位置到目标的绝对变化量，限制单条指令的最大跨度。
        deltas = tuple(
            abs(target_value - current_value)
            for target_value, current_value in zip(
                target, state.joint_positions_rad, strict=True
            )
        )
        largest_delta = max(deltas)
        if largest_delta > self._max_command_delta_rad:
            index = deltas.index(largest_delta) + 1
            raise JointCommandError(
                f"joint {index} delta {largest_delta:.6f}rad exceeds "
                f"{self._max_command_delta_rad:.6f}rad"
            )
        # fast_response 使用高响应但无平滑规划的 move_js，普通运动使用 move_j。
        try:
            if fast_response:
                self._arm.move_js(list(target))
            else:
                if self._fast_response_mode_active:
                    self.end_fast_response_mode()
                self._arm.move_j(list(target))
        except Exception as exc:
            raise PiperXError(f"failed to command PiPER-X joints: {exc}") from exc

    def begin_fast_response_mode(self) -> None:
        """进入只设置一次 JS/MIT 模式的高速目标流会话。"""

        self._require_connected()
        if not self._enabled_by_driver:
            raise NotEnabledError("enable() must succeed before entering JS mode")
        if self._fast_response_mode_active:
            return
        try:
            previous = bool(self._arm.get_auto_set_motion_mode_enabled())
            self._arm.set_motion_mode("js")
            self._arm.set_auto_set_motion_mode_enabled(False)
        except Exception as exc:
            raise PiperXError(f"failed to enter PiPER-X JS stream mode: {exc}") from exc
        self._previous_auto_motion_mode = previous
        self._fast_response_mode_active = True

    def end_fast_response_mode(self) -> None:
        """结束 JS/MIT 目标流并恢复 SDK 原来的自动模式设置。"""

        if not self._fast_response_mode_active:
            return
        previous = self._previous_auto_motion_mode
        try:
            self._arm.set_auto_set_motion_mode_enabled(
                True if previous is None else previous
            )
        except Exception as exc:
            raise PiperXError(f"failed to leave PiPER-X JS stream mode: {exc}") from exc
        finally:
            self._fast_response_mode_active = False
            self._previous_auto_motion_mode = None

    def wait_for_joints(
        self,
        joints_rad: Sequence[float],
        *,
        joint_indices: Sequence[int] | None = None,
        timeout: float = 3.0,
        tolerance_rad: float = math.radians(0.1),
        poll_interval: float = 0.02,
        refresh_command: Callable[[], None] | None = None,
    ) -> PiperXState:
        """持续读取 CAN 反馈，直到指定关节在容差内到达目标"""
        # 目标和轮询参数先全部校验，避免进入无法正确终止的等待循环。
        target = self._validate_target(joints_rad)
        if timeout <= 0:
            raise ValueError("joint wait timeout must be positive")
        if tolerance_rad <= 0:
            raise ValueError("joint wait tolerance must be positive")
        if poll_interval <= 0:
            raise ValueError("joint wait poll interval must be positive")
        if refresh_command is not None and not callable(refresh_command):
            raise TypeError("joint refresh command must be callable")
        # 未指定 joint_indices 时检查全部六轴，否则只检查用户选择的轴。
        indices = tuple(range(6)) if joint_indices is None else tuple(joint_indices)
        if not indices or any(
            isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < 6
            for index in indices
        ):
            raise ValueError("joint indices must contain zero-based indices in [0, 5]")

        # monotonic() 不受系统时间调整影响，适合计算稳定的超时时间。
        deadline = time.monotonic() + timeout
        while True:
            state = self.get_state()
            # 运动过程中若机械臂报错或任一关节失能，应立即报告真实原因，
            # 不再一直等待到 timeout 后才显示“未到达目标”。
            self._require_safe_state(state)
            if not all(state.enabled_joints):
                disabled = ", ".join(
                    str(index + 1)
                    for index, enabled in enumerate(state.enabled_joints)
                    if not enabled
                )
                raise ArmStateError(
                    f"joint(s) {disabled} became disabled while waiting for target"
                )
            # 找出误差最大的关节；全部误差小于 tolerance_rad 才算到位。
            errors = tuple(
                (
                    index,
                    abs(target[index] - state.joint_positions_rad[index]),
                )
                for index in indices
            )
            worst_index, worst_error = max(errors, key=lambda item: item[1])
            if worst_error <= tolerance_rad:
                return state
            # 到达截止时间仍未满足容差时，报告最差关节及其剩余误差。
            if time.monotonic() >= deadline:
                raise ArmStateError(
                    f"joint {worst_index + 1} did not reach its target within "
                    f"{timeout:.1f}s (remaining error "
                    f"{math.degrees(worst_error):.3f}deg)"
                )
            # move_js 属于流式 Follower 控制，需要周期性重发同一目标；普通
            # move_j 不传回调，仍由固件内部规划并持续执行。
            if refresh_command is not None:
                refresh_command()
            time.sleep(poll_interval)

    def _validate_target(self, joints_rad: Sequence[float]) -> tuple[float, ...]:
        """将关节目标转换为六个浮点数并检查各轴角度范围"""

        # 先统一数值类型，无法转换的输入作为关节命令错误报告。
        try:
            target = tuple(float(value) for value in joints_rad)
        except (TypeError, ValueError) as exc:
            raise JointCommandError("joint command must be a numeric sequence") from exc
        if len(target) != 6 or not all(math.isfinite(value) for value in target):
            raise JointCommandError("joint command must contain 6 finite values")
        # 标称限制外加入小幅标定余量，兼容真实零点的轻微反馈偏差。
        for index, (value, limits) in enumerate(
            zip(target, PIPER_X_JOINT_LIMITS_RAD, strict=True), start=1
        ):
            lower = limits[0] - self._joint_limit_margin_rad
            upper = limits[1] + self._joint_limit_margin_rad
            if not lower <= value <= upper:
                raise JointCommandError(
                    f"joint {index} target {value:.6f}rad is outside "
                    f"[{lower:.6f}, {upper:.6f}]rad"
                )
        return target

    @staticmethod
    def _require_safe_state(state: PiperXState) -> None:
        """确认控制器状态正常且没有关节限位或通信错误"""

        # arm_status 非零或任意错误位有效时都禁止继续发送运动指令。
        if state.arm_status != 0:
            raise ArmStateError(f"PiPER-X arm status is {state.arm_status}, expected 0")
        if state.errors.any:
            raise ArmStateError("PiPER-X reports a joint limit or communication error")

    def _require_connected(self) -> None:
        """确认驱动已经完成连接，否则立即抛出通信异常"""

        # 所有依赖 SDK 反馈或写入的公开方法都通过此处统一保护。
        if not self._connected:
            raise CommunicationError("connect() must be called first")

    def _confirm_motor_command(
        self, command: Callable[[int], object], action: str
    ) -> bool:
        """重复发送电机命令，直到异步反馈确认或等待超时"""
        # 使能/失能反馈存在延迟，因此在固定截止时间前允许重复请求。
        deadline = time.monotonic() + self._motor_confirmation_timeout
        while True:
            try:
                if bool(command(255)):
                    return True
            except Exception as exc:
                raise PiperXError(f"failed to {action} PiPER-X: {exc}") from exc
            # 超时返回 False，由调用者决定需要抛出哪一种状态异常。
            if time.monotonic() >= deadline:
                return False
            time.sleep(self._motor_confirmation_interval)
