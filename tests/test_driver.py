from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from agilexrobotics.driver import PiperXDriver
from agilexrobotics.exceptions import (
    ArmStateError,
    CommunicationError,
    FeedbackError,
    JointCommandError,
    NotEnabledError,
)


@dataclass
class FakeMessage:
    msg: object
    timestamp: float = 100.0
    hz: float = 200.0


@dataclass
class FakeErrors:
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


@dataclass
class FakeStatus:
    ctrl_mode: int = 1
    arm_status: int = 0
    motion_status: int = 1
    err_status: FakeErrors | None = None

    def __post_init__(self) -> None:
        if self.err_status is None:
            self.err_status = FakeErrors()


class FakeGripper:
    """模拟只允许初始化一次的 AGX 夹爪。"""

    def __init__(self) -> None:
        """准备夹爪状态和命令记录。"""

        foc_status = SimpleNamespace(
            driver_enable_status=True,
            homing_status=True,
            driver_error_status=False,
        )
        self.status = FakeMessage(
            SimpleNamespace(
                value=0.07, force=1.0, mode="width", foc_status=foc_status
            )
        )
        self.move_commands: list[tuple[float, float]] = []
        self.calibration_timeouts: list[float] = []
        self.reset_calls = 0

    def get_gripper_status(self) -> FakeMessage:
        """返回固定的夹爪反馈。"""

        return self.status

    def move_gripper_m(self, width_m: float, force_n: float) -> None:
        """记录夹爪运动命令。"""

        self.move_commands.append((width_m, force_n))

    def reset_gripper(self) -> bool:
        """记录复位请求并模拟确认成功。"""

        self.reset_calls += 1
        return True

    def calibrate_gripper(self, *, timeout: float) -> bool:
        """记录零点标定超时并模拟确认成功。"""

        self.calibration_timeouts.append(timeout)
        return True


class FakeOptions:
    """提供测试所需的 SDK 末端执行器枚举。"""

    EFFECTOR = SimpleNamespace(AGX_GRIPPER="agx_gripper")


class FakeSdkArm:
    def __init__(self) -> None:
        self.connected = False
        self.comm_ok = True
        self.comm_error: Exception | None = None
        self.joints = [0.0] * 6
        self.status = FakeStatus()
        self.enabled = [False] * 6
        self.enable_result = True
        self.enable_results: list[bool] = []
        self.disable_result = True
        self.speed_percent: int | None = None
        self.commands: list[list[float]] = []
        self.fast_commands: list[list[float]] = []
        self.motion_modes: list[str] = []
        self.auto_motion_mode = True
        self.speed_limit_commands: list[tuple[int, float]] = []
        self.acceleration_limit_commands: list[tuple[int, float]] = []
        self.motion_limit_result = True
        self.acknowledged_instruction_indexes: set[int] = set()
        self.firmware = {"software_version": "S-V1.8-9"}
        self.reset_called = False
        self.OPTIONS = FakeOptions()
        self.gripper = FakeGripper()
        self.init_effector_calls = 0

    def init_effector(self, effector: str) -> FakeGripper:
        """模拟 SDK 对重复末端初始化的拒绝行为。"""

        assert effector == "agx_gripper"
        self.init_effector_calls += 1
        if self.init_effector_calls > 1:
            raise RuntimeError("effector already initialized: agx_gripper")
        return self.gripper

    def connect(self, *, start_read_thread: bool) -> None:
        assert start_read_thread
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def has_comm_error(self) -> bool:
        return self.comm_error is not None

    def get_comm_error(self):
        return self.comm_error

    def is_ok(self) -> bool:
        return self.comm_ok

    def get_firmware(self, *, timeout: float):
        assert timeout == 1.0
        return self.firmware

    def get_joint_angles(self):
        return FakeMessage(self.joints)

    def get_arm_status(self):
        return FakeMessage(self.status)

    def get_flange_pose(self):
        return FakeMessage([0.0] * 6)

    def get_joints_enable_status_list(self):
        return self.enabled

    def enable(self, joint_index: int) -> bool:
        assert joint_index == 255
        result = (
            self.enable_results.pop(0) if self.enable_results else self.enable_result
        )
        if result:
            self.enabled = [True] * 6
        return result

    def disable(self, joint_index: int) -> bool:
        assert joint_index == 255
        if self.disable_result:
            self.enabled = [False] * 6
        return self.disable_result

    def reset(self) -> None:
        self.reset_called = True

    def move_j(self, joints: list[float]) -> None:
        self.commands.append(joints)
        self.joints = list(joints)

    def move_js(self, joints: list[float]) -> None:
        self.fast_commands.append(joints)
        self.joints = list(joints)

    def get_auto_set_motion_mode_enabled(self) -> bool:
        return self.auto_motion_mode

    def set_auto_set_motion_mode_enabled(self, enabled: bool) -> None:
        self.auto_motion_mode = enabled

    def set_motion_mode(self, mode: str) -> None:
        self.motion_modes.append(mode)

    def set_speed_percent(self, percent: int) -> None:
        self.speed_percent = percent

    def set_joint_angle_vel_limits(
        self, *, joint_index: int, max_joint_spd: float
    ) -> bool:
        self.speed_limit_commands.append((joint_index, max_joint_spd))
        return self.motion_limit_result

    def set_joint_acc_limits(self, *, joint_index: int, max_joint_acc: float) -> bool:
        self.acceleration_limit_commands.append((joint_index, max_joint_acc))
        return self.motion_limit_result

    def _is_resp_set_instruction(self, instruction_index: int) -> bool:
        return instruction_index in self.acknowledged_instruction_indexes


def connected_driver(**kwargs) -> tuple[PiperXDriver, FakeSdkArm]:
    sdk = FakeSdkArm()
    driver = PiperXDriver(sdk_arm=sdk, clock=lambda: 100.1, **kwargs)
    driver.connect()
    return driver, sdk


def test_get_state_returns_structured_fresh_feedback() -> None:
    driver, _ = connected_driver()

    state = driver.get_state()

    assert state.joint_positions_rad == (0.0,) * 6
    assert state.feedback_age == pytest.approx(0.1)
    assert state.arm_status == 0
    assert state.errors.any is False


def test_state_requires_connection_and_fresh_feedback() -> None:
    sdk = FakeSdkArm()
    driver = PiperXDriver(sdk_arm=sdk, clock=lambda: 101.0)
    with pytest.raises(CommunicationError):
        driver.get_state()

    driver.connect()
    with pytest.raises(FeedbackError, match="stale"):
        driver.get_state()


def test_communication_failure_blocks_state() -> None:
    driver, sdk = connected_driver()
    sdk.comm_error = RuntimeError("CAN receive failed")

    with pytest.raises(CommunicationError):
        driver.get_state()


def test_sdk_fps_monitor_startup_race_does_not_block_fresh_state() -> None:
    driver, sdk = connected_driver()
    sdk.comm_ok = False

    state = driver.get_state()

    assert state.communication_ok is True


def test_wait_for_initial_state_retries_missing_feedback(monkeypatch) -> None:
    driver, sdk = connected_driver()
    original_get_joint_angles = sdk.get_joint_angles
    attempts = 0

    def delayed_joint_angles():
        nonlocal attempts
        attempts += 1
        if attempts < 3:
            return None
        return original_get_joint_angles()

    monkeypatch.setattr(sdk, "get_joint_angles", delayed_joint_angles)

    state = driver.wait_for_initial_state(timeout=1.0)

    assert attempts == 3
    assert state.joint_positions_rad == (0.0,) * 6


def test_enable_then_small_joint_command() -> None:
    driver, sdk = connected_driver()
    driver.enable()

    driver.command_joints([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert sdk.commands == [[0.01, 0.0, 0.0, 0.0, 0.0, 0.0]]


def test_reset_forwards_motion_controller_reset_without_feedback() -> None:
    driver, sdk = connected_driver()

    driver.reset()

    assert sdk.reset_called is True


def test_fast_joint_command_uses_sdk_joint_servo_mode() -> None:
    driver, sdk = connected_driver()
    driver.enable()

    driver.command_joints([0.01, 0.0, 0.0, 0.0, 0.0, 0.0], fast_response=True)

    assert sdk.fast_commands == [[0.01, 0.0, 0.0, 0.0, 0.0, 0.0]]
    assert sdk.commands == []


def test_maximize_joint_motion_limits_sets_and_verifies_all_joints() -> None:
    driver, sdk = connected_driver()

    driver.maximize_joint_motion_limits()

    assert sdk.speed_limit_commands == [(joint, 3.0) for joint in range(1, 7)]
    assert sdk.acceleration_limit_commands == [(joint, 5.0) for joint in range(1, 7)]


def test_maximize_joint_motion_limits_requires_firmware_confirmation() -> None:
    driver, sdk = connected_driver()
    sdk.motion_limit_result = False

    with pytest.raises(ArmStateError, match="did not confirm maximum speed"):
        driver.maximize_joint_motion_limits()


def test_maximize_joint_motion_limits_accepts_legacy_firmware_ack() -> None:
    driver, sdk = connected_driver()
    sdk.motion_limit_result = False
    sdk.acknowledged_instruction_indexes = {0x74, 0x75}

    driver.maximize_joint_motion_limits()

    assert len(sdk.speed_limit_commands) == 6
    assert len(sdk.acceleration_limit_commands) == 6


def test_unconfirmed_motion_limits_can_be_non_blocking() -> None:
    driver, sdk = connected_driver()
    sdk.motion_limit_result = False

    confirmed = driver.maximize_joint_motion_limits(require_confirmation=False)

    assert confirmed is False
    assert len(sdk.speed_limit_commands) == 6
    assert len(sdk.acceleration_limit_commands) == 6


def test_legacy_firmware_skips_unsupported_motion_limit_writes() -> None:
    driver, sdk = connected_driver()
    sdk.firmware = {"software_version": "S-V1.8-2"}

    confirmed = driver.maximize_joint_motion_limits(require_confirmation=False)

    assert confirmed is False
    assert sdk.speed_limit_commands == []
    assert sdk.acceleration_limit_commands == []


def test_speed_percent_is_validated_and_forwarded() -> None:
    driver, sdk = connected_driver()

    driver.set_speed_percent(5)

    assert sdk.speed_percent == 5
    with pytest.raises(ValueError):
        driver.set_speed_percent(0)
    with pytest.raises(ValueError):
        driver.set_speed_percent(101)


def test_wait_for_joints_returns_confirmed_state() -> None:
    driver, _ = connected_driver()
    driver.enable()
    target = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
    driver.command_joints(target)

    state = driver.wait_for_joints(target)

    assert state.joint_positions_rad == pytest.approx(target)


def test_wait_for_joints_can_check_only_the_commanded_joint() -> None:
    driver, sdk = connected_driver()
    driver.enable()
    target = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
    driver.command_joints(target)
    sdk.joints[1] = 0.5

    state = driver.wait_for_joints(target, joint_indices=(0,))

    assert state.joint_positions_rad[0] == pytest.approx(target[0])


def test_wait_for_joints_can_refresh_streaming_command() -> None:
    driver, sdk = connected_driver()
    driver.enable()
    target = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]

    def move_partway(joints: list[float]) -> None:
        sdk.fast_commands.append(joints)
        sdk.joints = [
            current + (goal - current) * 0.5
            for current, goal in zip(sdk.joints, joints, strict=True)
        ]

    sdk.move_js = move_partway
    driver.command_joints(target, fast_response=True)
    state = driver.wait_for_joints(
        target,
        tolerance_rad=0.001,
        poll_interval=0.001,
        refresh_command=lambda: driver.command_joints(target, fast_response=True),
    )

    assert state.joint_positions_rad == pytest.approx(target, abs=0.001)
    assert len(sdk.fast_commands) > 1


def test_fast_response_session_sets_motion_mode_only_once() -> None:
    driver, sdk = connected_driver()
    driver.enable()

    driver.begin_fast_response_mode()
    driver.command_joints([0.0] * 6, fast_response=True)
    driver.command_joints([0.0] * 6, fast_response=True)

    assert sdk.motion_modes == ["js"]
    assert sdk.auto_motion_mode is False
    driver.end_fast_response_mode()
    assert sdk.auto_motion_mode is True


def test_wait_for_joints_reports_mid_motion_disable_immediately() -> None:
    driver, sdk = connected_driver()
    driver.enable()
    target = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
    driver.command_joints(target)
    sdk.joints = [0.0] * 6
    sdk.enabled[3] = False

    with pytest.raises(
        ArmStateError, match=r"joint\(s\) 4 became disabled while waiting for target"
    ):
        driver.wait_for_joints(target, timeout=10.0)


def test_enable_retries_until_feedback_confirms() -> None:
    driver, sdk = connected_driver(
        motor_confirmation_timeout=0.1,
        motor_confirmation_interval=0.001,
    )
    sdk.enable_results = [False, False, True]

    driver.enable()

    assert sdk.enabled == [True] * 6


def test_command_requires_driver_enable() -> None:
    driver, _ = connected_driver()

    with pytest.raises(NotEnabledError):
        driver.command_joints([0.0] * 6)


@pytest.mark.parametrize(
    "target",
    (
        [0.0] * 5,
        [0.0, 0.0, 0.0, 0.0, 0.0, float("nan")],
        [4.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ),
)
def test_invalid_joint_commands_are_rejected(target: list[float]) -> None:
    driver, _ = connected_driver()
    driver.enable()

    with pytest.raises(JointCommandError):
        driver.command_joints(target)


def test_large_single_step_is_rejected() -> None:
    driver, _ = connected_driver(max_command_delta_rad=0.05)
    driver.enable()

    with pytest.raises(JointCommandError, match="delta"):
        driver.command_joints([0.051, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_arm_error_blocks_enable_and_motion() -> None:
    driver, sdk = connected_driver()
    assert sdk.status.err_status is not None
    sdk.status.err_status.joint_2_angle_limit = True

    with pytest.raises(ArmStateError):
        driver.enable()

    sdk.status.err_status.joint_2_angle_limit = False
    driver.enable()
    sdk.status.arm_status = 7
    with pytest.raises(ArmStateError):
        driver.command_joints([0.0] * 6)


def test_disable_revokes_motion_permission() -> None:
    driver, _ = connected_driver()
    driver.enable()
    driver.disable()

    with pytest.raises(NotEnabledError):
        driver.command_joints([0.0] * 6)


def test_close_is_idempotent() -> None:
    driver, sdk = connected_driver()

    driver.close()
    driver.close()

    assert sdk.connected is False


def test_gripper_effector_is_initialized_once_and_reused() -> None:
    """读取与控制夹爪时应复用同一个 SDK 末端对象。"""

    driver, sdk = connected_driver()

    status = driver.get_gripper_status()
    driver.command_gripper(0.035, 2.0)
    driver.reset_gripper()
    driver.calibrate_gripper_zero(timeout=0.5)

    assert status["value"] == pytest.approx(0.07)
    assert sdk.init_effector_calls == 1
    assert sdk.gripper.move_commands == [(0.035, 2.0)]
    assert sdk.gripper.reset_calls == 1
    assert sdk.gripper.calibration_timeouts == [0.5]
