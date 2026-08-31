from __future__ import annotations

import numpy as np
import pytest

from agilexrobotics.driver import ArmErrorFlags, PiperXState
from agilexrobotics.exceptions import FeedbackError
from agilexrobotics.gello_robot import GelloPiperXRobot
from agilexrobotics.gello_server import _dispatch, _parser


class FakeDriver:
    def __init__(self) -> None:
        self.connected = False
        self.enabled = False
        self.closed = False
        self.speed = None
        self.positions = [0.0] * 6
        self.commands: list[list[float]] = []
        self.fast_commands: list[list[float]] = []
        self.fast_response_mode = False
        self.timestamp = 10.0
        self.motion_limits_maximized = False
        self.gripper_width = 0.1
        self.gripper_feedback_available = True
        self.gripper_commands: list[tuple[float, float]] = []

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.closed = True

    def begin_fast_response_mode(self) -> None:
        self.fast_response_mode = True

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def set_speed_percent(self, percent: int) -> None:
        self.speed = percent

    def maximize_joint_motion_limits(
        self, *, require_confirmation: bool = True
    ) -> bool:
        assert require_confirmation is True
        self.motion_limits_maximized = True
        return True

    def command_joints(
        self,
        joints: list[float] | tuple[float, ...],
        *,
        fast_response: bool = False,
    ) -> None:
        self.positions = list(joints)
        self.commands.append(list(joints))
        if fast_response:
            self.fast_commands.append(list(joints))

    def get_gripper_status(self):
        if not self.gripper_feedback_available:
            raise FeedbackError("gripper feedback is unavailable")
        return {"value": self.gripper_width}

    def command_gripper(self, width_m: float, force_n: float = 1.0) -> None:
        self.gripper_width = width_m
        self.gripper_commands.append((width_m, force_n))

    def get_state(self) -> PiperXState:
        self.timestamp += 0.01
        return PiperXState(
            joint_positions_rad=tuple(self.positions),
            flange_pose_m_rad=(0.1, 0.2, 0.3, 0.0, 0.0, 0.0),
            control_mode=1,
            arm_status=0,
            motion_status=0,
            enabled_joints=(self.enabled,) * 6,
            errors=ArmErrorFlags(),
            feedback_timestamp=self.timestamp,
            feedback_age=0.001,
            receive_hz=200.0,
            communication_ok=True,
        )


def test_gello_server_hardware_confirmation_is_implicit() -> None:
    args = _parser().parse_args([])

    assert args.channel == "can0"
    assert args.port == 6001
    assert args.hz == 50.0
    assert args.gripper_max_width_m == 0.1
    assert args.gripper_force_n == 1.0
    assert args.configure_motion_limits is True
    assert (
        _parser().parse_args(["--no-configure-motion-limits"]).configure_motion_limits
        is False
    )
    assert args.confirm_hardware_control is False


def test_gello_server_rejects_removed_speed_percent_option() -> None:
    with pytest.raises(SystemExit) as exc:
        _parser().parse_args(["--speed-percent", "20"])

    assert exc.value.code == 2


def started_robot(monkeypatch) -> tuple[GelloPiperXRobot, FakeDriver]:
    monkeypatch.setattr("agilexrobotics.gello_robot.time.sleep", lambda _: None)
    driver = FakeDriver()
    robot = GelloPiperXRobot(driver=driver, startup_wait=0)
    robot.start()
    return robot, driver


def test_start_does_not_send_motion_or_speed_mode_commands(monkeypatch) -> None:
    robot, driver = started_robot(monkeypatch)

    assert robot.num_dofs() == 7
    assert driver.connected is True
    assert driver.enabled is True
    assert driver.motion_limits_maximized is True
    assert robot.motion_limits_confirmed is True
    assert driver.speed is None
    assert driver.fast_response_mode is False
    assert driver.commands == []


def test_missing_gripper_feedback_does_not_block_start(monkeypatch) -> None:
    monkeypatch.setattr("agilexrobotics.gello_robot.time.sleep", lambda _: None)
    driver = FakeDriver()
    driver.gripper_feedback_available = False
    robot = GelloPiperXRobot(driver=driver, startup_wait=0)

    robot.start()
    state = robot.get_joint_state()

    assert robot.num_dofs() == 7
    assert state[6] == pytest.approx(0.0)
    robot.command_joint_state(np.array([0.0] * 6 + [0.25]))
    assert driver.fast_response_mode is True
    assert driver.gripper_commands[-1] == pytest.approx((0.025, 1.0))
    assert robot.get_joint_state()[6] == pytest.approx(0.25)


def test_motion_limit_configuration_is_explicit(monkeypatch) -> None:
    monkeypatch.setattr("agilexrobotics.gello_robot.time.sleep", lambda _: None)
    driver = FakeDriver()
    robot = GelloPiperXRobot(
        driver=driver, startup_wait=0, configure_motion_limits=True
    )

    robot.start()

    assert driver.motion_limits_maximized is True
    assert robot.motion_limits_confirmed is True


def test_control_hz_must_be_positive_and_finite() -> None:
    driver = FakeDriver()

    with pytest.raises(ValueError, match="control hz"):
        GelloPiperXRobot(driver=driver, control_hz=0)
    with pytest.raises(ValueError, match="control hz"):
        GelloPiperXRobot(driver=driver, control_hz=float("nan"))


def test_command_dispatch_is_rate_limited(monkeypatch) -> None:
    times = iter((10.0, 10.005, 10.02))
    sleeps: list[float] = []
    monkeypatch.setattr(
        "agilexrobotics.gello_robot.time.monotonic", lambda: next(times)
    )
    monkeypatch.setattr("agilexrobotics.gello_robot.time.sleep", sleeps.append)
    driver = FakeDriver()
    robot = GelloPiperXRobot(driver=driver, startup_wait=0, control_hz=50)
    robot.start()
    sleeps.clear()

    target = np.array([0.0] * 6 + [0.5])
    robot.command_joint_state(target)
    robot.command_joint_state(target)

    assert sleeps == pytest.approx([0.015])
    assert len(driver.fast_commands) == 2


def test_command_maps_seventh_axis_to_direct_gripper_width(monkeypatch) -> None:
    robot, driver = started_robot(monkeypatch)
    target = np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.25])

    robot.command_joint_state(target)

    assert driver.fast_commands[-1] == pytest.approx(target[:6])
    assert driver.fast_response_mode is True
    assert driver.gripper_commands[-1] == pytest.approx((0.025, 1.0))

    target[6] = 1.0
    robot.command_joint_state(target)
    assert driver.gripper_commands[-1] == pytest.approx((0.1, 1.0))

    with pytest.raises(ValueError, match="6 arm joints and 1 gripper"):
        robot.command_joint_state(np.zeros(6))


def test_observations_match_gello_contract(monkeypatch) -> None:
    robot, _ = started_robot(monkeypatch)

    observations = robot.get_observations()

    assert observations["joint_positions"].shape == (7,)
    assert observations["joint_positions"][6] == pytest.approx(1.0)
    assert observations["joint_velocities"].shape == (7,)
    assert observations["ee_pos_quat"] == pytest.approx(
        [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
    )
    assert observations["gripper_position"] == pytest.approx(1.0)


def test_server_dispatch_uses_gello_method_names(monkeypatch) -> None:
    robot, driver = started_robot(monkeypatch)

    assert _dispatch(robot, {"method": "num_dofs"}) == 7
    _dispatch(
        robot,
        {
            "method": "command_joint_state",
            "args": {"joint_state": np.array([0.01] * 6 + [0.5])},
        },
    )
    assert driver.commands[-1] == pytest.approx([0.01] * 6)
    with pytest.raises(ValueError, match="unsupported"):
        _dispatch(robot, {"method": "delete_everything"})
