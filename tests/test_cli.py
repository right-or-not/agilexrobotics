import math
from typing import ClassVar

import pytest

from agilexrobotics import cli
from agilexrobotics.driver import ArmErrorFlags, PiperXState
from agilexrobotics.reader import PiperXSnapshot


class FakeConnection:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        return None

    def read_snapshot(self, *, firmware_timeout: float) -> PiperXSnapshot:
        return PiperXSnapshot(
            connected=True,
            communication_ok=True,
            communication_error=None,
            receive_fps=100.0,
            firmware={"software_version": "S-V1.8-9"},
            joint_angles_rad=[0.0] * 6,
            flange_pose_m_rad=[0.0] * 6,
            arm_status=None,
        )


def test_status_command_is_read_only(monkeypatch, capsys) -> None:
    monkeypatch.setattr(cli, "PiperXConnection", FakeConnection)

    result = cli.main(["status", "--wait", "0", "--timeout", "0"])

    assert result == 0
    assert '"connected": true' in capsys.readouterr().out


class FakeDriver:
    instances: ClassVar[list["FakeDriver"]] = []

    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.enabled = False
        self.closed = False
        self.speed = None
        self.commands = []
        self.joints = [0.0] * 6
        self.waited_indices = None
        self.reset_called = False
        self.motion_limits_maximized = False
        self.stopped = False
        self.cleared_joint = None
        self.defaults_restored = False
        self.payload = None
        self.installation = None
        self.fast_commands = []
        self.fast_response_mode = False
        self.pose_commands = []
        self.circle_command = None
        self.instances.append(self)

    def connect(self) -> None:
        return None

    def close(self) -> None:
        self.closed = True

    def get_state(self) -> PiperXState:
        return PiperXState(
            joint_positions_rad=tuple(self.joints),
            flange_pose_m_rad=(0.0,) * 6,
            control_mode=1,
            arm_status=0,
            motion_status=0,
            enabled_joints=(self.enabled,) * 6,
            errors=ArmErrorFlags(),
            feedback_timestamp=100.0,
            feedback_age=0.01,
            receive_hz=200.0,
            communication_ok=True,
        )

    def set_speed_percent(self, percent: int) -> None:
        self.speed = percent

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def reset(self) -> None:
        self.reset_called = True

    def maximize_joint_motion_limits(
        self, *, require_confirmation: bool = True
    ) -> bool:
        assert require_confirmation is True
        self.motion_limits_maximized = True
        return True

    def emergency_stop(self) -> None:
        self.stopped = True

    def clear_errors(self, joint_index: int = 255) -> None:
        self.cleared_joint = joint_index

    def restore_default_motion_limits(self) -> None:
        self.defaults_restored = True

    def set_payload(self, payload: str) -> None:
        self.payload = payload

    def set_installation_position(self, position: str) -> None:
        self.installation = position

    def get_receive_fps(self) -> float:
        return 200.0

    def get_motion_limits(self, timeout: float = 1.0):
        return {"joints": [], "flange": None}

    def get_ratings(self, timeout: float = 1.0):
        return {"crash_protection": [1] * 6, "joint_assistance": [2] * 6}

    def set_rating(self, kind: str, rating: int, joint_index: int = 255) -> None:
        self.rating = (kind, rating, joint_index)

    def command_joints(self, joints, *, fast_response: bool = False) -> None:
        self.commands.append(list(joints))
        if fast_response:
            self.fast_commands.append(list(joints))

    def begin_fast_response_mode(self) -> None:
        self.fast_response_mode = True

    def end_fast_response_mode(self) -> None:
        self.fast_response_mode = False

    def command_pose(self, pose, *, mode: str = "p") -> None:
        self.pose_commands.append((mode, list(pose)))

    def command_circle(self, start_pose, mid_pose, end_pose) -> None:
        self.circle_command = (list(start_pose), list(mid_pose), list(end_pose))

    def wait_for_joints(
        self, joints, *, joint_indices=None, timeout=None, refresh_command=None
    ) -> PiperXState:
        self.joints = list(joints)
        self.waited_indices = joint_indices
        return self.get_state()


def test_driver_status_does_not_require_confirmation(monkeypatch, capsys) -> None:
    monkeypatch.setattr(cli, "PiperXDriver", FakeDriver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    result = cli.main(["driver-status", "--wait", "0"])

    assert result == 0
    assert '"action": "read-only"' in capsys.readouterr().out
    assert FakeDriver.instances[-1].enabled is False
    assert FakeDriver.instances[-1].closed is True


def test_reset_uses_implicit_confirmation_and_does_not_require_feedback(
    monkeypatch, capsys
) -> None:
    driver = FakeDriver()
    monkeypatch.setattr(cli, "PiperXDriver", lambda **_: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    result = cli.main(["reset", "--wait", "0"])

    assert result == 0
    assert driver.reset_called is True
    assert driver.closed is True
    assert '"action": "motion-controller-reset-command-sent"' in (
        capsys.readouterr().out
    )


def test_max_limits_confirms_and_exits_without_starting_motion(
    monkeypatch, capsys
) -> None:
    driver = FakeDriver()
    monkeypatch.setattr(cli, "PiperXDriver", lambda **_: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    result = cli.main(["max-limits", "--wait", "0"])

    assert result == 0
    assert driver.motion_limits_maximized is True
    assert driver.enabled is False
    assert driver.commands == []
    assert driver.closed is True
    output = capsys.readouterr().out
    assert '"action": "firmware-motion-limits-confirmed"' in output
    assert '"max_joint_speed_rad_s": 3.0' in output
    assert '"max_joint_acceleration_rad_s2": 5.0' in output


def test_hold_accepts_explicit_lower_speed_and_legacy_confirmation_flag(
    monkeypatch, capsys
) -> None:
    monkeypatch.setattr(cli, "PiperXDriver", FakeDriver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    result = cli.main(
        ["hold", "--wait", "0", "--speed-percent", "5", "--confirm-hardware-test"]
    )

    driver = FakeDriver.instances[-1]
    assert result == 0
    assert driver.speed == 5
    assert driver.commands == [[0.0] * 6]
    assert driver.closed is True
    assert '"action": "holding-current-position"' in capsys.readouterr().out


def test_move_joint_is_restricted_to_ninety_degrees(monkeypatch) -> None:
    monkeypatch.setattr(cli, "PiperXDriver", FakeDriver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    with pytest.raises(SystemExit):
        cli.main(
            [
                "move-joint",
                "--joint",
                "1",
                "--delta-deg",
                "90.1",
                "--confirm-hardware-test",
            ]
        )


def test_move_joint_sends_one_target_and_waits_for_feedback(
    monkeypatch, capsys
) -> None:
    monkeypatch.setattr(cli, "PiperXDriver", FakeDriver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    result = cli.main(
        [
            "move-joint",
            "--wait",
            "0",
            "--joint",
            "1",
            "--delta-deg",
            "0.5",
            "--speed-percent",
            "5",
            "--confirm-hardware-test",
        ]
    )

    driver = FakeDriver.instances[-1]
    assert result == 0
    assert len(driver.commands) == 1
    assert driver.commands[0][0] == pytest.approx(math.radians(0.5))
    assert driver.waited_indices == (0,)
    assert '"action": "relative-joint-target-reached"' in capsys.readouterr().out


def test_zero_moves_all_joints_to_zero_in_ninety_degree_segments(
    monkeypatch, capsys
) -> None:
    driver = FakeDriver()
    driver.joints = [1.0, 0.5, -0.5, 0.2, -0.2, 0.1]
    monkeypatch.setattr(cli, "PiperXDriver", lambda **_: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    result = cli.main(
        [
            "zero",
            "--wait",
            "0",
        ]
    )

    assert result == 0
    assert len(driver.commands) == 1
    previous = [1.0, 0.5, -0.5, 0.2, -0.2, 0.1]
    for command in driver.commands:
        assert max(abs(a - b) for a, b in zip(command, previous, strict=True)) <= (
            math.radians(90) + 1e-12
        )
        previous = command
    assert driver.commands[-1] == pytest.approx([0.0] * 6)
    assert driver.waited_indices is None
    assert '"action": "zero-position-reached"' in capsys.readouterr().out


def test_zero_defaults_match_the_short_command(monkeypatch) -> None:
    driver = FakeDriver()
    monkeypatch.setattr(cli, "PiperXDriver", lambda **kwargs: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    assert cli.main(["zero"]) == 0
    assert driver.kwargs == {}
    assert driver.speed == 20
    assert driver.commands == [[0.0] * 6]


def test_hold_defaults_to_twenty_percent_without_confirmation_flag(
    monkeypatch, capsys
) -> None:
    driver = FakeDriver()
    monkeypatch.setattr(cli, "PiperXDriver", lambda **_: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    assert cli.main(["hold", "--wait", "0"]) == 0
    assert driver.speed == 20
    assert driver.commands == [[0.0] * 6]


def test_speed_percent_accepts_one_to_one_hundred() -> None:
    with pytest.raises(SystemExit) as exc:
        cli.main(["hold", "--speed-percent", "101"])

    assert exc.value.code == 2


@pytest.mark.parametrize("command", ["on", "off", "fw"])
def test_short_command_aliases_are_accepted(command) -> None:
    cli._parser().parse_args([command])


def test_jog_alias_is_rejected() -> None:
    with pytest.raises(SystemExit) as exc:
        cli._parser().parse_args(["jog", "--joint", "1", "--delta-deg", "1"])

    assert exc.value.code == 2


def test_sdk_diagnostic_commands_are_short(monkeypatch, capsys) -> None:
    driver = FakeDriver()
    monkeypatch.setattr(cli, "PiperXDriver", lambda **_: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    assert cli.main(["fps", "--wait", "0"]) == 0
    assert '"receive_fps": 200.0' in capsys.readouterr().out
    assert cli.main(["limits", "--wait", "0"]) == 0
    assert '"joints": []' in capsys.readouterr().out
    assert cli.main(["ratings", "--wait", "0"]) == 0
    assert '"crash_protection"' in capsys.readouterr().out


def test_stop_clear_and_configuration_commands(monkeypatch) -> None:
    driver = FakeDriver()
    monkeypatch.setattr(cli, "PiperXDriver", lambda **_: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    assert cli.main(["stop", "--wait", "0"]) == 0
    assert driver.stopped is True
    assert cli.main(["clear", "--wait", "0", "--joint", "2"]) == 0
    assert driver.cleared_joint == 2
    assert cli.main(["defaults", "--wait", "0"]) == 0
    assert driver.defaults_restored is True
    assert cli.main(["payload", "--wait", "0", "--payload", "full"]) == 0
    assert driver.payload == "full"
    assert cli.main(["install", "--wait", "0", "--position", "left"]) == 0
    assert driver.installation == "left"


def test_movejs_and_cartesian_commands(monkeypatch) -> None:
    driver = FakeDriver()
    monkeypatch.setattr(cli, "PiperXDriver", lambda **_: driver)
    monkeypatch.setattr(cli.time, "sleep", lambda _: None)

    joints = ["0.1", "0", "0", "0", "0", "0"]
    assert cli.main(["movejs", "--wait", "0", "--joints", *joints]) == 0
    assert driver.fast_commands[-1] == pytest.approx([0.1, 0, 0, 0, 0, 0])
    pose = ["0.2", "0", "0.3", "0", "1.0", "0"]
    assert cli.main(["movep", "--wait", "0", "--pose", *pose]) == 0
    assert driver.pose_commands[-1][0] == "p"
    assert (
        cli.main(
            ["movec", "--wait", "0", "--pose", *pose, "--mid", *pose, "--end", *pose]
        )
        == 0
    )
    assert driver.circle_command is not None
