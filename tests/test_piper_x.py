from __future__ import annotations

from dataclasses import dataclass

from agilexrobotics.piper_x import PiperXConnection


@dataclass
class FakeMessage:
    msg: object
    hz: float = 50.0
    timestamp: float = 123.0


@dataclass
class FakeStatus:
    ctrl_mode: int = 0
    arm_status: int = 0
    mode_feedback: int = 1
    teach_status: int = 0
    motion_status: int = 0
    trajectory_num: int = 0
    err_status: str = "normal"


class FakeArm:
    def __init__(self) -> None:
        self.connected = False
        self.disconnected = False

    def connect(self, *, start_read_thread: bool) -> None:
        assert start_read_thread
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def is_ok(self) -> bool:
        return True

    def get_comm_error(self):
        return None

    def get_fps(self) -> float:
        return 100.0

    def get_firmware(self, *, timeout: float):
        return {"software_version": "S-V1.8-9", "timeout": timeout}

    def get_joint_angles(self):
        return FakeMessage([0, 1, 2, 3, 4, 5])

    def get_flange_pose(self):
        return FakeMessage([0.1, 0.2, 0.3, 0, 0, 0])

    def get_arm_status(self):
        return FakeMessage(FakeStatus())


def test_snapshot_and_connection_lifecycle() -> None:
    sdk = FakeArm()
    with PiperXConnection(arm=sdk) as arm:
        snapshot = arm.read_snapshot(firmware_timeout=0.25)

    assert snapshot.connected is True
    assert snapshot.communication_ok is True
    assert snapshot.joint_angles_rad == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert snapshot.flange_pose_m_rad == [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    assert snapshot.firmware == {
        "software_version": "S-V1.8-9",
        "timeout": 0.25,
    }
    assert snapshot.arm_status is not None
    assert snapshot.arm_status["arm_status"] == 0
    assert sdk.disconnected is True


def test_missing_feedback_is_reported_as_none() -> None:
    sdk = FakeArm()
    sdk.get_joint_angles = lambda: None  # type: ignore[method-assign]
    sdk.get_flange_pose = lambda: None  # type: ignore[method-assign]
    sdk.get_arm_status = lambda: None  # type: ignore[method-assign]

    arm = PiperXConnection(arm=sdk)
    snapshot = arm.read_snapshot(firmware_timeout=0)

    assert snapshot.joint_angles_rad is None
    assert snapshot.flange_pose_m_rad is None
    assert snapshot.arm_status is None
