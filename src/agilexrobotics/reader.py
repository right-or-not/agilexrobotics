"""Read-only pyAgxArm adapter for the AgileX PiPER-X."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from types import TracebackType
from typing import Any, Literal, Self

from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

Firmware = Literal["default", "v183", "v188", "v189"]


@dataclass(frozen=True)
class PiperXSnapshot:
    """A JSON-friendly snapshot of the feedback currently held by the SDK."""

    connected: bool
    communication_ok: bool
    communication_error: str | None
    receive_fps: float
    firmware: dict[str, Any] | None
    joint_angles_rad: list[float] | None
    flange_pose_m_rad: list[float] | None
    arm_status: dict[str, Any] | None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _message_values(message: Any) -> list[float] | None:
    if message is None:
        return None
    return [float(value) for value in message.msg]


def _status_values(message: Any) -> dict[str, Any] | None:
    if message is None:
        return None
    status = message.msg
    return {
        "control_mode": int(status.ctrl_mode),
        "arm_status": int(status.arm_status),
        "mode_feedback": int(status.mode_feedback),
        "teach_status": int(status.teach_status),
        "motion_status": int(status.motion_status),
        "trajectory_number": int(status.trajectory_num),
        "error_status": str(status.err_status),
        "receive_hz": float(message.hz),
        "timestamp": float(message.timestamp),
    }


class PiperXConnection:
    """Own a pyAgxArm PiPER-X connection and expose read-only operations."""

    def __init__(
        self,
        *,
        channel: str = "can0",
        interface: str = "socketcan",
        firmware: Firmware = "default",
        arm: Any | None = None,
    ) -> None:
        if not channel:
            raise ValueError("CAN channel must not be empty")
        if arm is None:
            arm = create_piper_x_arm(
                channel=channel, interface=interface, firmware=firmware
            )
        self._arm: Any = arm

    def connect(self) -> None:
        self._arm.connect(start_read_thread=True)

    def close(self) -> None:
        self._arm.disconnect()

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def read_firmware(self, *, timeout: float = 1.0) -> dict[str, Any] | None:
        if timeout < 0:
            raise ValueError("firmware timeout must be non-negative")
        return self._arm.get_firmware(timeout=timeout)

    def read_joint_angles(self) -> list[float] | None:
        return _message_values(self._arm.get_joint_angles())

    def read_snapshot(self, *, firmware_timeout: float = 1.0) -> PiperXSnapshot:
        error = self._arm.get_comm_error()
        return PiperXSnapshot(
            connected=bool(self._arm.is_connected()),
            communication_ok=bool(self._arm.is_ok()),
            communication_error=None if error is None else str(error),
            receive_fps=float(self._arm.get_fps()),
            firmware=self.read_firmware(timeout=firmware_timeout),
            joint_angles_rad=_message_values(self._arm.get_joint_angles()),
            flange_pose_m_rad=_message_values(self._arm.get_flange_pose()),
            arm_status=_status_values(self._arm.get_arm_status()),
        )


def create_piper_x_arm(
    *,
    channel: str = "can0",
    interface: str = "socketcan",
    firmware: Firmware = "default",
) -> Any:
    """Create an unconnected official SDK instance for PiPER-X."""
    if not channel:
        raise ValueError("CAN channel must not be empty")
    config = create_agx_arm_config(
        robot=ArmModel.PIPER_X,
        firmeware_version=firmware,
        interface=interface,
        channel=channel,
        bitrate=1_000_000,
        auto_connect=False,
    )
    return AgxArmFactory.create_arm(config)
