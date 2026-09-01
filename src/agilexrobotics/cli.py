"""提供 AgileX PiPER-X 机械臂诊断与控制命令行入口"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Sequence
from typing import Any

from agilexrobotics.driver import PiperXDriver
from agilexrobotics.exceptions import PiperXError
from agilexrobotics.reader import PiperXConnection

# 命令顺序与 README 保持一致：常用命令放在前面，便于帮助信息和源码检索。
_COMMON_READ_COMMANDS = ("status", "joints")
_UNCOMMON_READ_COMMANDS = ("firmware",)
_READ_COMMANDS = _COMMON_READ_COMMANDS + _UNCOMMON_READ_COMMANDS

_COMMON_HARDWARE_COMMANDS = (
    "enable",
    "disable",
    "stop",
    "reset",
    "movej",
    "movejs",
    "movep",
    "movel",
    "movec",
)
_UNCOMMON_HARDWARE_COMMANDS = (
    "driver-status",
    "motor-status",
    "pose",
    "fps",
    "limits",
    "ratings",
    "hold",
    "zero",
    "move-joint",
    "clear",
    "max-limits",
    "defaults",
    "speed",
    "payload",
    "install",
    "protect",
    "assist",
    "grip-status",
    "grip",
    "grip-reset",
    "grip-zero",
)
_HARDWARE_COMMANDS = _COMMON_HARDWARE_COMMANDS + _UNCOMMON_HARDWARE_COMMANDS
_ALIASES = {
    "on": "enable",
    "off": "disable",
    "fw": "firmware",
}
_MAX_HARDWARE_TEST_DELTA_DEG = 90.0
_HARDWARE_TEST_MOTION_TIMEOUT = 10.0


def _parser() -> argparse.ArgumentParser:
    """创建并返回用于解析 `ag` 命令行参数的 parser。"""

    # 先定义程序名称和总体说明
    # 用户执行 `ag --help` 时会看到这些内容。
    parser = argparse.ArgumentParser(
        prog="ag",
        description="AgileX PiPER-X diagnostics and guarded hardware checks.",
    )
    # command: 可选择 read commands、hardware commands and aliases
    parser.add_argument(
        "command", choices=_READ_COMMANDS + _HARDWARE_COMMANDS + tuple(_ALIASES)
    )
    # --channel: Select SocketCAN channel
    # default: can0
    parser.add_argument("--channel", default="can0", help="SocketCAN channel")
    # --interface: Select python-can rear end
    # default: socketcan
    parser.add_argument("--interface", default="socketcan", help="python-can interface")
    # --firmware: Select firmware version
    # default: default
    parser.add_argument(
        "--firmware",
        choices=("default", "v183", "v188", "v189"),
        default="default",
        help="PiPER-X main-controller firmware family",
    )
    # --wait: Give periodic feedback time to be received
    # default: 0.2
    parser.add_argument(
        "--wait",
        type=float,
        default=0.2,
        help="seconds to wait for periodic feedback",
    )
    # --timeout: Limit the maximum time to wait for firmware or configuration feedback.
    # default: 1.0s
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="firmware query timeout in seconds",
    )
    # --speed-percent: Set the motion speed percentage for move_j commands
    # default: 20%
    parser.add_argument(
        "--speed-percent",
        type=int,
        default=20,
        help="move_j speed percentage; defaults to 20 and accepts 1..100",
    )
    # --joint: Select a single joint for move-joint command (1..6)
    # range: 1..6
    parser.add_argument("--joint", type=int, help="joint number for move-joint (1..6)")
    # --delta-deg: relative move for move-joint command
    # range: 0..90 degrees
    parser.add_argument(
        "--delta-deg",
        type=float,
        help="relative move for move-joint; magnitude restricted to 90 degrees",
    )
    # --joints: Receive the absolute target angles of the six joints (J1～J6)
    # In radians
    parser.add_argument(
        "--joints", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6")
    )
    # --pose: Get the value of X、Y、Z、Roll、Pitch、Yaw
    parser.add_argument(
        "--pose", type=float, nargs=6, metavar=("X", "Y", "Z", "R", "P", "Y")
    )
    # --mid & --end 只用于圆弧运动
    # --mid: 途经点位姿
    # --end: 终点位姿
    parser.add_argument("--mid", type=float, nargs=6)
    parser.add_argument("--end", type=float, nargs=6)
    # --rating: 设置碰撞保护或助力等级
    parser.add_argument("--rating", type=int)
    # --width: gripper width (m)
    # --force: gripper force (N) default: 1.0N
    parser.add_argument("--width", type=float, help="gripper width in meters")
    parser.add_argument("--force", type=float, default=1.0, help="gripper force in N")
    # --payload & --position 告诉控制器当前负载 & 安装方向
    parser.add_argument("--payload", choices=("empty", "half", "full"), default="empty")
    parser.add_argument(
        "--position",
        choices=("horizontal", "left", "right"),
        default="horizontal",
    )
    # 保留旧版确认参数以兼容已有脚本
    parser.add_argument(
        "--confirm-hardware-test",
        action="store_true",
        help="legacy compatibility flag; hardware confirmation is implicit",
    )

    return parser


def _run_read_command(args: argparse.Namespace) -> object:
    """Run Read Commands: status、joints、firmware"""

    # connect to the robot and perform the requested read command
    with PiperXConnection(
        channel=args.channel,
        interface=args.interface,
        firmware=args.firmware,
    ) as arm:
        if args.wait:
            time.sleep(args.wait)
        # 常用只读命令放在前面，与 README 和 _READ_COMMANDS 的顺序一致。
        if args.command == "status":
            return arm.read_snapshot(firmware_timeout=args.timeout).as_dict()
        if args.command == "joints":
            return arm.read_joint_angles()
        return arm.read_firmware(timeout=args.timeout)


def _validate_hardware_arguments(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    """Check the hardware command arguments & output error messages"""

    # check the speed percent
    if not 1 <= args.speed_percent <= 100:
        parser.error("--speed-percent must be in [1, 100]")
    # check the parameters of move-joint command
    if args.command == "move-joint":
        if args.joint is None or not 1 <= args.joint <= 6:
            parser.error("move_joint requires --joint in [1, 6]")
        if (
            args.delta_deg is None
            or not math.isfinite(args.delta_deg)
            or args.delta_deg == 0
            or abs(args.delta_deg) > _MAX_HARDWARE_TEST_DELTA_DEG
        ):
            parser.error("move-joint requires --delta-deg within ±90")
    # check the parameters of different move commands
    if args.command in ("movej", "movejs") and args.joints is None:
        parser.error(f"{args.command} requires --joints J1 J2 J3 J4 J5 J6")
    if args.command in ("movep", "movel", "movec") and args.pose is None:
        parser.error(f"{args.command} requires --pose X Y Z R P Y")
    if args.command == "movec" and (args.mid is None or args.end is None):
        parser.error("movec also requires --mid and --end, each with six values")
    # protect 和 assist 的等级上限不同
    # 可以只设置某一个关节
    if args.command in ("protect", "assist") and args.rating is not None:
        limit = 8 if args.command == "protect" else 10
        if not 0 <= args.rating <= limit:
            parser.error(f"--rating for {args.command} must be in [0, {limit}]")
        if args.joint is not None and not 1 <= args.joint <= 6:
            parser.error("--joint must be in [1, 6]")
    if args.command == "grip" and args.width is None:
        parser.error("grip requires --width in meters")


def _enable_for_motion(driver: PiperXDriver, args: argparse.Namespace) -> Any:
    """检查当前状态、使能六轴并返回使能后的最新反馈。"""

    # 保留原有控制流程：发送使能前必须先确认反馈完整且机械臂状态安全。
    driver.get_state()
    driver.enable()
    driver.set_speed_percent(args.speed_percent)
    # 使能反馈存在延迟，等待后重新读取，不能使用使能前的旧状态构造目标。
    time.sleep(0.5)
    return driver.get_state()


def _run_segmented_joint_motion(
    driver: PiperXDriver,
    args: argparse.Namespace,
    enabled: Any,
) -> dict[str, Any]:
    """分段执行 ``movej`` 或 ``movejs`` 六轴绝对运动并等待到位。"""

    target = list(args.joints)
    start = enabled.joint_positions_rad
    # 按最大关节变化计算分段数，保持原有单条命令 90° 的跨度限制。
    largest_delta = max(
        abs(target_value - start_value)
        for target_value, start_value in zip(target, start, strict=True)
    )
    max_step_rad = math.radians(_MAX_HARDWARE_TEST_DELTA_DEG)
    segment_count = max(1, math.ceil(largest_delta / max_step_rad))
    reached = enabled
    fast_response = args.command == "movejs"
    if fast_response:
        driver.begin_fast_response_mode()
    try:
        for segment in range(1, segment_count + 1):
            fraction = segment / segment_count
            waypoint = [
                start_value + (target_value - start_value) * fraction
                for start_value, target_value in zip(start, target, strict=True)
            ]
            driver.command_joints(waypoint, fast_response=fast_response)
            reached = driver.wait_for_joints(
                waypoint,
                timeout=_HARDWARE_TEST_MOTION_TIMEOUT,
                refresh_command=(
                    lambda waypoint=waypoint: driver.command_joints(
                        waypoint, fast_response=True
                    )
                )
                if fast_response
                else None,
            )
    finally:
        if fast_response:
            driver.end_fast_response_mode()
    return {
        "action": f"{args.command}-target-reached",
        "segments": segment_count,
        "target_rad": target,
        "state": reached.as_dict(),
    }


def _run_common_hardware_command(
    driver: PiperXDriver, args: argparse.Namespace
) -> dict[str, Any]:
    """执行 README 中优先展示的常用 Hardware command。"""

    # 急停和控制状态恢复不依赖关节状态反馈，也不会进入统一使能流程。
    if args.command == "stop":
        driver.emergency_stop()
        return {"action": "electronic-emergency-stop-sent"}

    if args.command == "reset":
        driver.reset()
        time.sleep(0.2)
        return {"action": "motion-controller-reset-command-sent"}

    # 失能前先读取状态；操作完成后再次读取反馈确认最终状态。
    if args.command == "disable":
        driver.get_state()
        driver.disable()
        time.sleep(0.2)
        return {"action": "disabled", "state": driver.get_state().as_dict()}

    # enable 和全部常用运动命令共用完全相同的安全检查、使能与速度设置。
    enabled = _enable_for_motion(driver, args)
    if args.command == "enable":
        return {"action": "enabled", "state": enabled.as_dict()}

    if args.command in ("movej", "movejs"):
        return _run_segmented_joint_motion(driver, args, enabled)

    if args.command in ("movep", "movel"):
        # movep 使用 P 模式，movel 使用 L 模式；位姿由驱动再次校验。
        mode = "p" if args.command == "movep" else "l"
        driver.command_pose(args.pose, mode=mode)
        return {"action": f"{args.command}-sent", "pose_m_rad": args.pose}

    if args.command == "movec":
        # 圆弧运动需要起点、途经点和终点三个完整位姿。
        driver.command_circle(args.pose, args.mid, args.end)
        return {
            "action": "movec-sent",
            "start_pose_m_rad": args.pose,
            "mid_pose_m_rad": args.mid,
            "end_pose_m_rad": args.end,
        }

    raise RuntimeError(f"unsupported common hardware command: {args.command}")


def _run_uncommon_hardware_command(
    driver: PiperXDriver, args: argparse.Namespace
) -> dict[str, Any]:
    """执行 README 中归类为不常用的诊断、配置和辅助控制命令。"""

    # 以下配置、查询和夹爪命令不需要先使能六轴机械臂。
    if args.command == "max-limits":
        driver.maximize_joint_motion_limits(require_confirmation=True)
        return {
            "action": "firmware-motion-limits-confirmed",
            "max_joint_speed_rad_s": 3.0,
            "max_joint_acceleration_rad_s2": 5.0,
            "joints": [1, 2, 3, 4, 5, 6],
        }
    if args.command == "clear":
        driver.clear_errors(args.joint or 255)
        return {"action": "errors-cleared", "joint": args.joint or "all"}
    if args.command == "defaults":
        driver.restore_default_motion_limits()
        return {"action": "default-motion-limits-restored"}
    if args.command == "speed":
        driver.set_speed_percent(args.speed_percent)
        return {"action": "speed-set", "percent": args.speed_percent}
    if args.command == "payload":
        driver.set_payload(args.payload)
        return {"action": "payload-set", "payload": args.payload}
    if args.command == "install":
        driver.set_installation_position(args.position)
        return {"action": "installation-set", "position": args.position}
    if args.command == "fps":
        return {"receive_fps": driver.get_receive_fps()}
    if args.command == "limits":
        return driver.get_motion_limits(timeout=args.timeout)
    if args.command == "ratings":
        return driver.get_ratings(timeout=args.timeout)
    if args.command in ("protect", "assist"):
        # 未给 rating 时只读取等级；给出 rating 时才写入固件。
        if args.rating is None:
            return driver.get_ratings(timeout=args.timeout)
        driver.set_rating(args.command, args.rating, args.joint or 255)
        return {
            "action": f"{args.command}-rating-set",
            "joint": args.joint or "all",
            "rating": args.rating,
        }
    if args.command == "grip-status":
        return driver.get_gripper_status()
    if args.command == "grip":
        driver.command_gripper(args.width, args.force)
        return {
            "action": "gripper-command-sent",
            "width_m": args.width,
            "force_n": args.force,
        }
    if args.command == "grip-reset":
        driver.reset_gripper()
        return {"action": "gripper-reset"}
    if args.command == "grip-zero":
        driver.calibrate_gripper_zero(timeout=args.timeout)
        return {"action": "gripper-zero-calibrated"}

    # 状态类诊断先共享一次完整反馈读取，不会使能机械臂。
    if args.command in ("driver-status", "motor-status", "pose"):
        before = driver.get_state()
        if args.command == "driver-status":
            return {"action": "read-only", "state": before.as_dict()}
        if args.command == "motor-status":
            return {
                "action": "read-only-motor-diagnostics",
                "joint_state": before.as_dict(),
                "motors": driver.get_motor_diagnostics(),
            }
        return {
            "flange_pose_m_rad": before.flange_pose_m_rad,
            "joint_positions_rad": before.joint_positions_rad,
        }

    # hold、zero 和 move-joint 仍沿用原来的统一安全使能流程。
    enabled = _enable_for_motion(driver, args)
    if args.command == "hold":
        driver.command_joints(enabled.joint_positions_rad)
        return {"action": "holding-current-position", "state": enabled.as_dict()}

    if args.command == "zero":
        start = enabled.joint_positions_rad
        largest_delta = max(abs(value) for value in start)
        max_step_rad = math.radians(_MAX_HARDWARE_TEST_DELTA_DEG)
        segment_count = max(1, math.ceil(largest_delta / max_step_rad))
        reached = enabled
        for segment in range(1, segment_count + 1):
            fraction = segment / segment_count
            waypoint = [value * (1.0 - fraction) for value in start]
            driver.command_joints(waypoint)
            reached = driver.wait_for_joints(
                waypoint, timeout=_HARDWARE_TEST_MOTION_TIMEOUT
            )
        return {
            "action": "zero-position-reached",
            "segments": segment_count,
            "target_rad": [0.0] * 6,
            "state": reached.as_dict(),
        }

    if args.command == "move-joint":
        # 只修改指定关节目标，其余五轴使用使能后的最新反馈位置。
        target = list(enabled.joint_positions_rad)
        target[args.joint - 1] += math.radians(args.delta_deg)
        driver.command_joints(target)
        reached = driver.wait_for_joints(
            target,
            joint_indices=(args.joint - 1,),
            timeout=_HARDWARE_TEST_MOTION_TIMEOUT,
        )
        return {
            "action": "relative-joint-target-reached",
            "joint": args.joint,
            "delta_deg": args.delta_deg,
            "target_rad": target,
            "state": reached.as_dict(),
        }

    raise RuntimeError(f"unsupported uncommon hardware command: {args.command}")


def _run_hardware_command(args: argparse.Namespace) -> dict[str, Any]:
    """连接驱动，并按 README 的常用/不常用分组执行 Hardware command。"""

    driver = PiperXDriver(
        channel=args.channel,
        interface=args.interface,
        firmware=args.firmware,
        max_command_delta_rad=math.radians(_MAX_HARDWARE_TEST_DELTA_DEG),
    )
    try:
        driver.connect()
        if args.wait:
            time.sleep(args.wait)
        if args.command in _COMMON_HARDWARE_COMMANDS:
            return _run_common_hardware_command(driver, args)
        return _run_uncommon_hardware_command(driver, args)
    finally:
        # 始终释放主机连接但不主动失能，避免无制动机械臂因重力下坠。
        driver.close()


def main(argv: Sequence[str] | None = None) -> int:
    """解析参数、分派命令，并以 JSON 输出结果和进程退出码。"""

    # 将字符串参数解析为 Namespace，并把 on、off、fw 等别名转成内部命令。
    parser = _parser()
    args = parser.parse_args(argv)
    args.command = _ALIASES.get(args.command, args.command)
    if args.wait < 0 or args.timeout < 0:
        parser.error("--wait and --timeout must be non-negative")

    # 在连接真实硬件前完成参数校验，避免无效输入触发机械臂动作。
    _validate_hardware_arguments(parser, args)

    try:
        # 只读命令使用轻量连接，其余命令统一交由安全驱动处理。
        result = (
            _run_read_command(args)
            if args.command in _READ_COMMANDS
            else _run_hardware_command(args)
        )
    # 将预期内的连接、协议和参数异常转成简洁错误信息及退出码 1。
    except (OSError, PiperXError, RuntimeError, ValueError) as exc:
        print(f"agilexrobotics: {exc}", file=sys.stderr)
        return 1

    # None 表示设备在超时时间内没有返回反馈，使用单独的退出码 2。
    if result is None:
        print("No feedback received.", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
    return 0
