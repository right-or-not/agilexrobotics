"""ZMQ server exposing PiPER-X through GELLO's Robot wire protocol."""

from __future__ import annotations

import argparse
import math
import pickle
import signal
from collections.abc import Sequence
from typing import Any

import zmq

from agilexrobotics.exceptions import PiperXError
from agilexrobotics.gello_follower import GelloPiperXRobot


def _dispatch(robot: GelloPiperXRobot, request: dict[str, Any]) -> Any:
    method = request.get("method")
    args = request.get("args", {})
    if method == "num_dofs":
        return robot.num_dofs()
    if method == "get_joint_state":
        return robot.get_joint_state()
    if method == "command_joint_state":
        return robot.command_joint_state(**args)
    if method == "get_observations":
        return robot.get_observations()
    raise ValueError(f"unsupported GELLO robot method: {method!r}")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve PiPER-X to GELLO over ZMQ")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--firmware", default="default")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6001)
    parser.add_argument(
        "--hz",
        type=float,
        default=50.0,
        help="maximum PiPER-X JS command dispatch frequency (default: 50)",
    )
    parser.add_argument("--gripper-max-width-m", type=float, default=0.1)
    parser.add_argument("--gripper-force-n", type=float, default=1.0)
    parser.add_argument(
        "--configure-motion-limits",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "persistently request 3.0rad/s and 5.0rad/s^2; enabled by default; "
            "use --no-configure-motion-limits to skip the write"
        ),
    )
    parser.add_argument(
        "--confirm-hardware-control",
        action="store_true",
        help="legacy compatibility flag; hardware confirmation is implicit",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not math.isfinite(args.hz) or args.hz <= 0:
        raise SystemExit("--hz must be a positive finite value")

    robot = GelloPiperXRobot(
        channel=args.channel,
        interface=args.interface,
        firmware=args.firmware,
        configure_motion_limits=args.configure_motion_limits,
        control_hz=args.hz,
        gripper_max_width_m=args.gripper_max_width_m,
        gripper_force_n=args.gripper_force_n,
    )
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.RCVTIMEO, 1000)
    socket.bind(f"tcp://{args.host}:{args.port}")
    stopping = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        robot.start()
        limit_status = (
            "3.0rad/s and 5.0rad/s^2 confirmed"
            if robot.motion_limits_confirmed
            else "firmware motion limits unchanged"
        )
        print(
            f"PiPER-X GELLO server listening on tcp://{args.host}:{args.port}; "
            f"control limit {args.hz:g} Hz; {limit_status}"
        )
        while not stopping:
            try:
                request = pickle.loads(socket.recv())
            except zmq.Again:
                continue
            try:
                result = _dispatch(robot, request)
            except (PiperXError, RuntimeError, TypeError, ValueError) as exc:
                result = {"error": str(exc)}
            socket.send(pickle.dumps(result))
    finally:
        robot.close()
        socket.close()
        context.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
