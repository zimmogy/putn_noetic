#!/usr/bin/env python3
"""Repeat a one-episode roslaunch campaign until its checkpoint is complete."""

import argparse
import json
import os
import socket
import subprocess
import sys
import time


def read_checkpoint(path):
    try:
        with open(path, "r") as stream:
            return json.load(stream)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as exc:
        raise RuntimeError("unable to read checkpoint {}: {}".format(path, exc))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Relaunch an isolated navigation evaluation after every checkpointed "
            "sample, stopping when the campaign reports complete=true."
        )
    )
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--restart-delay-s", type=float, default=5.0)
    parser.add_argument(
        "--gazebo-master-port-base",
        type=int,
        default=0,
        help=(
            "Gazebo transport port for the first session; 0 selects a free "
            "port for every session (recommended). Positive values increment "
            "once per session."
        ),
    )
    parser.add_argument(
        "--ros-master-port-base",
        type=int,
        default=0,
        help=(
            "ROS master port for the first session; 0 selects a free port "
            "for every session (recommended). Positive values increment "
            "once per session."
        ),
    )
    parser.add_argument(
        "--max-sessions",
        type=int,
        default=30,
        help="safety limit; use a value larger than the requested sample count",
    )
    parser.add_argument(
        "launch_args",
        nargs=argparse.REMAINDER,
        help="arguments after --: PACKAGE FILE.launch [name:=value ...]",
    )
    args = parser.parse_args()
    if args.launch_args and args.launch_args[0] == "--":
        args.launch_args = args.launch_args[1:]
    if len(args.launch_args) < 2:
        parser.error(
            "provide roslaunch package and launch file after --, for example "
            "-- putn_launch map_test_nav_eval_with_sim.launch"
        )
    if args.max_sessions <= 0:
        parser.error("--max-sessions must be positive")
    if args.restart_delay_s < 0.0:
        parser.error("--restart-delay-s cannot be negative")
    if args.gazebo_master_port_base < 0 or args.gazebo_master_port_base > 65535:
        parser.error("--gazebo-master-port-base must be 0 or a valid TCP port")
    if (
        args.gazebo_master_port_base > 0
        and args.gazebo_master_port_base + args.max_sessions - 1 > 65535
    ):
        parser.error("Gazebo master port range exceeds 65535")
    if args.ros_master_port_base < 0 or args.ros_master_port_base > 65535:
        parser.error("--ros-master-port-base must be 0 or a valid TCP port")
    if (
        args.ros_master_port_base > 0
        and args.ros_master_port_base + args.max_sessions - 1 > 65535
    ):
        parser.error("ROS master port range exceeds 65535")
    args.checkpoint = os.path.abspath(os.path.expanduser(args.checkpoint))
    return args


def find_free_tcp_port(excluded_ports=None):
    """Select an unused loopback port, avoiding ports chosen for this session."""
    excluded_ports = set(excluded_ports or [])
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.bind(("127.0.0.1", 0))
            port = listener.getsockname()[1]
        if port not in excluded_ports:
            return port


def sampling_progress(checkpoint):
    directions = checkpoint.get("sampling", {}).get("directions", [])
    if not directions:
        return "sampling progress unavailable"
    return ", ".join(
        "{}={}/{}".format(
            item.get("direction", item.get("group", "unknown")),
            item.get("eligible_samples", "?"),
            item.get("target_eligible_samples", "?"),
        )
        for item in directions
    )


def main():
    args = parse_args()
    command = ["roslaunch"] + args.launch_args
    for session_index in range(1, args.max_sessions + 1):
        checkpoint = read_checkpoint(args.checkpoint)
        if checkpoint and checkpoint.get("complete", False):
            print(
                "Campaign {} is complete: {}".format(
                    checkpoint.get("campaign_id", ""),
                    sampling_progress(checkpoint),
                ),
                flush=True,
            )
            return 0

        if args.gazebo_master_port_base > 0:
            gazebo_master_port = args.gazebo_master_port_base + session_index - 1
        else:
            gazebo_master_port = find_free_tcp_port()
        if args.ros_master_port_base > 0:
            ros_master_port = args.ros_master_port_base + session_index - 1
        else:
            ros_master_port = find_free_tcp_port([gazebo_master_port])
        if ros_master_port == gazebo_master_port:
            print(
                "ROS and Gazebo master ports must differ; both resolved to {}".format(
                    ros_master_port
                ),
                file=sys.stderr,
            )
            return 2
        launch_env = os.environ.copy()
        launch_env["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(
            ros_master_port
        )
        launch_env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:{}".format(
            gazebo_master_port
        )
        print(
            "Starting isolated simulator session {}/{} with "
            "ROS_MASTER_URI={} GAZEBO_MASTER_URI={}: {}".format(
                session_index,
                args.max_sessions,
                launch_env["ROS_MASTER_URI"],
                launch_env["GAZEBO_MASTER_URI"],
                " ".join(command),
            ),
            flush=True,
        )
        try:
            return_code = subprocess.call(command, env=launch_env)
        except KeyboardInterrupt:
            print("Interrupted; the last atomic checkpoint remains resumable.")
            return 130
        if return_code != 0:
            print(
                "roslaunch exited with code {}; refusing an automatic retry".format(
                    return_code
                ),
                file=sys.stderr,
            )
            return return_code

        checkpoint = read_checkpoint(args.checkpoint)
        if checkpoint is None:
            print(
                "roslaunch exited without creating checkpoint {}; stopping".format(
                    args.checkpoint
                ),
                file=sys.stderr,
            )
            return 2
        print(
            "Session finished: complete={} reason={} progress={}".format(
                bool(checkpoint.get("complete", False)),
                checkpoint.get("terminal_reason", ""),
                sampling_progress(checkpoint),
            ),
            flush=True,
        )
        if checkpoint.get("complete", False):
            return 0
        if session_index < args.max_sessions and args.restart_delay_s > 0.0:
            time.sleep(args.restart_delay_s)

    print(
        "Campaign is still incomplete after {} sessions; stopping at the safety limit".format(
            args.max_sessions
        ),
        file=sys.stderr,
    )
    return 3


if __name__ == "__main__":
    sys.exit(main())
