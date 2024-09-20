#!/usr/bin/env python3
from dotenv import load_dotenv
import os
import pathlib
import shlex
import subprocess
import sys


def main() -> None:
    if len(sys.argv) < 2:
        print("Please specify rosbag name")
        exit(1)
    target_rosbag_name = sys.argv[1]
    load_dotenv()
    URL = os.getenv('URL')
    TAG = os.getenv('TAG')
    ROS_DOMAIN_ID = os.getenv('ROS_DOMAIN_ID')
    USER_ID = os.getuid()
    GROUP_ID = os.getgid()
    cmd = [
        "ros2",
        "bag",
        "play",
        f"/resources/{target_rosbag_name}",
        "--clock",
    ]
    exec_cmd = [
        "docker",
        "run",
        "--rm",
        "--net",
        "host",
        "--env",
        "ROS_LOG_DIR=/tmp",
        "--env",
        f"ROS_DOMAIN_ID={ROS_DOMAIN_ID}",
        "--user",
        f"{USER_ID}:{GROUP_ID}",
        "--volume",
        "./resources:/resources",
        "--volume",
        "/dev/shm:/dev/shm",
        "-it",
        f"{URL}:{TAG}"
    ]
    setup_bash = pathlib.Path("/opt/ros/humble/setup.bash")
    exec_cmd += ["bash", "-c", f"source {str(setup_bash)}; {shlex.join(cmd)}"]
    subprocess.run(exec_cmd, check=True)


if __name__ == "__main__":
    main()
