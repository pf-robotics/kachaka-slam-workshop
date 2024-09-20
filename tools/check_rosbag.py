#!/usr/bin/env python3
from dotenv import load_dotenv
import os
import pathlib
import shlex
import subprocess
import sys


def main() -> None:
    if len(sys.argv) < 2:
        print ("Please specify rosbag name")
        exit(1)
    target_rosbag_name = sys.argv[1]

    load_dotenv()
    URL = os.getenv('URL')
    TAG = os.getenv('TAG')
    cmd = [
        "ros2",
        "bag",
        "info",
        f"/resources/{target_rosbag_name}",
    ]
    exec_cmd = [
        "docker",
        "run",
        "--rm",
        "--volume",
        "./resources:/resources",
        "-it",
        f"{URL}:{TAG}"
    ]
    setup_bash = pathlib.Path("/opt/ros/humble/setup.bash")
    exec_cmd += ["bash", "-c", f"source {str(setup_bash)}; {shlex.join(cmd)}"]
    subprocess.run(exec_cmd, check=True)


if __name__ == "__main__":
    main()
