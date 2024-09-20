#!/usr/bin/env python3
import os
import pathlib
import shlex
import subprocess


def main() -> None:
    root = pathlib.Path(__file__).resolve().parent.parent
    uid = os.getuid()
    gid = os.getgid()
    cmd = [
        "colcon",
        "build",
        "--cmake-args",
        "-DCMAKE_BUILD_TYPE=Release",
    ]
    exec_cmd = [
        "docker",
        "run",
        "--rm",
        "--net=host",
        "--user",
        f"{uid}:{gid}",
        "--volume",
        f"{root}/rosconjp:/rosconjp",
        "--volume",
        f"{root}/rosconjp_interfaces:/rosconjp_interfaces",
        "--volume",
        f"{root}/docker/ws:/docker/ws",
        "--workdir",
        "/docker/ws",
    ]
    exec_cmd += ["asia-northeast1-docker.pkg.dev/kachaka-api/docker/slam-workshop:main"]
    setup_bash = pathlib.Path("/opt/ros/humble/setup.bash")
    exec_cmd += ["bash", "-c", f"source {str(setup_bash)}; exec {shlex.join(cmd)}"]
    subprocess.run(exec_cmd, check=True)


if __name__ == "__main__":
    main()
