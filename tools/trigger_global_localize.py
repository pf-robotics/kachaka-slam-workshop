#!/usr/bin/env python3
import pathlib
import subprocess


def main() -> None:
    cmd = [
        "ros2",
        "service",
        "call",
        "/global_localize",
        "std_srvs/srv/Trigger",
        "{}",
    ]
    exec_cmd = ["docker", "exec", "-it", "slam-workshop-base-1"]
    setup_bash = pathlib.Path("/docker/ws/install/setup.bash")
    cmd_str = f'{cmd[0]} {cmd[1]} {cmd[2]} {cmd[3]} {cmd[4]} "{cmd[5]}"'
    exec_cmd += ["bash", "-c", f"source {str(setup_bash)}; {cmd_str}"]
    subprocess.run(exec_cmd, check=True)


if __name__ == "__main__":
    main()
