#!/usr/bin/env python3
import pathlib
import subprocess
import sys


def main() -> None:
    if len(sys.argv) < 2:
        print("Please specify output directory in resources")
        exit(1)
    target_output_dir = sys.argv[1]
    cmd = [
        "ros2",
        "service",
        "call",
        "/save_map",
        "rosconjp_interfaces/srv/SaveMap",
        "{output_dir: /resources/" + target_output_dir + "}",
    ]
    exec_cmd = ["docker", "exec", "-it", "slam-workshop-base-1"]
    setup_bash = pathlib.Path("/docker/ws/install/setup.bash")
    cmd_str = f'{cmd[0]} {cmd[1]} {cmd[2]} {cmd[3]} {cmd[4]} "{cmd[5]}"'
    exec_cmd += ["bash", "-c", f"source {str(setup_bash)}; {cmd_str}"]
    subprocess.run(exec_cmd, check=True)
    subprocess.run(['sed',
                    '-i',
                    f's/^TARGET_MAP_DIRECTORY *= *.*/TARGET_MAP_DIRECTORY=\"{target_output_dir}\"/',
                    '.env'], check=True)


if __name__ == "__main__":
    main()
