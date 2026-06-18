#!/usr/bin/env python3
"""Spawn autocar_car SDF model into running Gazebo instance."""
import sys
import time
import subprocess


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "test_obstacle"
    sdf_path = sys.argv[2] if len(sys.argv) > 2 else ""

    # Wait for Gazebo to be ready
    time.sleep(5)

    req = (
        'sdf_filename: "{}" name: "autocar_car" '
        "pose: {{position: {{x: 0 y: 0 z: 0.05}}}}"
    ).format(sdf_path)

    cmd = [
        "ign", "service", "-s", f"/world/{world}/create",
        "--reqtype", "ignition.msgs.EntityFactory",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "15",
        "--req", req,
    ]

    print(f"Spawning robot: {sdf_path}")
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    if result.returncode == 0:
        print(f"Spawn success: {result.stdout}")
    else:
        print(f"Spawn failed: {result.stderr or result.stdout}")
        sys.exit(1)


if __name__ == "__main__":
    main()
