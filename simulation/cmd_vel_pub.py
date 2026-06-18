#!/usr/bin/env python3
"""Publish cmd_vel to Gazebo to make the car drive."""
import subprocess, time, sys

topic = "/model/autocar_car/cmd_vel"
msg_type = "ignition.msgs.Twist"

# Forward for 5 seconds
print("Publishing cmd_vel: forward 0.3 m/s")
end = time.time() + 5
while time.time() < end:
    subprocess.run([
        "ign", "topic", "-p", "linear: {x: 0.3}, angular: {z: 0.0}",
        "-t", topic, "-m", msg_type
    ], capture_output=True, timeout=3)
    # Also publish without angular to be safe
    subprocess.run([
        "ign", "topic", "-p", "linear: {x: 0.3}",
        "-t", topic, "-m", msg_type
    ], capture_output=True, timeout=3)
    time.sleep(0.2)

# Turn
print("Publishing cmd_vel: turn 0.5 rad/s")
end = time.time() + 3
while time.time() < end:
    subprocess.run([
        "ign", "topic", "-p", "angular: {z: 0.5}",
        "-t", topic, "-m", msg_type
    ], capture_output=True, timeout=3)
    time.sleep(0.2)

# Stop
print("Publishing cmd_vel: stop")
for _ in range(5):
    subprocess.run([
        "ign", "topic", "-p", "linear: {x: 0.0}, angular: {z: 0.0}",
        "-t", topic, "-m", msg_type
    ], capture_output=True, timeout=3)
    time.sleep(0.2)

print("Done")
