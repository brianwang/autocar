#!/bin/bash
# Run on RPi: Gazebo + chassis simulator + cmd_vel
set -e

SIM_DIR="/home/andy/autocar_sim"

echo "=== Kill old processes ==="
pkill -f 'ign gazebo' 2>/dev/null || true
pkill -f 'ign.*server' 2>/dev/null || true
pkill -f 'ign.*gui' 2>/dev/null || true
pkill -f chassis_simulator 2>/dev/null || true
sleep 2

echo "=== Start chassis simulator ==="
nohup python3 /home/andy/chassis_simulator.py --host 0.0.0.0 --port 9999 > /tmp/chassis_sim.log 2>&1 &
sleep 1

echo "=== Start Gazebo on display :0 ==="
export DISPLAY=:0
export IGN_GAZEBO_RESOURCE_PATH=$SIM_DIR/models
setsid ign gazebo $SIM_DIR/worlds/test_obstacle.sdf > /tmp/gazebo.log 2>&1 &
disown
echo "Waiting for Gazebo (15s)..."
sleep 15

echo "=== Verify car topics ==="
ign topic -l 2>/dev/null | grep car || echo "WARNING: No car topics found"

echo "=== Publishing cmd_vel for 8 seconds ==="
python3 /home/andy/cmd_vel_pub.py

echo "=== Final odometry ==="
ign topic -e -t /model/autocar_car/odom -n 1 2>&1 | grep -A5 "position"

echo ""
echo "=== ALL DONE ==="
echo "Gazebo on screen :0"
echo "Chassis sim on port 9999"
