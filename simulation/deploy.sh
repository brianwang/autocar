#!/bin/bash
# Deploy and run Gazebo + chassis simulator + cmd_vel on RPi
set -e

HOST="192.168.4.11"
USER="andy"
SIM_DIR="/home/andy/autocar_sim"

echo "=== Step 1: Copy simulation files ==="
# Already copied, just verify
ssh "$USER@$HOST" "ls $SIM_DIR/worlds/test_obstacle.sdf"

echo "=== Step 2: Kill old processes ==="
ssh "$USER@$HOST" "pkill -f 'ign gazebo' 2>/dev/null; pkill -f 'ign.*server' 2>/dev/null; pkill -f 'ign.*gui' 2>/dev/null; pkill -f chassis_simulator 2>/dev/null; sleep 2"

echo "=== Step 3: Start chassis simulator (background) ==="
ssh "$USER@$HOST" "nohup python3 /home/andy/chassis_simulator.py --host 0.0.0.0 --port 9999 > /tmp/sim.log 2>&1 &"
sleep 1

echo "=== Step 4: Start Gazebo with display ==="
ssh "$USER@$HOST" "env DISPLAY=:0 IGN_GAZEBO_RESOURCE_PATH=$SIM_DIR/models setsid ign gazebo $SIM_DIR/worlds/test_obstacle.sdf > /tmp/gz.log 2>&1 & disown"
echo "Waiting for Gazebo to start..."
sleep 8

echo "=== Step 5: Spawn autocar_car model ==="
ssh "$USER@$HOST" "ign service -s /world/test_obstacle/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 10 --req 'sdf_filename: \"$SIM_DIR/models/autocar_car/model.sdf\" name: \"autocar_car\" pose: {position: {x: 0 y: 0 z: 0.05}}'"
sleep 2

echo "=== Step 6: Verify car loaded ==="
ssh "$USER@$HOST" "ign topic -l 2>/dev/null | grep car || echo 'NO CAR TOPICS'"

echo "=== Step 7: Move the car! ==="
ssh "$USER@$HOST" "python3 << 'PYEOF'
import subprocess, time
topic = '/model/autocar_car/cmd_vel'
msg = 'ignition.msgs.Twist'
print('Moving forward 0.3 m/s for 5s...')
for i in range(50):
    subprocess.run(['ign', 'topic', '-p', 'linear: {x: 0.3}', '-t', topic, '-m', msg],
                   capture_output=True, timeout=2)
    time.sleep(0.1)
print('Turning...')
for i in range(30):
    subprocess.run(['ign', 'topic', '-p', 'angular: {z: 0.5}', '-t', topic, '-m', msg],
                   capture_output=True, timeout=2)
    time.sleep(0.1)
print('Done')
PYEOF"

echo "=== Step 8: Check odometry ==="
ssh "$USER@$HOST" "ign topic -e -t /model/autocar_car/odom -n 1 2>&1 | grep -A3 position"

echo ""
echo "=== DONE ==="
echo "Gazebo on display :0"
echo "Chassis simulator on port 9999"
echo "Check: ssh $USER@$HOST 'tail -f /tmp/gz.log'"
