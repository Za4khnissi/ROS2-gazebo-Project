#!/bin/bash

export ROS_DOMAIN_ID=49

if ! grep -q "export ROS_DOMAIN_ID=49" ~/.bashrc; then
    echo 'export ROS_DOMAIN_ID=49' >> ~/.bashrc
    source ~/.bashrc
fi

cleanup() {
    echo "Attempting graceful shutdown of ROS nodes and processes..."

    # Terminate by process names without using forceful kill immediately
    pkill -f 'gzserver'
    pkill -f 'gzclient'

    # Allow time for processes to terminate gracefully
    sleep 1

    # Forcefully kill lingering processes if they still exist
    for PROCESS in 'gzserver' 'gzclient'; do
        if pgrep -f "$PROCESS" > /dev/null; then
            echo "Force killing lingering $PROCESS processes..."
            pkill -9 -f "$PROCESS"
        fi
    done

    sleep 1
    echo "Cleanup complete."
}

trap cleanup SIGINT SIGTERM EXIT

DRIVE_MODE_3=${2:-diff_drive}  # Default to 'diff_drive' if not provided
DRIVE_MODE_4=${3:-diff_drive}  # Default to 'diff_drive' if not provided

cd ~/inf3995/project_ws

source install/setup.sh

cleanup

ros2 launch ros_gz_example_bringup gazebo.launch.py drive_mode_3:=$DRIVE_MODE_3 drive_mode_4:=$DRIVE_MODE_4 &
GAZEBO_PID=$!
PIDS+=($GAZEBO_PID)