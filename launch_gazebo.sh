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

    GAZEBO_PID=$(pgrep -f 'gzserver')
    if [ ! -z "$GAZEBO_PID" ]; then
        echo "Force killing lingering Gazebo processes: $GAZEBO_PID"
        kill -9 $GAZEBO_PID
    fi

    GAZEBO_PID=$(pgrep -f 'gzclient')
    if [ ! -z "$GAZEBO_PID" ]; then
        echo "Force killing lingering Gazebo processes: $GAZEBO_PID"
        kill -9 $GAZEBO_PID
    fi

    sleep 1
    echo "Cleanup complete."
}

trap cleanup SIGINT SIGTERM EXIT

DRIVE_MODE_3=${2:-diff_drive}
DRIVE_MODE_4=${3:-diff_drive}

cd ~/inf3995/project_ws

source install/setup.sh

cleanup

cd src/ros_gz_example_gazebo/worlds

# Generate modified world file
MODIFIED_WORLD_FILE="modified_world.sdf"
python3 generate_world_with_obstacles.py $DRIVE_MODE_3 $DRIVE_MODE_4 $MODIFIED_WORLD_FILE

cd ~/inf3995/project_ws

# Launch Gazebo with the modified world file
ros2 launch ros_gz_example_bringup gazebo.launch.py
GAZEBO_PID=$!
PIDS+=($GAZEBO_PID)