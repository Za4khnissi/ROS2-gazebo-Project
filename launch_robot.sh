#!/bin/bash

# Check for ROBOT_ID parameter
if [ -z "$1" ]; then
    echo "Usage: $0 ROBOT_ID [DRIVE_MODE_3] [DRIVE_MODE_4]"
    echo "For multiple robots in simulation, use 'simulation' as ROBOT_ID"
    echo "For example: $0 simulation diff_drive ackermann"
    exit 1
fi

ROBOT_ID=$1

# Set drive modes for robots 3 and 4 only in simulation
if [ "$ROBOT_ID" == "simulation" ]; then
    DRIVE_MODE_3=${2:-diff_drive}  # Default to 'diff_drive' if not provided
    DRIVE_MODE_4=${3:-diff_drive}  # Default to 'diff_drive' if not provided
fi

# Function to check if an apt package is installed
is_apt_package_installed() {
    dpkg -s "$1" &> /dev/null
}

# Function to check if a pip package is installed
is_pip_package_installed() {
    pip show "$1" &> /dev/null
}

cleanup() {
    echo "Cleaning up..."
    # Terminate all collected process IDs gracefully
    for PID in "${PIDS[@]}"; do
        if kill -0 $PID 2>/dev/null; then
            kill $PID
            wait $PID
        fi
    done

    # Kill Gazebo if still running
    if kill -0 $GAZEBO_PID 2>/dev/null; then
        kill $GAZEBO_PID
        wait $GAZEBO_PID
    fi

    echo "All processes terminated."
}

trap cleanup SIGINT SIGTERM EXIT

kill_existing_services() {
    echo "Killing existing identify and mission services..."

    # Identify and kill any existing identify_service nodes
    IDENTIFY_PIDS=$(ros2 node list | grep identify_service)
    if [ ! -z "$IDENTIFY_PIDS" ]; then
        for NODE in $IDENTIFY_PIDS; do
            ros2 node kill $NODE
            echo "Killed $NODE"
        done
    fi

    # Identify and kill any existing mission_service nodes
    MISSION_PIDS=$(ros2 node list | grep mission_service)
    if [ ! -z "$MISSION_PIDS" ]; then
        for NODE in $MISSION_PIDS; do
            ros2 node kill $NODE
            echo "Killed $NODE"
        done
    fi

    DRIVE_MODE_PIDS=$(ros2 node list | grep change_drive_mode)
    if [ ! -z "$DRIVE_MODE_PIDS" ]; then
        for NODE in $DRIVE_MODE_PIDS; do
            ros2 node kill $NODE
            echo "Killed $NODE"
        done
    fi
}

kill_rosbridge_server() {
    ROSBRIDGE_PIDS=$(pgrep -f 'rosbridge_websocket_launch')
    if [ -z "$ROSBRIDGE_PIDS" ];then
        ROSBRIDGE_PIDS=$(pgrep -f 'rosbridge_websocket')
    fi

    if [ ! -z "$ROSBRIDGE_PIDS" ]; then
        echo "Killing existing rosbridge_server processes: $ROSBRIDGE_PIDS"
        kill $ROSBRIDGE_PIDS

        sleep 2

        ROSBRIDGE_PIDS=$(pgrep -f 'rosbridge_websocket_launch')
        if [ -z "$ROSBRIDGE_PIDS" ]; then
            ROSBRIDGE_PIDS=$(pgrep -f 'rosbridge_websocket')
        fi

        if [ ! -z "$ROSBRIDGE_PIDS" ]; then
            echo "Force killing rosbridge_server processes: $ROSBRIDGE_PIDS"
            kill -9 $ROSBRIDGE_PIDS
        fi
    fi
}

# Ensure necessary dependencies are installed
if ! is_apt_package_installed ros-humble-rosbridge-server; then
    sudo apt install -y ros-humble-rosbridge-server
fi

if ! is_apt_package_installed python3-pip; then
    sudo apt install -y python3-pip
fi

if ! is_pip_package_installed pygame; then
    pip install pygame
fi

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=49

if ! grep -q "export ROS_DOMAIN_ID=49" ~/.bashrc; then
    echo 'export ROS_DOMAIN_ID=49' >> ~/.bashrc
    source ~/.bashrc
fi

ros2 daemon stop
ros2 daemon start

cd "$HOME"

if [ ! -d "$HOME/inf3995" ]; then
    git clone https://gitlab.com/polytechnique-montr-al/inf3995/20243/equipe-105/inf3995.git "$HOME/inf3995"
fi

if [ -e /dev/ttyTHS1 ]; then
    sudo chmod 666 /dev/ttyTHS1
fi

cd "$HOME/inf3995/project_ws"

if [ "$ROBOT_ID" == "simulation" ]; then
colcon build --cmake-args -DBUILD_TESTING=ON --packages-skip voice_control ydlidar_ros2_driver limo_base

else colcon build --cmake-args -DBUILD_TESTING=ON

fi

source install/setup.sh

if [ "$ROBOT_ID" == "simulation" ]; then
    echo "Launching Gazebo simulation with Robot 3 and Robot 4."

    kill_rosbridge_server

    kill_existing_services

    ros2 launch ros_gz_example_bringup bridge.launch.py &
    BRIDGE_PID=$!
    PIDS+=($BRIDGE_PID)

    ros2 launch ros_gz_example_bringup gazebo.launch.py &
    GAZEBO_PID=$!

    sleep 5

    # Launch the simulation with the drive modes for Robot 3 and 4
    ros2 launch ros_gz_example_bringup diff_drive_sim.launch.py drive_mode_3:=$DRIVE_MODE_3 drive_mode_4:=$DRIVE_MODE_4 &
    PIDS+=($!)

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    ROSBRIDGE_PID=$!
    PIDS+=($ROSBRIDGE_PID)

    for PID in "${PIDS[@]}"; do
        wait $PID
    done

    wait $GAZEBO_PID

elif [ "$ROBOT_ID" == "1" ]; then
    ros2 launch limo_base limo_base.launch.py &
    LAUNCH_PID=$!
    echo "Robot 1 ready. Launch rosbridge to start interacting from the dashboard."
    wait $LAUNCH_PID

elif [ "$ROBOT_ID" == "2" ]; then
    ros2 launch limo_base limo_base.launch.py &
    LAUNCH_PID=$!
    echo "Robot 2 ready. Launch rosbridge to start interacting with the robot from the dashboard."
    wait $LAUNCH_PID

else
    echo "Invalid ROBOT_ID. Please provide ROBOT_ID as 1, 2, or 'simulation'"
    exit 1
fi
