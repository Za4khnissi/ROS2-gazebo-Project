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
    echo "Attempting graceful shutdown of ROS nodes and processes..."

    # Terminate by process names without using forceful kill immediately
    pkill -f 'gzserver'
    pkill -f 'gzclient'
    pkill -f 'rosbridge_websocket'
    pkill -f 'identify_service'
    pkill -f 'mission_service'

    # Allow time for processes to terminate gracefully
    sleep 1

    # Forcefully kill lingering processes if they still exist
    for PROCESS in 'gzserver' 'gzclient' 'rosbridge_websocket' 'identify_service' 'mission_service'; do
        if pgrep -f "$PROCESS" > /dev/null; then
            echo "Force killing lingering $PROCESS processes..."
            pkill -9 -f "$PROCESS"
        fi
    done

    sleep 1
    echo "Cleanup complete."
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
    
trap cleanup SIGINT SIGTERM EXIT

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

    colcon build --cmake-args -DBUILD_TESTING=ON --packages-skip voice_control ydlidar_ros2_driver limo_base limo_msgs limo_description limo_bringup

else colcon build --cmake-args -DBUILD_TESTING=ON

fi

if [ "$ROBOT_ID" == "simulation" ]; then

    source install/setup.sh

    cleanup

    kill_rosbridge_server

    echo "Launching simulation with Robot 3 mode: $DRIVE_MODE_3 and Robot 4 mode: $DRIVE_MODE_4."

    ros2 launch ros_gz_example_bringup bridge.launch.py &
    BRIDGE_PID=$!
    PIDS+=($BRIDGE_PID)

    ros2 launch ros_gz_example_bringup gazebo.launch.py drive_mode_3:=$DRIVE_MODE_3 drive_mode_4:=$DRIVE_MODE_4 &
    GAZEBO_PID=$!
    PIDS+=($GAZEBO_PID)

    ros2 launch ros_gz_example_bringup diff_drive_sim.launch.py drive_mode_3:=$DRIVE_MODE_3 drive_mode_4:=$DRIVE_MODE_4 &
    PIDS+=($!)


    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    ROSBRIDGE_PID=$!
    PIDS+=($ROSBRIDGE_PID)

    for PID in "${PIDS[@]}"; do
        wait $PID
    done

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
    echo "Invalid ROBOT_ID. Please provide ROBOT_ID as 1, 2, 'simulation', 'gazebo'."
    exit 1
fi