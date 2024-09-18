#!/bin/bash

# Check for ROBOT_ID parameter
if [ -z "$1" ]; then
    echo "Usage: $0 ROBOT_ID"
    exit 1
fi

ROBOT_ID=$1

# Function to check if an apt package is installed
is_apt_package_installed() {
    dpkg -s "$1" &> /dev/null
}

# Function to check if a pip package is installed
is_pip_package_installed() {
    pip show "$1" &> /dev/null
}

# Update package lists
# sudo apt update

# Install ros-humble-rosbridge-server if not installed
if ! is_apt_package_installed ros-humble-rosbridge-server; then
    sudo apt install -y ros-humble-rosbridge-server
fi

# Install mpg123 if not installed
if ! is_apt_package_installed mpg123; then
    sudo apt install -y mpg123
fi

# Install python3-pip if not installed
if ! is_apt_package_installed python3-pip; then
    sudo apt install -y python3-pip
fi

# Install pygame using pip if not installed
if ! is_pip_package_installed pygame; then
    pip install pygame
fi

# Source ROS setup
source /opt/ros/humble/setup.bash

# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=49

# Go to user home directory
cd "$HOME"

# Check if inf3995 folder exists, if not clone it
if [ ! -d "$HOME/inf3995" ]; then
    git clone https://gitlab.com/polytechnique-montr-al/inf3995/20243/equipe-105/inf3995.git "$HOME/inf3995"
fi

# Change permissions
sudo chmod 666 /dev/ttyTHS1

# Navigate to the project workspace
cd "$HOME/inf3995/project_ws"


# Build the workspace
if [ "$ROBOT_ID" == "0" ]; then
	colcon build --cmake-args -DBUILD_TESTING=ON --packages-skip limo_msgs voice_control ydlidar_ros2_driver limo_base
else
	colcon build --cmake-args -DBUILD_TESTING=ON
fi

# Source the workspace
source install/setup.sh

# Set namespace
export ROBOT_ID=${ROBOT_ID}

# Run commands based on ROBOT_ID
if [ "$ROBOT_ID" == "0" ]; then
    # Run command with namespace in background
    ros2 launch ros_gz_example_bringup diff_drive.launch.py &
    PID1=$!
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    PID2=$!
    # Echo message
    echo "Simulation ready. Rosbridge ready. You can start interacting with the robot from the dashboard."
    # Bring the second background process to the foreground
    wait $PID2
elif [ "$ROBOT_ID" == "1" ]; then
    # Run first command with namespace in background
    ros2 launch limo_base limo_base.launch.py &
    LAUNCH_PID=$!
    # Echo message
    echo "Robot 1 ready. Launch this bash file with robot_id = 2 on the second robot to start interacting from the dashboard."
    # Bring the background process to the foreground
    wait $LAUNCH_PID
elif [ "$ROBOT_ID" == "2" ]; then
    # Run first command with namespace in background
    ros2 launch limo_base limo_base.launch.py &
    PID1=$!
    # Run second command with namespace in background
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    PID2=$!
    # Echo message
    echo "Robot 2 ready. Rosbridge ready. You can start interacting with the robot from the dashboard."
    # Bring the second background process to the foreground
    wait $PID2
else
    echo "Invalid ROBOT_ID. Please provide ROBOT_ID as 0, 1, or 2."
    exit 1
fi

