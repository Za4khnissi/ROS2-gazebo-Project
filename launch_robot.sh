#!/bin/bash

# Check for ROBOT_ID parameter
if [ -z "$1" ]; then
    echo "Usage: $0 ROBOT_ID"
    echo "For multiple robots in simulation, use 'simulation' as ROBOT_ID"
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

# kill_existing_services() {
#     echo "Killing existing identify and mission services..."

#     # Identify and kill any existing identify_service nodes
#     IDENTIFY_PIDS=$(ros2 node list | grep identify_service)
#     if [ ! -z "$IDENTIFY_PIDS" ]; then
#         for NODE in $IDENTIFY_PIDS; do
#             kill $NODE
#             echo "Killed $NODE"
#         done
#     fi

#     # Identify and kill any existing mission_service nodes
#     MISSION_PIDS=$(ros2 node list | grep mission_service)
#     if [ ! -z "$MISSION_PIDS" ]; then
#         for NODE in $MISSION_PIDS; do
#             kill $NODE
#             echo "Killed $NODE"
#         done
#     fi
# }

# # Function to kill existing rosbridge_server processes
kill_rosbridge_server() {
    # Adjust the pattern based on your actual process
    ROSBRIDGE_PIDS=$(pgrep -f 'rosbridge_websocket_launch')
    # If the above doesn't find any, try a broader pattern
    if [ -z "$ROSBRIDGE_PIDS" ]; then
        ROSBRIDGE_PIDS=$(pgrep -f 'rosbridge_websocket')
    fi
    
    if [ ! -z "$ROSBRIDGE_PIDS" ]; then
        echo "Killing existing rosbridge_server processes: $ROSBRIDGE_PIDS"
        kill $ROSBRIDGE_PIDS
        
        # Wait for processes to terminate gracefully
        sleep 2
        
        # Verify termination
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

pkill -f ros2

# Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start


# Update package lists (uncomment if needed)
# sudo apt update

# Install ros-humble-rosbridge-server if not installed
if ! is_apt_package_installed ros-humble-rosbridge-server; then
    sudo apt install -y ros-humble-rosbridge-server
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

# Ensure ROS_DOMAIN_ID is set in ~/.bashrc
if ! grep -q "export ROS_DOMAIN_ID=49" ~/.bashrc; then
    echo 'export ROS_DOMAIN_ID=49' >> ~/.bashrc
    source ~/.bashrc
fi



# Go to user home directory
cd "$HOME"

# Check if inf3995 folder exists, if not clone it
if [ ! -d "$HOME/inf3995" ]; then
    git clone https://gitlab.com/polytechnique-montr-al/inf3995/20243/equipe-105/inf3995.git "$HOME/inf3995"
fi

# Change permissions if necessary
if [ -e /dev/ttyTHS1 ]; then
    sudo chmod 666 /dev/ttyTHS1
fi

# Navigate to the project workspace
cd "$HOME/inf3995/project_ws"

# Build the workspace
if [ "$ROBOT_ID" == "simulation" ]; then
    colcon build --cmake-args -DBUILD_TESTING=ON --packages-skip voice_control ydlidar_ros2_driver limo_base
else
    colcon build --cmake-args -DBUILD_TESTING=ON
fi

# Source the workspace
source install/setup.sh

# Run commands based on ROBOT_ID
if [ "$ROBOT_ID" == "simulation" ]; then
    echo "Launching Gazebo simulation with Robot 3 and Robot 4."

    # Kill existing rosbridge_server processes
    kill_rosbridge_server

    # Kill existing services to avoid duplicates
    #kill_existing_services

    # Launch the bridge once
    ros2 launch ros_gz_example_bringup bridge.launch.py &
    BRIDGE_PID=$!
    PIDS+=($BRIDGE_PID)

    # Launch Gazebo simulation once
    ros2 launch ros_gz_example_bringup gazebo.launch.py &
    GAZEBO_PID=$!

    # Give Gazebo time to start
    sleep 5

    # Launch the robot instance without Gazebo
    ros2 launch ros_gz_example_bringup diff_drive_sim.launch.py gazebo:=false &
    PIDS+=($!)  # Collect the process ID

    # Launch rosbridge_server
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    ROSBRIDGE_PID=$!
    PIDS+=($ROSBRIDGE_PID)

    # Wait for all robot nodes and rosbridge_server
    for PID in "${PIDS[@]}"; do
        wait $PID
    done

    # Wait for Gazebo to finish
    wait $GAZEBO_PID

elif [ "$ROBOT_ID" == "1" ]; then
    # Run command for Robot 1
    ros2 launch limo_base limo_base.launch.py &
    LAUNCH_PID=$!
    echo "Robot 1 ready. Launch rosbridge to start interacting from the dashboard."
    wait $LAUNCH_PID

elif [ "$ROBOT_ID" == "2" ]; then
    # Run command for Robot 2
    ros2 launch limo_base limo_base.launch.py &
    LAUNCH_PID=$!
    echo "Robot 2 ready. Launch rosbridge to start interacting with the robot from the dashboard."
    wait $LAUNCH_PID

else
    echo "Invalid ROBOT_ID. Please provide ROBOT_ID as 1, 2, or 'simulation'"
    exit 1
fi
