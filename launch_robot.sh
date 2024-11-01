#!/bin/bash

# Check for ROBOT_ID parameter
if [ -z "$1" ]; then
    echo "Usage: $0 ROBOT_ID [DRIVE_MODE_3] [DRIVE_MODE_4]"
    echo "For multiple robots in simulation, use 'simulation' as ROBOT_ID"
    echo "For example: $0 simulation diff_drive ackermann"
    exit 1
fi

ROBOT_ID=$1
MODIFIED_WORLD_FILE="modified_world.sdf"

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
    
trap cleanup SIGINT SIGTERM EXIT

pkill -f ros2



# Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# Clear any stale shared memory segments
sudo rm -rf /dev/shm/*


# Update package lists (uncomment if needed)
# sudo apt update

if ! is_apt_package_installed python3-pip; then
    sudo apt install -y python3-pip
fi

if ! is_pip_package_installed pygame; then
    pip install pygame
fi


# Install ROS 2 packages if not installed
ROS_DISTRO="humble"

# Install slam_toolbox
if ! is_apt_package_installed "ros-$ROS_DISTRO-slam-toolbox"; then
    sudo apt install -y "ros-$ROS_DISTRO-slam-toolbox"
fi

# Install nav2_bringup
if ! is_apt_package_installed "ros-$ROS_DISTRO-nav2-bringup"; then
    sudo apt install -y "ros-$ROS_DISTRO-nav2-bringup"
fi

if ! is_apt_package_installed "ros-$ROS_DISTRO-navigation2"; then
    sudo apt install -y "ros-$ROS_DISTRO-navigation2"
fi

# Install image_geometry
if ! is_apt_package_installed "ros-$ROS_DISTRO-image-geometry"; then
    sudo apt install -y "ros-$ROS_DISTRO-image-geometry"
fi

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=49

if ! grep -q "export ROS_DOMAIN_ID=49" ~/.bashrc; then
    echo 'export ROS_DOMAIN_ID=49' >> ~/.bashrc
    source ~/.bashrc
fi


cd "$HOME"

if [ ! -d "$HOME/inf3995" ]; then
    git clone https://gitlab.com/polytechnique-montr-al/inf3995/20243/equipe-105/inf3995.git "$HOME/inf3995"
fi

if [ -e /dev/ttyTHS1 ]; then
    sudo chmod 666 /dev/ttyTHS1
fi

cd "$HOME/inf3995/project_ws"

if [ "$ROBOT_ID" == "simulation" ]; then

    cd src/ros_gz_example_gazebo/worlds

    # Generate modified world file
    
    python3 generate_world_with_obstacles.py $DRIVE_MODE_3 $DRIVE_MODE_4 $MODIFIED_WORLD_FILE

    cd ~/inf3995/project_ws

    colcon build --cmake-args -DBUILD_TESTING=ON --packages-skip limo_base limo_msgs limo_description limo_bringup ydlidar_ros2_driver voice_control

else colcon build --cmake-args -DBUILD_TESTING=ON

fi

if [ "$ROBOT_ID" == "simulation" ]; then

    source install/setup.sh

    cleanup

    ros2 launch ros_gz_example_bringup full.launch.py
else
    echo "Invalid ROBOT_ID. Please provide ROBOT_ID as 1, 2, 'simulation', 'gazebo'."
    exit 1
fi