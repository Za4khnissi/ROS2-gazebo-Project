#!/bin/bash

# Array to store PIDs
declare -a PIDS=()

cleanup() {
    echo "Cleaning up..."
    # Terminate all collected process IDs gracefully
    for PID in "${PIDS[@]}"; do
        if kill -0 $PID 2>/dev/null; then
            kill $PID
            wait $PID 2>/dev/null
        fi
    done

    # Kill any remaining npm processes
    pkill -f "npm"
    
    # Kill any remaining ROS2 processes
    pkill -f ros2

    echo "All processes terminated."
}

trap cleanup SIGINT SIGTERM EXIT

run_in_new_terminal() {
    gnome-terminal -- bash -c "$1; exec bash" &
    PIDS+=($!)
}

# Kill any existing ROS2 processes
pkill -f ros2

# Clear any stale shared memory segments
sudo rm -rf /dev/shm/*

# Install ROS 2 packages if not installed
ROS_DISTRO="humble"

export ROS_DOMAIN_ID=49

if ! grep -q "export ROS_DOMAIN_ID=49" ~/.bashrc; then
    echo 'export ROS_DOMAIN_ID=49' >> ~/.bashrc
    source ~/.bashrc
fi

git pull

echo "Checking for processes using port 3000..."
PORT_PID=$(lsof -ti:3000)
if [ ! -z "$PORT_PID" ]; then
    echo "Killing process using port 3000 (PID: $PORT_PID)"
    kill -9 $PORT_PID
fi

# Build ROS2 workspace
cd project_ws
colcon build --cmake-args -DBUILD_TESTING=ON --packages-skip limo_base limo_description limo_bringup ydlidar_ros2_driver voice_control cliff_detector
source install/setup.sh

# Install dependencies for server and client
echo "Installing server dependencies..."
cd ../server
npm install

echo "Installing client dependencies..."
cd ../client
npm install

# Return to server directory and start server in current terminal
cd ../server
echo "Starting server..."
npm start &
PIDS+=($!)

# Wait 10 seconds before launching client
sleep 10

# Launch client in new terminal
run_in_new_terminal "cd $(pwd)/../client && npm start"

# Wait for all background processes
wait