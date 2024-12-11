#!/bin/bash

# Function to kill all background processes
cleanup() {
    echo "Stopping all processes..."
    kill $(jobs -p)
    headhunter web_server
    exit
}

# Set up trap to catch SIGINT (Ctrl+C)
trap cleanup SIGINT

# Check if an argument is passed
if [ -n "$1" ]; then
    export ROS_DOMAIN_ID=$1
    echo "ROS_DOMAIN_ID is $ROS_DOMAIN_ID"
else
    unset ROS_DOMAIN_ID
    echo -e "ROS_DOMAIN_ID is \e[1;31mBLANK\e[0m"
fi

source ~/source_both.sh

ros2 run foxglove_bridge foxglove_bridge --ros-args -p address:=localhost &

ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/mranti_lanelet/ vehicle_model:=buggy_vehicle sensor_model:=buggy_sensor_kit &

# Wait for all background processes
wait

