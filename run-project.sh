#!/bin/bash

figlet "Running Digital Twin Project"
echo "----------------------------------------"
echo

# Function to check command success
check_command() {
    if [ $? -ne 0 ]; then
        echo "Error: Command failed."
        exit 1
    fi
}

# Start Docker containers in a new terminal
echo "Starting Docker containers"
gnome-terminal -- bash -c "cd AUC-Thesis-Leithy-DT/AUC-Thesis-DT-Physical/RemoteDrivingDashboard-master && docker-compose up; exec bash"
check_command

# Launch Qt GUI in a new terminal
echo "Launching Qt GUI"
gnome-terminal -- bash -c "cd AUC-Thesis-Leithy-DT/OTA_RemoteDrivingConfigurator-main/Designs/ && python3 QtGUI.py; exec bash"
check_command

# Run cloudconnect in a new terminal
echo "Running cloudconnect"
gnome-terminal -- bash -c "cd AUC-Thesis-Leithy-DT/ROS-master/cloudconnect/ && python3 TeleOperations.py; exec bash"
check_command

# Run actuate in a new terminal
echo "Changing directory to actuate"
gnome-terminal -- bash -c "cd AUC-Thesis-Leithy-DT/ROS-master/actuate/ && python3 actuate_singlebot.py; exec bash"
check_command

# Set TURTLEBOT3_MODEL and launch Gazebo
export TURTLEBOT3_MODEL=waffle_pi
echo "TURTLEBOT3_MODEL set to waffle_pi"

# Launch the Gazebo environment in a new terminal
gnome-terminal -- bash -c "ros2 launch turtlebot3_gazebo empty_world.launch.py; exec bash"
check_command

echo "----------------------------------------"
echo "All components launched successfully."
echo "----------------------------------------"

