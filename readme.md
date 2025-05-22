# Swarm-Behavior-Package

This package provides swarm behavior implementations for ROS 2 robots, supporting various robot types including TurtleBot3 and Thymio.

## Required Software

### Foxy Version
- Ubuntu 20.04 LTS
- ROS 2 Foxy Fitzroy
- ROS 2 TurtleBot3 package
- ROS 2 Thymio package
- Python 3.8.10
- Gazebo 11 for simulation
- Docker Desktop

# Installation guide

This guide is a detailed step by step instruction to install the Swarm-Behavior-Package on top of a Ubuntu 20.04 OS for the use with 
- the TurtleBot 3
- the Jackal UGV or
- the Thymio II or
- the AgileX Limo. 

It is based on the guide to set up a turtlebot3 development environment and uses the manual install of
- https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
- https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/


## Prerequirements

This guide expects that the OS Ubuntu Bionic 20.04 is already installed.

### Install ROS 2 Foxy
Install ROS 2 desktop version following:

https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
Setup Sources
```
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Install ROS 2 packages
```
sudo apt update
sudo apt install ros-foxy-desktop
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
```
Install argcomplete (optional)
```
sudo apt install -y python3-pip
pip3 install -U argcomplete
```
Test if installation were successful
new Terminal
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```
### Install Dependent ROS 2 Packages, including gazebo

Install colcon
```
sudo apt install python3-colcon-common-extensions
```

Install Gazebo
```
sudo apt install ros-foxy-gazebo-ros-pkgs
```

Install Cartographer
```
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
```

Install Navigation2
```
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```
<!--
Install vcstool
```
sudo apt install python3-vcstool
```
-->
### Installation of TurtleBot3 Support 

Install TurtleBot3 Packages
<!--
```
source ~/.bashrc
sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3
```
-->

TurtleBot3 packages with source code:
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
sudo apt install ros-foxy-dynamixel-sdk
cd ~/turtlebot3_ws && colcon build --symlink-install
```

Environment Configuration
```
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
```

Install turtlebot3_simulation package
```
cd ~/turtlebot3_ws/src/
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Set the gazebo model path
```
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
```

build turtlebot3 packages
```
cd ~/turtlebot3_ws && colcon build --symlink-install
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc
```

Test if example simulation works
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Installation of the ROS2swarm package

place the project folder in your home directory (also required for using the scripts)
```
git git@github.com:ahmed-amr-salah/shared-autonomy-swarm-based-SaR-framework.git
cd Swarm-Behavior-Package/
colcon build --symlink-install
echo 'source ~/Swarm-Behavior-Package/install/setup.bash' >> ~/.bashrc
```

Test if Swarm Behavior Package package starts a simulation
```
source ~/.bashrc
bash ~/Swarm-Behavior-Package/restart.sh
```

## Running the Package

### Starting a Robot

Use the `start_robot.sh` script to launch a single robot:

```bash
./start_robot.sh
```

Parameters:
- `pattern`: Behavior pattern to use
  - Movement patterns: drive_pattern, dispersion_pattern, aggregation_pattern, flocking_pattern, flocking_pattern2, attraction_pattern, attraction_pattern2, magnetometer_pattern, minimalist_flocking_pattern, discussed_dispersion_pattern, beeclust_pattern
  - Voting patterns: voter_model_pattern, voter_model_with_limiter_pattern, majority_rule_pattern
- `robot`: Robot type (waffle_pi, burger, jackal, thymio)
- `sensor_type`: Sensor type (lidar, ir, ir_tf)
- `robot_number`: Number of robots

### Starting a Simulation

Use the `start_simulation.sh` script to launch a simulation:

```bash
./start_simulation.sh
```

Parameters:
- `gazebo_world`: World file to use (arena_large.world, arena.world, empty.world, turtle.world, 560x540m.world, Ymaze.world, Ymaze_camber.world, Ymaze_camber_top.world)
- `pattern`: Behavior pattern to use
- `number_robots`: Number of robots
- `total_robots`: Total number of robots for heterogeneous swarm
- `robot`: Robot type (burger, waffle_pi, jackal, thymio)
- `sensor_type`: Sensor type (lidar, ir, ir_tf)
- `driving_swarm`: Enable/disable driving swarm framework (true/false)
- `logging`: Enable/disable logging (true/false)

### Sending Commands

Use the `start_command.sh` script to send commands to the swarm:

```bash
./start_command.sh
```

This script publishes commands to the `/swarm_command` topic.

## Adding Robots to Simulation

To add heterogeneous swarm/robots:
```bash
bash add_robots_to_simulation.sh
```

# Digital Twin Simulation 

Project Setup Steps
===================

Follow these steps to get the project running:

1\. Start Docker Service
------------------------
```bash
systemctl start docker
docker-compose up
```


2\. Setup Remote Driving Dashboard
----------------------------------

```bash
cd AUC-Thesis-Leithy-DT/AUC-Thesis-DT-Physical/RemoteDrivingDashboard-master
export HOST_IP=localhost
# Note: You can also source the bashrc as HOST_IP is included inside it
docker-compose up
```


3\. Setup OTA Remote Driving Configurator
-----------------------------------------

```bash
cd AUC-Thesis-Leithy-DT/OTA_RemoteDrivingConfigurator-main/Designs
export HOST_IP=localhost
python3 QtGUI.py `
```

4\. Run Cloud Connect Service
-----------------------------

```bash
cd AUC-Thesis-Leithy-DT/ROS-master/cloudconnect
python3 TeleOperations.py
```

5\. Run Actuate Service
-----------------------

```bash
cd AUC-Thesis-Leithy-DT/ROS-master/actuate
python3 actuate.py
```

6\. Access the Dashboard
------------------------
Once all services are running, you can access the dashboard at:
```bash
http://localhost:8000/ 
```

Notes
-----

*   Make sure Docker is installed and running before starting
    
*   Ensure all Python dependencies are installed for the Python scripts
    
*   All commands should be run from the project root directory
    
*   The services need to be running simultaneously for the full system to work
```
