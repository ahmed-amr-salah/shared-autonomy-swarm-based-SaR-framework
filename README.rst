# Swarm-Behavior-Package

This package provides swarm behavior implementations for ROS 2 robots, supporting various robot types including TurtleBot3 and Thymio.

## Required Software

### Dashing Version
- Ubuntu 18.04 LTS
- ROS 2 Dashing Diademata
- ROS 2 TurtleBot3 package
- Python 3.6
- Gazebo 9 for simulation

### Foxy Version
- Ubuntu 20.04 LTS
- ROS 2 Foxy Fitzroy
- ROS 2 TurtleBot3 package
- ROS 2 Thymio package
- Python 3.8.10
- Gazebo 11 for simulation

## Robot Models

### Using the Thymio Model and Modified TurtleBot3 Models

A Gazebo model for the Thymio~II robot is provided at: [thymio_description](https://github.com/ROS2swarm/thymio_description)

ROS2swarm includes meshes for modified Turtlebot3 models.

### Setting Up Thymio~II in Gazebo

1. Download the thymio_description package to your workspace:
```bash
cd ~/colcon_ws/src/
git clone https://github.com/ROS2swarm/thymio_description.git
```

2. Build the workspace:
```bash
cd ~/colcon_ws
colcon build --symlink-install
```

3. Use the robot selection parameter in start scripts:
```bash
robot:=thymio
sensor_type:=ir
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

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.
