# Particle Swarm Optimization (PSO) Pattern for Robot Swarms

## Overview
The PSO pattern implements a Particle Swarm Optimization algorithm for area coverage and target discovery in robot swarms. Each robot acts as a particle in the swarm, optimizing its position based on local and global best positions while avoiding obstacles and discovering targets.

## Technical Details

### Core Components

1. **Grid-based Coverage**
   - Grid resolution: 0.5 meters per cell
   - Grid size: 20x20 meters (default, can be modified)
   - Coverage model: Linear decay from robot position to max range
   - Coverage value: 1.0 at robot position, 0.0 at max range
   - Note: The grid size is independent of the world size. The PSO algorithm will work in any world size, but the coverage tracking is done within the grid boundaries. The grid can be considered as a "coverage tracking window" that moves with the robots.

2. **Grid-World Interaction**
   - The grid is a local coverage tracking mechanism
   - Robots can explore any world size regardless of grid dimensions
   - Coverage is tracked relative to robot positions
   - Grid acts as a sliding window for coverage optimization
   - Coverage metrics are calculated within grid boundaries
   - Grid position updates with robot movement

3. **Supported World Sizes**
   - arena.world: Small arena environment
   - arena_large.world: Larger arena environment
   - empty.world: Unlimited space
   - turtle.world: Turtlebot test environment
   - 560x540m.world: Large open space
   - Ymaze.world: Maze-like environment
   - Ymaze_camber.world: Cambered maze
   - Ymaze_camber_top.world: Top-view cambered maze

4. **Grid Configuration**
   ```yaml
   # config/burger/movement_pattern/basic/pso_pattern.yaml
   pso_grid_resolution: 0.5  # meters per cell
   pso_grid_size: [20, 20]  # [width, height] in meters
   
   # Example configurations for different scenarios:
   # Small area tracking:
   pso_grid_size: [10, 10]  # 10x10 meter grid
   
   # Large area tracking:
   pso_grid_size: [50, 50]  # 50x50 meter grid
   
   # Rectangular tracking:
   pso_grid_size: [30, 20]  # 30x20 meter grid
   ```

5. **Coverage Tracking in Large Worlds**
   - Coverage is tracked in local grid around each robot
   - Grid moves with robot position
   - Coverage decay follows formula:
     ```python
     coverage = 1.0 - (distance / max_range)
     grid[i, j] = max(grid[i, j], coverage)
     ```
   - Coverage values persist in grid memory
   - Grid resets when coverage is logged
   - Coverage metrics are relative to grid size

6. **PSO Parameters**
   - Maximum sensor range (`pso_max_range`): Maximum distance for obstacle detection
   - Minimum sensor range (`pso_min_range`): Minimum safe distance from obstacles
   - Front attraction (`pso_front_attraction`): Bias towards forward movement
   - Neighborhood radius (`pso_neighborhood_radius`): Radius for local best calculation
   - Inertia weight (`pso_inertia_weight`): Weight for maintaining current velocity
   - Cognitive weight (`pso_cognitive_weight`): Weight for personal best influence
   - Social weight (`pso_social_weight`): Weight for global best influence
   - Maximum velocity (`pso_max_velocity`): Maximum allowed velocity
   - Position publish rate (`pso_position_publish_rate`): Rate of position updates

7. **Target Discovery**
   - Number of targets (`pso_num_targets`): Default 5 random targets
   - Target radius (`pso_target_radius`): Default 1.0 meters
   - Target properties: Random positions within grid bounds
   - Target discovery: Based on distance to target center

### State Management

1. **Robot State**
   - Position: [x, y] coordinates in odom frame
   - Velocity: [vx, vy] current velocity vector
   - Personal best: Best position found by this robot
   - Local best: Best position found by neighbors
   - Fitness: Combined score of coverage and target discovery

2. **Neighbor Management**
   - Dynamic neighbor tracking
   - 5-second timeout for stale neighbors
   - Position and fitness sharing between neighbors

### Movement Control

1. **Velocity Update**
   ```python
   velocity = (inertia_weight * current_velocity + 
              cognitive_weight * random() * (personal_best - current_position) +
              social_weight * random() * (local_best - current_position))
   ```

2. **Position Update**
   - Based on current velocity
   - Limited by maximum velocity
   - Obstacle avoidance using sensor data

### Coverage Calculation

1. **Grid Update**
   ```python
   coverage = 1.0 - (distance / max_range)
   grid[i, j] = max(grid[i, j], coverage)
   ```

2. **Fitness Calculation**
   - Coverage fitness: Percentage of grid covered
   - Target fitness: Number of discovered targets
   - Obstacle fitness: Distance to nearest obstacle
   - Total fitness: Weighted combination of all components

### Communication

1. **Topics**
   - `/coverage_grid`: Grid coverage data (Float32MultiArray)
   - `/neighbor_positions`: Neighbor position updates (PoseStamped)
   - `/range_data`: Sensor data for obstacle detection (RangeData)
   - `/drive_command`: Movement commands (Twist)

2. **Transforms**
   - Uses TF2 for pose tracking
   - Transforms from base_link to odom frame
   - 0.1-second timeout for transform lookups

### Error Handling

1. **Transform Errors**
   - Handles LookupException, ConnectivityException, ExtrapolationException
   - Falls back to default movement pattern
   - Logs warnings for debugging

2. **Grid Updates**
   - Bounds checking for grid coordinates
   - Exception handling for coverage updates
   - Logging of coverage metrics

### Logging

1. **Coverage Log**
   - CSV format: timestamp, coverage_percentage, targets_discovered, total_targets
   - File per robot: coverage_log_robot_X.csv
   - Regular updates during operation

2. **Debug Logs**
   - Position updates
   - Grid coverage changes
   - Target discoveries
   - Transform errors

## Usage

### Launch Parameters
```bash
ros2 launch launch_gazebo create_enviroment.launch.py \
    pattern:=pso_pattern \
    number_robots:=4 \
    robot:=burger \
    sensor_type:=lidar \
    log_level:=info \
    gazebo_world:=arena_large.world  # Specify world size
```

### Configuration
Edit the YAML configuration file in `config/burger/movement_pattern/basic/pso_pattern.yaml` to adjust:
- PSO parameters
- Grid settings
- Target properties
- Movement constraints

## Dependencies
- ROS2 Foxy
- numpy
- tf2_ros
- geometry_msgs
- std_msgs
- communication_interfaces

## Limitations
1. Grid-based coverage may not be optimal for all environments
2. Fixed grid size may not suit all scenarios
3. Transform lookup failures may affect performance
4. Target discovery is based on simple distance checks

## Future Improvements
1. Dynamic grid sizing
2. Adaptive PSO parameters
3. Improved target discovery mechanisms
4. Better obstacle avoidance
5. Distributed coverage optimization 