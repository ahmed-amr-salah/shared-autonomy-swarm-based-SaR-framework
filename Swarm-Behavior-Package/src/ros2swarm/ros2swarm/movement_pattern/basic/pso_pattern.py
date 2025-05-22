#!/usr/bin/env python3
#    Copyright 2024 Marian Begemann
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from ros2swarm.utils import setup_node
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import random
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import tf2_ros
from std_msgs.msg import Bool

class PSOPattern(MovementPattern):
    """
    Pattern implementing Particle Swarm Optimization for area coverage and target discovery.
    
    The PSO pattern node creates drive commands based on:
    1. Local best positions from neighbors
    2. Current position and velocity
    3. Sensor data for obstacle detection
    4. Dynamic target discovery
    
    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """Initialize the PSO pattern node."""
        super().__init__('pso_pattern')
        
        # Add autonomous mode subscription
        self.mode_subscription = self.create_subscription(
            Bool,
            self.get_namespace() + '/Mode',
            self.mode_callback,
            10
        )
        
        self.autonomous = True  # Start in autonomous mode
        self.get_logger().info('PSO Pattern initialized in autonomous mode')
        
        # Create coverage log file
        self.coverage_log_file = f'coverage_log_{self.get_namespace().replace("/", "_")}.csv'
        with open(self.coverage_log_file, 'w') as f:
            f.write('timestamp,coverage_percentage,targets_discovered,total_targets\n')
        
        # Declare PSO parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pso_max_range', 0.0),  # Maximum sensor range
                ('pso_min_range', 0.0),  # Minimum sensor range
                ('pso_front_attraction', 0.0),  # Forward bias
                ('pso_neighborhood_radius', 0.0),  # Radius for local best calculation
                ('pso_inertia_weight', 0.0),  # PSO inertia weight
                ('pso_cognitive_weight', 0.0),  # PSO cognitive weight
                ('pso_social_weight', 0.0),  # PSO social weight
                ('pso_max_velocity', 0.0),  # Maximum velocity
                ('pso_position_publish_rate', 1.0),  # Rate to publish position updates
                ('pso_grid_resolution', 0.5),  # Grid cell size in meters
                ('pso_num_targets', 5),  # Number of random targets
                ('pso_target_radius', 1.0),  # Radius of target areas
                ('max_translational_velocity', 0.0),
                ('max_rotational_velocity', 0.0)
            ])

        # Initialize PSO state
        self.position = np.zeros(2)  # Current position [x, y]
        self.velocity = np.zeros(2)  # Current velocity [vx, vy]
        self.pbest = np.zeros(2)  # Personal best position
        self.pbest_fitness = float('inf')  # Personal best fitness
        self.neighbors = {}
        self.lbest = None  # Local best position
        self.lbest_fitness = float('inf')  # Local best fitness
        self.last_position_update = Time()  # Last time position was published
        
        # Initialize grid for area coverage
        self.grid_resolution = self.get_parameter("pso_grid_resolution").get_parameter_value().double_value
        self.grid_size = (20, 20)  # 20x20 meter area
        self.grid = np.zeros((int(self.grid_size[0]/self.grid_resolution), 
                            int(self.grid_size[1]/self.grid_resolution)))
        
        # Log grid initialization
        self.get_logger().info(
            f'Grid initialized - Size: {self.grid.shape}, '
            f'Resolution: {self.grid_resolution}m, '
            f'Area: {self.grid_size[0]}x{self.grid_size[1]}m'
        )
        
        # Initialize random targets
        self.num_targets = self.get_parameter("pso_num_targets").get_parameter_value().integer_value
        self.target_radius = self.get_parameter("pso_target_radius").get_parameter_value().double_value
        self.targets = self.generate_random_targets()
        self.discovered_targets = set()
        
        # Initialize TF2 for pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to range data
        self.range_data_subscription = self.create_subscription(
            RangeData,
            self.get_namespace() + '/range_data',
            self.swarm_command_controlled(self.range_data_callback),
            qos_profile=qos_profile_sensor_data
        )
        
        # Subscribe to neighbor positions
        self.neighbor_subscription = self.create_subscription(
            PoseStamped,
            self.get_namespace() + '/neighbor_positions',
            self.neighbor_callback,
            10
        )
        
        # Create publisher for position updates
        self.position_publisher = self.create_publisher(
            PoseStamped,
            self.get_namespace() + '/position',
            10
        )
        
        # Create timer for position updates
        self.position_timer = self.create_timer(
            1.0 / self.get_parameter("pso_position_publish_rate").get_parameter_value().double_value,
            self.publish_position
        )
        
        # Load parameters
        self.param_max_range = float(
            self.get_parameter("pso_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "pso_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "pso_front_attraction").get_parameter_value().double_value
        self.param_neighborhood_radius = self.get_parameter(
            "pso_neighborhood_radius").get_parameter_value().double_value
        self.param_inertia_weight = self.get_parameter(
            "pso_inertia_weight").get_parameter_value().double_value
        self.param_cognitive_weight = self.get_parameter(
            "pso_cognitive_weight").get_parameter_value().double_value
        self.param_social_weight = self.get_parameter(
            "pso_social_weight").get_parameter_value().double_value
        self.param_max_velocity = self.get_parameter(
            "pso_max_velocity").get_parameter_value().double_value
        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
            
        # Log parameter initialization
        self.get_logger().info(
            f'Parameters initialized - Max Range: {self.param_max_range}m, '
            f'Min Range: {self.param_min_range}m, '
            f'Grid Resolution: {self.grid_resolution}m'
        )

    def generate_random_targets(self):
        """Generate random target areas with specific properties."""
        targets = []
        for _ in range(self.num_targets):
            # Generate random position within grid bounds
            x = random.uniform(-self.grid_size[0]/2, self.grid_size[0]/2)
            y = random.uniform(-self.grid_size[1]/2, self.grid_size[1]/2)
            # Generate random property value (e.g., intensity)
            property_value = random.uniform(0.5, 1.0)
            targets.append({
                'position': np.array([x, y]),
                'radius': self.target_radius,
                'property': property_value
            })
        return targets

    def log_coverage_metrics(self):
        """Log coverage metrics to a dedicated file."""
        timestamp = self.get_clock().now().to_msg().sec
        coverage_percentage = np.mean(self.grid) * 100
        targets_discovered = len(self.discovered_targets)
        
        with open(self.coverage_log_file, 'a') as f:
            f.write(f'{timestamp},{coverage_percentage:.2f},{targets_discovered},{self.num_targets}\n')
        
        # Also print to console in a clear format
        self.get_logger().info(
            f'COVERAGE METRICS - Coverage: {coverage_percentage:.2f}% | '
            f'Targets: {targets_discovered}/{self.num_targets}'
        )
        
        # Publish grid data for visualization
        grid_msg = Float32MultiArray()
        grid_msg.layout.dim.append(MultiArrayDimension())
        grid_msg.layout.dim.append(MultiArrayDimension())
        grid_msg.layout.dim[0].label = "height"
        grid_msg.layout.dim[0].size = self.grid.shape[0]
        grid_msg.layout.dim[0].stride = self.grid.shape[0] * self.grid.shape[1]
        grid_msg.layout.dim[1].label = "width"
        grid_msg.layout.dim[1].size = self.grid.shape[1]
        grid_msg.layout.dim[1].stride = self.grid.shape[1]
        grid_msg.data = self.grid.flatten().tolist()
        self.coverage_publisher.publish(grid_msg)

    def update_grid_coverage(self, position):
        """Update the grid coverage based on current position."""
        try:
            # Convert position to grid coordinates
            grid_x = int((position[0] + self.grid_size[0]/2) / self.grid_resolution)
            grid_y = int((position[1] + self.grid_size[1]/2) / self.grid_resolution)
            
            # Debug log the position and grid coordinates
            self.get_logger().debug(f'Position: {position}, Grid coords: ({grid_x}, {grid_y})')
            
            # Update grid cells within sensor range
            sensor_range_cells = int(self.param_max_range / self.grid_resolution)
            cells_updated = 0
            
            # Ensure grid coordinates are within bounds
            if not (0 <= grid_x < self.grid.shape[0] and 0 <= grid_y < self.grid.shape[1]):
                self.get_logger().warn(f'Position {position} maps to out-of-bounds grid coordinates ({grid_x}, {grid_y})')
                return
            
            # Update cells in a circular pattern around the robot
            for i in range(max(0, grid_x - sensor_range_cells), 
                          min(self.grid.shape[0], grid_x + sensor_range_cells + 1)):
                for j in range(max(0, grid_y - sensor_range_cells), 
                              min(self.grid.shape[1], grid_y + sensor_range_cells + 1)):
                    # Calculate distance to cell center
                    cell_center = np.array([
                        (i * self.grid_resolution) - self.grid_size[0]/2 + self.grid_resolution/2,
                        (j * self.grid_resolution) - self.grid_size[1]/2 + self.grid_resolution/2
                    ])
                    distance = np.linalg.norm(cell_center - position)
                    
                    # Update coverage based on distance
                    if distance <= self.param_max_range:
                        # Linear coverage model: 1.0 at robot position, 0.0 at max range
                        coverage = 1.0 - (distance / self.param_max_range)
                        # Update cell with maximum coverage value
                        self.grid[i, j] = max(self.grid[i, j], coverage)
                        cells_updated += 1
            
            # Log coverage update
            coverage_percentage = np.mean(self.grid) * 100
            self.get_logger().info(
                f'Coverage Update - Position: {position}, '
                f'Cells Updated: {cells_updated}, '
                f'Total Coverage: {coverage_percentage:.2f}%'
            )
            
            # Publish grid data for visualization
            grid_msg = Float32MultiArray()
            grid_msg.layout.dim.append(MultiArrayDimension())
            grid_msg.layout.dim.append(MultiArrayDimension())
            grid_msg.layout.dim[0].label = "height"
            grid_msg.layout.dim[0].size = self.grid.shape[0]
            grid_msg.layout.dim[0].stride = self.grid.shape[0] * self.grid.shape[1]
            grid_msg.layout.dim[1].label = "width"
            grid_msg.layout.dim[1].size = self.grid.shape[1]
            grid_msg.layout.dim[1].stride = self.grid.shape[1]
            grid_msg.data = self.grid.flatten().tolist()
            self.coverage_publisher.publish(grid_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error updating grid coverage: {e}')
            raise

    def check_target_discovery(self, position):
        """Check if any targets are discovered at current position."""
        for i, target in enumerate(self.targets):
            if i not in self.discovered_targets:
                distance = np.linalg.norm(position - target['position'])
                if distance <= target['radius']:
                    self.discovered_targets.add(i)
                    self.get_logger().info(f'Target {i} discovered at position {target["position"]}')

    def calculate_fitness(self, range_data):
        """Calculate fitness based on area coverage and target discovery."""
        # Update grid coverage
        self.update_grid_coverage(self.position)
        
        # Check for target discovery
        self.check_target_discovery(self.position)
        
        # Calculate coverage fitness (percentage of grid covered)
        coverage_fitness = np.mean(self.grid)
        
        # Calculate target discovery fitness
        target_fitness = len(self.discovered_targets) / self.num_targets
        
        # Calculate obstacle avoidance fitness
        obstacle_fitness = 0.0
        if range_data is not None:
            # Check if any obstacles are too close
            min_range = min(range_data.ranges)
            if min_range < self.param_min_range:
                obstacle_fitness = 1.0
            elif min_range < self.param_max_range:
                obstacle_fitness = (min_range - self.param_min_range) / (self.param_max_range - self.param_min_range)
        
        # Combine fitness components
        # Higher weight for target discovery, then coverage, then obstacle avoidance
        total_fitness = (0.5 * (1.0 - target_fitness) +  # Lower is better for PSO
                        0.3 * (1.0 - coverage_fitness) + 
                        0.2 * obstacle_fitness)
        
        return total_fitness

    def publish_position(self):
        """Publish current position and fitness to neighbors."""
        if not self.autonomous:
            return  # Don't publish position updates in manual mode
            
        if not hasattr(self, 'position') or self.position is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(self.position[0])
        msg.pose.position.y = float(self.position[1])
        msg.pose.position.z = float(self.pbest_fitness)  # Use z coordinate for fitness
        self.position_publisher.publish(msg)

    def mode_callback(self, msg):
        """Handle mode changes between autonomous and manual control."""
        self.autonomous = msg.data
        self.get_logger().info(f'PSO Pattern mode changed to: {"Autonomous" if self.autonomous else "Manual"}')
        
        if not self.autonomous:
            # Stop any ongoing PSO calculations
            self.velocity = np.zeros(2)
            # Publish zero velocity to stop the robot
            direction = Twist()
            direction.linear.x = 0.0
            direction.angular.z = 0.0
            self.command_publisher.publish(direction)
            self.get_logger().info('PSO Pattern published zero velocity command')
            
            # Reset PSO state
            self.position = None
            self.pbest = None
            self.pbest_fitness = float('inf')
            self.lbest = None
            self.lbest_fitness = float('inf')

    def range_data_callback(self, incoming_msg):
        """Process incoming range data and update PSO state."""
        if not self.autonomous:
            return  # Don't process PSO updates in manual mode
            
        try:
            # Get namespace without leading slash and construct frame ID
            namespace = self.get_namespace().lstrip('/')
            frame_id = f'{namespace}/base_link'
            
            # Try to get transform with timeout
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom',
                    frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))  # Add timeout
                
                self.position = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y
                ])
                
                # Update grid coverage with new position
                self.update_grid_coverage(self.position)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # If transform fails, use a default movement pattern
                direction = Twist()
                direction.linear.x = self.param_max_translational_velocity
                direction.angular.z = 0.0
                self.command_publisher.publish(direction)
                return
                
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            # If transform fails, use a default movement pattern
            direction = Twist()
            direction.linear.x = self.param_max_translational_velocity
            direction.angular.z = 0.0
            self.command_publisher.publish(direction)
            return

        # Calculate movement based on PSO
        direction = self.vector_calc(incoming_msg)
        self.command_publisher.publish(direction)

    def neighbor_callback(self, msg):
        """Process incoming neighbor position updates."""
        # Extract neighbor ID from topic name
        neighbor_id = msg.header.frame_id
        
        # Update neighbor information
        neighbor_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        neighbor_fitness = msg.pose.position.z
        
        # Update neighbors dictionary
        self.neighbors[neighbor_id] = {
            'position': neighbor_pos,
            'fitness': neighbor_fitness,
            'last_update': self.get_clock().now()
        }
        
        # Remove old neighbors (not updated in last 5 seconds)
        current_time = self.get_clock().now()
        self.neighbors = {
            k: v for k, v in self.neighbors.items()
            if (current_time - v['last_update']).nanoseconds < 5e9
        }
        
        # Update local best if needed
        if neighbor_fitness < self.lbest_fitness:
            self.lbest = neighbor_pos
            self.lbest_fitness = neighbor_fitness

    def vector_calc(self, range_data):
        """Calculate the PSO movement vector."""
        if not self.autonomous:
            return Twist()  # Return zero velocity in manual mode
            
        if range_data is None:
            return Twist()

        # Calculate fitness based on area coverage and target discovery
        current_fitness = self.calculate_fitness(range_data)
        
        # Update personal best if needed
        if current_fitness < self.pbest_fitness:
            self.pbest = self.position.copy()
            self.pbest_fitness = current_fitness

        # Update velocity using PSO equations
        cognitive = self.param_cognitive_weight * np.random.rand(2) * (self.pbest - self.position)
        social = self.param_social_weight * np.random.rand(2) * (self.lbest - self.position)
        
        self.velocity = (self.param_inertia_weight * self.velocity + 
                        cognitive + social)
        
        # Limit velocity
        velocity_norm = np.linalg.norm(self.velocity)
        if velocity_norm > self.param_max_velocity:
            self.velocity = self.velocity * (self.param_max_velocity / velocity_norm)

        # Convert velocity to Twist message
        direction = Twist()
        direction.linear.x = self.velocity[0]
        direction.angular.z = self.velocity[1]
        
        return direction

def main(args=None):
    """Create a node for the PSO pattern and handle the setup."""
    setup_node.init_and_spin(args, PSOPattern)

if __name__ == '__main__':
    main() 