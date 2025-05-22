import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class CoverageTracker(Node):
    def __init__(self):
        super().__init__('coverage_tracker')

        self.grid_resolution = 0.1  # meters per cell
        self.visited_cells = set()
        self.robot_visited_cells = {}
        self.subscribed_robots = set()  # Keep track of already subscribed robots

        self.discover_and_subscribe_robots()

        # Log total area every 30 seconds
        self.create_timer(10.0, self.log_area_periodically)
        # Re-check for new robots every 10 seconds
        self.create_timer(10.0, self.discover_and_subscribe_robots)

    def discover_and_subscribe_robots(self):
        topic_list = self.get_topic_names_and_types()
        for topic_name, _ in topic_list:
            if topic_name.endswith('/odom'):
                parts = topic_name.strip('/').split('/')
                if len(parts) >= 2:
                    robot_name = parts[0]
                    if robot_name not in self.subscribed_robots:
                        self.get_logger().info(f'Discovered new robot: {robot_name}')
                        self.subscribed_robots.add(robot_name)
                        self.robot_visited_cells[robot_name] = set()
                        self.create_subscription(
                            Odometry,
                            f'/{robot_name}/odom',
                            self.create_callback(robot_name),
                            10
                        )

    def log_area_periodically(self):
        total_area = len(self.visited_cells) * self.grid_resolution ** 2
        self.get_logger().info(f'[Periodic] Area covered so far: {total_area:.2f} m²')
        print(f'Total area discovered = {total_area:.2f} m²')

        for robot_name, cells in self.robot_visited_cells.items():
            robot_area = len(cells) * self.grid_resolution ** 2
            self.get_logger().info(f'[{robot_name}] Total covered area: {robot_area:.2f} m²')

    def create_callback(self, robot_name):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            cell_x = math.floor(x / self.grid_resolution)
            cell_y = math.floor(y / self.grid_resolution)
            cell = (cell_x, cell_y)

            # Add to global and robot-specific sets
            self.visited_cells.add(cell)
            self.robot_visited_cells[robot_name].add(cell)

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = CoverageTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
