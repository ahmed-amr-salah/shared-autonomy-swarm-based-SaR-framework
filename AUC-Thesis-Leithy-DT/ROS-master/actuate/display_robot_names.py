import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class RobotNameDisplay(Node):
    def __init__(self):
        super().__init__('robot_name_display')

        self.robot_max = 3
        self.markers_pub = self.create_publisher(Marker, '/robot_name_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        for i in range(self.robot_max):
            marker = Marker()
            marker.header.frame_id = f"robot_{i}/base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "robot_names"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 1.0  # Height above the robot
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.4  # Text height
            marker.color.a = 1.0  # Opacity
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.text = f"Robot_{i}"

            self.markers_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNameDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
