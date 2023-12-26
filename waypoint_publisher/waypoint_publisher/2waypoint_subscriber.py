import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

class WaypointSubscriber(Node):

    def __init__(self):
        super().__init__('waypoint_subscriber')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/waypoints',
            self.waypoint_callback,
            10)

    def waypoint_callback(self, marker_array_msg):
        for marker in marker_array_msg.markers:
            if marker.text == 'wp_3':
                x = marker.pose.position.x
                y = marker.pose.position.y
                z = marker.pose.position.z
                self.get_logger().info(f"wp_3 coordinates: x={x}, y={y}, z={z}")

def main(args=None):
    rclpy.init(args=args)
    waypoint_subscriber = WaypointSubscriber()
    rclpy.spin(waypoint_subscriber)
    waypoint_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

