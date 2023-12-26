import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import subprocess
import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/waypoints',
            self.listener_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

    def listener_callback(self, marker_array_msg):
        for marker in marker_array_msg.markers:
            if marker.text == 'wp_3':
               wp_x = marker.pose.position.x
               wp_y = marker.pose.position.y
               self.get_logger().info(f"wp_3 coordinates: x={wp_x},y={wp_y}")

    def odom_callback(self, odom_msg):
        robo_x = odom_msg.pose.pose.position.x
        robo_y = odom_msg.pose.pose.position.y

       # self.get_logger().info('Robot position: d = %f' % d)

    def check_distance(self, wp_x, wp_y, robo_x, robo_y):
        # ロボットとウェイポイントの距離を計算
        distance = math.sqrt((wp_x - robo_x)**2 + (wp_y - robo_y)**2)

        self.get_logger().info('Distance to waypoint: %f' % d)


         
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

