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

        self.wp_x = None
        self.wp_y = None

    def listener_callback(self, marker_array_msg):
        for marker in marker_array_msg.markers:
            if marker.text == 'wp_3':
               self.wp_x = marker.pose.position.x
               self.wp_y = marker.pose.position.y
               self.get_logger().info(f"wp_3 coordinates: x={self.wp_x},y={self.wp_y}")
               #self.odom_callback(wp_x, wp_y, robo_x, robo_y)
    
    def odom_callback(self, odom_msg):
        if self.wp_x is not None and self.wp_y is not None:
            robo_x = odom_msg.pose.pose.position.x
            robo_y = odom_msg.pose.pose.position.y

            distance = math.sqrt((self.wp_x -robo_x)**2 + (self.wp_y - robo_y)**2)
            self.get_logger().info('Robot position: %f' % distance)

    #def check_distance(self, wp_x, wp_y, robo_x, robo_y):
        # ロボットとウェイポイントの距離を計算
        #distance = math.sqrt((self.wp_x - self.robo_x)**2 + (self.wp_y - self.robo_y)**2)

        #self.get_logger().info('Distance to waypoint: %f' % d)


         
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

