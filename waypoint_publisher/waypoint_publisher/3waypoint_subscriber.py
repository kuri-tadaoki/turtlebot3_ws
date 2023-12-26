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
            #wp = marker.text
            #wp_x = marker.pose.position.x
            #wp_y = marker.pose.position.y
            if marker.text == 'wp_3':
               x = marker.pose.position.x
               y = marker.pose.position.y
               self.get_logger().info(f"wp_3 coordinates: x={x},y={y}")
               self.check_distance(wp_x, wp_y)
            #self.get_logger().info('Received waypoint: "%s"' % wp)
            #self.check_distance(wp_x, wp_y)

    def odom_callback(self, odom_msg):
        robo_x = odom_msg.pose.pose.position.x
        robo_y = odom_msg.pose.pose.position.y

        self.get_logger().info('Robot position: x=%f, y=%f' % (robo_x, robo_y))

    def check_distance(self, wp_x, wp_y):
        # ロボットとウェイポイントの距離を計算
        robo_x, robo_y = self.get_robot_position()
        distance = math.sqrt((x - robo_x)**2 + (y - robo_y)**2)

        self.get_logger().info('Distance to waypoint: %f' % distance)

        if distance <= 0.3:
            subprocess.run(["bash", "test2.bash"])
            subprocess.run(["bash", "test3.bash"])

    def get_robot_position(self):
        # ここでロボットの現在の位置を取得するロジックを実装
        # odomコールバック関数で取得したデータを利用するなど
        # 仮の値を返しているため、実際のロジックに置き換える必要があります
        return 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

