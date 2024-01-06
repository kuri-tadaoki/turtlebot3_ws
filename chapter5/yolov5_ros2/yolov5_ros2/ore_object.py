import sys
import subprocess
import threading
import tkinter as tk
import cv2
import numpy as np
import time
import asyncio
import rclpy
from rclpy.node import Node
from rclpy import action
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist
from yolov5_ros2.detector import Detector, parse_opt
from playsound import playsound
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

class ObjectDetection(Node):
    
    def __init__(self, **args):
        super().__init__('object_detection')
        
        self.target_visible = False
        
        self.subscription = self.create_subscription(
            String,'waypoint_count', self.count_callback, 10)


        self.node = rclpy.create_node('goal_cancel_node')
        self.client = ActionClient(self.node, CancelGoal, 'cancel_goal')
        cancel_future = action_client.async_cancel_goal(goal_handle)


        self.pub = self.create_publisher(Twist, "/cmd_vel",qos_profile=rclpy.qos.qos_profile_system_default)   
        self.twist = Twist()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        l = 0.0
        a = 0.0
        try:
            self.linear_speed = float(l)
            self.linear_speed = float(a)
        except ValueError:
            print("Invalid input. Using default values for linear and angular speeds.")
        self.run(l,a)

        self.target_name = 'bottle'
        self.frame_id = 'target'

        self.detector = Detector(**args)

        self.bridge = CvBridge()

        self.sub_info = Subscriber(
            self, CameraInfo, 'camera/aligned_depth_to_color/camera_info')
        self.sub_color = Subscriber(
            self, Image, 'camera/color/image_raw')
        self.sub_depth = Subscriber(
            self, Image, 'camera/aligned_depth_to_color/image_raw')
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)
        self.broadcaster = TransformBroadcaster(self)

    def count_callback(self, msg):
        count_value = int(msg.data)
        self.get_logger().info(f"受信したカウント: {count_value}")
        self.count_value_list = [0] 
        self.count_value_list.append(count_value)
        self.count_value_list.pop(0)

    def cancel_goal(future):
        self.cancel_response = future.result()


    def run(self, l, a):
        self.twist.linear.x = float(l)
        self.twist.angular.z = float(a)

        self.pub.publish(self.twist)

    def images_callback(self, msg_info, msg_color, msg_depth):
        try:
            img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        if img_color.shape[0:2] != img_depth.shape[0:2]:
            self.get_logger().warn('カラーと深度の画像サイズが異なる')
            return

        img_color, result = self.detector.detect(img_color)

        #cv2.imshow('color', img_color)

        target = None
        if self.target_visible == False:
            for r in result:
                if r.name == self.target_name:
                    target = r
                    break

        if target is not None:
            self.cancel_response
            u1 = round(target.u1)
            u2 = round(target.u2)
            v1 = round(target.v1)
            v2 = round(target.v2)
            u = round((target.u1 + target.u2) / 2)
            v = round((target.v1 + target.v2) / 2)
            depth = np.median(img_depth[v1:v2+1, u1:u2+1])
            if depth != 0:
                z = depth * 1e-3
                fx = msg_info.k[0]
                fy = msg_info.k[4]
                cx = msg_info.k[2]
                cy = msg_info.k[5]
                x = z / fx * (u - cx)
                y = z / fy * (v - cy)
                self.get_logger().info(
                    f'{target.name} ({x:.3f}, {y:.3f}, {z:.3f})')
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)
               
               #ロボット制御部分
                if ts.transform.translation.x >= 0.03 and ts.transform.translation.z >=0.5:
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = -0.7
                    self.pub.publish(self.twist)
                    time.sleep(3)

                elif ts.transform.translation.x <= -0.03 and ts.transform.translation.z >=0.5:
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = 0.7
                    self.pub.publish(self.twist)
                    time.sleep(3)

                elif abs(ts.transform.translation.x) <= 0.03 and ts.transform.translation.z >=0.5:
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                    time.sleep(3)

                elif abs(ts.transform.translation.x) >= 0.03 and ts.transform.translation.z <=0.7:
                    self.twist.linear.x = -0.2
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                    time.sleep(3)



                elif abs(ts.transform.translation.x) <= 0.03 and ts.transform.translation.z <=0.5:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                    time.sleep(5)
                    #sys.exit()
                    print("fin!!")
                    self.target_visible = True

        elif self.target_visible == True:
            subprocess.Popen(["bash", "move_goal.bash", str(self.count_value_list[-1])])
            self.target_visible = False

        img_depth *= 16
        if target is not None:
            pt1 = (int(target.u1), int(target.v1))
            pt2 = (int(target.u2), int(target.v2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)

        #cv2.imshow('depth', img_depth)
        cv2.waitKey(1)

def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
