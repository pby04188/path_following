#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
import rclpy
import numpy as np
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from path_following.lib.current_vel import get_current_vel

class pose_stamped_subscriber(Node):
    def __init__(self):
        super().__init__('pose_stamped_subscriber')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        self.declare_parameter('frequency', 20)
        self.control_time = float(1)/float(self.get_parameter("frequency").value)
        
        ## Odometry 메시지 초기화
        self.odom_msg = Odometry()
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.odom_msg.pose.covariance = [0.0]*36
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0
        self.odom_msg.twist.covariance = [0.0]*36
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        self.status = False
        
        ## Twist 메시지 초기화
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0

        self.pose_sub = self.create_subscription(PoseStamped, '/Rigidbody', self.callback, QOS_RKL10V)
        self.odom_pub = self.create_publisher(Odometry, "/Ego_globalstate", QOS_RKL10V)
        self.twist_pub = self.create_publisher(Twist, "/deepracer/current_vel", QOS_RKL10V)
        
    def callback(self, msg):
        ## Pose 메시지를 받는 즉시 Odometry 메시지로 변환하여 보내기 위해 callback함수에서 바로 publish
        yaw = self.convertQuat2Rad(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        if self.status:
            self.odom_msg.pose.pose.position.x = msg.pose.position.x
            self.odom_msg.pose.pose.position.y = msg.pose.position.y

            self.odom_msg.pose.pose.orientation.x = msg.pose.orientation.x
            self.odom_msg.pose.pose.orientation.y = msg.pose.orientation.y
            self.odom_msg.pose.pose.orientation.z = msg.pose.orientation.z
            self.odom_msg.pose.pose.orientation.w = msg.pose.orientation.w

            prev = [self.prev_x, self.prev_y, self.prev_yaw]
            current = [msg.pose.position.x, msg.pose.position.y, yaw]
            linear_vel, angular_vel = get_current_vel(prev, current, self.control_time)
            self.odom_msg.twist.twist.linear.x = linear_vel
            self.odom_msg.twist.twist.angular.z = angular_vel
            
            self.twist_msg.linear.x = linear_vel
            self.twist_msg.angular.z = angular_vel
            
            self.odom_pub.publish(self.odom_msg)
            self.twist_pub.publish(self.twist_msg)
            
        self.prev_x = msg.pose.position.x
        self.prev_y = msg.pose.position.y
        self.prev_yaw = yaw
        self.status = True
        
    # def get_quaternion_from_euler(self, roll, pitch, yaw):
    #     ## Pose 메시지의 theta를 quaternion으로 변환
    #     ## PoseStamped 메시지에서는 각도가 quaternion으로 구성되어 있으므로 해당 함수 불필요
    #     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    #     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    #     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    #     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
    #     return [qx, qy, qz, qw]
    
    def convertQuat2Rad(self, x, y, z, w):
        ysqr = y * y
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.arctan2(t3, t4)

        return Z 
       
def main():
    rclpy.init(args=None)
    try:
        node = pose_stamped_subscriber()
        try:
            while rclpy.ok():
                rclpy.spin_once(node)
                time.sleep(node.control_time)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrypt (SIGINT)')
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()