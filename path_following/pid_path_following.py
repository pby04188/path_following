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

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped

from path_following.lib.utils import pathReader, findLocalPath, purePursuit, pidController, velocityPlanning, vaildObject, cruiseControl
# from tf2_ros.transform_broadcaster import TransformBroadcaster
from math import cos,sin,sqrt,pow,atan2,pi

class pid_planner(Node):
    def __init__(self):
        super().__init__('pid_planner')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.configure()
        self.is_status=False ## 차량 상태 점검

        # path data reader
        path_reader = pathReader('path_following') ## 경로 파일의 패키지 위치
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame) ## 출력할 경로의 이름

        # ris cmd publisher
        self.ctrl_pub = self.create_publisher(Twist, '/ctrl_cmd', QOS_RKL10V)
        self.ctrl_msg = Twist()

        self.global_path_pub = self.create_publisher(Path, '/global_path', QOS_RKL10V) ## global_path publisher
        self.local_path_pub = self.create_publisher(Path, '/local_path', QOS_RKL10V) ## local_path publisher
        self.odometry_path_pub = self.create_publisher(Path, '/odom_path', QOS_RKL10V) ## odometry history
        self.odometry_path_msg = Path()

        # ros subscriber
        self.status_subscriber = self.create_subscription(Odometry, "/Ego_globalstate", self.statusCB, QOS_RKL10V) ## Vehicl Status Subscriber 
        ## subscribe하는 메세지 타입이 Odometry여야함

        # class
        self.pure_pursuit = purePursuit(self.vehicle_length, self.lfd, self.min_lfd, self.max_lfd) ## purePursuit import
        self.pid = pidController(self.p_gain, self.i_gain, self.d_gain, self.control_time)
        self.cc = cruiseControl(0.5,1) ## cruiseControl import (object_vel_gain, object_dis_gain)
        ref_vel = float(self.reference_velocity)/float(3.6) # m/s
        self.vel_planner = velocityPlanning(ref_vel, self.road_friction) ## 속도 계획 (reference velocity, friciton)
        self.vel_profile = self.vel_planner.curveBasedVelocity(self.global_path,100)

        # time var
        # count = 0
        # if self.is_status==True:
        #     self.pub_local_path_ctrl_msg()

        #     if count == self.frequency:
        #         self.global_path_pub.publish(self.global_path)
        #         count=0
        #     count+=1

    def configure(self):
        # declare parameters
        self.declare_parameter('path_file_name', 'turtlebot.txt')
        self.declare_parameter('platform', 'turtlebot')
        self.declare_parameter('frequency', 20)
        self.declare_parameter('path_frame', '/odom')
        self.declare_parameter('local_path_step', 5)
        self.declare_parameter('vehicle_length', 0.28)
        self.declare_parameter('initial_lfd', 0.5)
        self.declare_parameter('min_lfd', 0.5)
        self.declare_parameter('max_lfd', 3)
        self.declare_parameter('road_friction', 0.15)
        self.declare_parameter('reference_velocity', 1.5)
        self.declare_parameter('p_gain', 1.0)
        self.declare_parameter('i_gain', 0.0)
        self.declare_parameter('d_gain', 0.05)
        
        self.path_file_name = self.get_parameter("path_file_name").value
        self.platform = self.get_parameter("platform").value
        self.frequency = self.get_parameter("frequency").value
        self.path_frame = self.get_parameter("path_frame").value
        self.local_path_step = self.get_parameter("local_path_step").value

        # Steering (purePursuit)
        self.vehicle_length = self.get_parameter("vehicle_length").value
        self.lfd = self.get_parameter("initial_lfd").value
        self.min_lfd = self.get_parameter("min_lfd").value
        self.max_lfd = self.get_parameter("max_lfd").value
        
        # PID Controller
        self.road_friction = self.get_parameter("road_friction").value
        self.reference_velocity = self.get_parameter("reference_velocity").value
        self.p_gain = self.get_parameter("p_gain").value
        self.i_gain = self.get_parameter("i_gain").value
        self.d_gain = self.get_parameter("d_gain").value
        self.control_time = float(1)/float(self.get_parameter("frequency").value)

    def pub_local_path_ctrl_msg(self):
        ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
        self.local_path, self.current_waypoint = findLocalPath(self.global_path, self.status_msg, self.path_frame, self.local_path_step)
        
        # Steering Control (steering_angle; pure pursuit control)
        self.get_steering_angle()

        # Cruise Control (control_input; velocity)
        self.get_control_velocity()

        self.local_path_pub.publish(self.local_path) ## Local Path 출력
        self.ctrl_pub.publish(self.ctrl_msg) ## Vehicl Control 출력

    def get_steering_angle(self):
        self.pure_pursuit.getPath(self.local_path) ## pure_pursuit 알고리즘에 Local path 적용
        self.pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용

        ego_current_velocity = self.status_msg.twist.twist.linear
        velocity = ego_current_velocity.x
        steering_angle = self.pure_pursuit.steering_angle()
        L = self.vehicle_length # vehicle length (m)

        self.ctrl_msg.angular.z = velocity * sin(steering_angle) / L  # angular velocity

    def get_control_velocity(self):
        ego_current_velocity = self.status_msg.twist.twist.linear
        target_velocity = self.cc.acc(ego_current_velocity, self.vel_profile[self.current_waypoint]) ## advanced cruise control 적용한 속도 계획
        control_input = self.pid.pid(target_velocity, ego_current_velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)

        if control_input > 0:
            self.ctrl_msg.linear.x = control_input # (km/h)
        else :
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.linear.y = 0.0
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.x = 0.0
            self.ctrl_msg.angular.y = 0.0
            self.ctrl_msg.angular.x = 0.0

    def statusCB(self, msg): ## Vehicle Status Subscriber 
        self.is_status=True

        self.status_msg = msg
        Ego_HeadingAngle = [self.status_msg.pose.pose.orientation.x, self.status_msg.pose.pose.orientation.y, self.status_msg.pose.pose.orientation.z, self.status_msg.pose.pose.orientation.w]
        
        # Map -> gps TF Broadcaster
        # self.TFsender = TransformBroadcaster()
        # self.TFsender.sendTransform((self.status_msg.pose.pose.position.x, self.status_msg.pose.pose.position.y, 0),
        #                 Ego_HeadingAngle,
        #                 self.get_clock().now(),
        #                 "gps", # child frame "base_link"
        #                 "map") # parent frame "map"

        # Odometry history viewer
        last_point = PoseStamped()
        last_point.pose.position.x = self.status_msg.pose.pose.position.x
        last_point.pose.position.y = self.status_msg.pose.pose.position.y
        last_point.pose.position.z = 0.0
        last_point.pose.orientation.x = 0.0
        last_point.pose.orientation.y = 0.0
        last_point.pose.orientation.z = 0.0
        last_point.pose.orientation.w = 1.0

        self.odometry_path_msg.header.frame_id = "map"
        self.odometry_path_msg.poses.append(last_point)
        self.odometry_path_pub.publish(self.odometry_path_msg)  
        count = 0
        if self.is_status==True:
            self.pub_local_path_ctrl_msg()
            self.global_path_pub.publish(self.global_path)

            # if count == self.frequency:
            #     self.global_path_pub.publish(self.global_path)
            #     count=0
            # count+=1


    def getEgoVel(self):
        vx = self.status_msg.twist.twist.linear.x
        vy = self.status_msg.twist.twist.linear.y
        return np.sqrt(np.power(vx, 2) + np.power(vy,2))

def main():
    rclpy.init(args=None)
    try:
        node = pid_planner()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrypt (SIGINT)')
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()