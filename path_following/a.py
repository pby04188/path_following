#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path, Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped
from morai_msgs.msg import CtrlCmd
from ackermann_msgs.msg import AckermannDrive

from lib.utils import pathReader, findLocalPath, purePursuit, pidController, velocityPlanning, vaildObject, cruiseControl
import tf
from math import cos,sin,sqrt,pow,atan2,pi

class pid_planner():
    def __init__(self):
        rospy.init_node('pid_planner', anonymous=True)
        self.configure()
        self.is_status=False ## 차량 상태 점검

        # path data reader
        path_reader = pathReader('path_extraction') ## 경로 파일의 패키지 위치
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame) ## 출력할 경로의 이름

        # ris cmd publisher
        if (self.platform == "real"):
            self.ctrl_pub = rospy.Publisher('/ctrl_cmd', AckermannDrive, queue_size=1) ## Vehicl Control
            self.ctrl_msg = AckermannDrive()
            pass
        elif (self.platform == "sim"):
            self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1) ## Vehicl Control
            self.ctrl_msg = CtrlCmd()
        elif (self.platform == "turtlebot"):
            self.ctrl_pub = rospy.Publisher('/ctrl_cmd', Twist, queue_size=1)
            self.ctrl_msg = Twist()
        else:
            rospy.logerr("[ERROR] Wrong Platform. please check launch file arg")
            return 0

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        self.odometry_path_pub = rospy.Publisher('/odom_path', Path, queue_size=1) ## odometry history
        self.odometry_path_msg = Path()

        # ros subscriber
        rospy.Subscriber("/Ego_globalstate", Odometry, self.statusCB) ## Vehicl Status Subscriber 
        
        # class
        self.pure_pursuit = purePursuit(self.vehicle_length, self.lfd, self.min_lfd, self.max_lfd) ## purePursuit import
        self.pid = pidController(self.p_gain, self.i_gain, self.d_gain, self.control_time)
        self.cc = cruiseControl(0.5,1) ## cruiseControl import (object_vel_gain, object_dis_gain)
        ref_vel = float(self.reference_velocity)/float(3.6) # m/s
        self.vel_planner = velocityPlanning(ref_vel, self.road_friction) ## 속도 계획 (reference velocity, friciton)
        self.vel_profile = self.vel_planner.curveBasedVelocity(self.global_path,100)

        # time var
        count = 0
        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            if self.is_status==True:
                self.spin_once()

                if count == self.frequency : ## global path 출력
                    self.global_path_pub.publish(self.global_path)
                    count=0
                count+=1

            rate.sleep()
        
    def configure(self):
        self.path_file_name = rospy.get_param("~path_file_name")
        self.platform = rospy.get_param("~platform")
        self.frequency = rospy.get_param("~frequency")
        self.path_frame = rospy.get_param("~path_frame")
        self.local_path_step = rospy.get_param("~local_path_step")

        # Steering (purePursuit)
        self.vehicle_length = rospy.get_param("~vehicle_length")
        self.lfd = rospy.get_param("~initial_lfd")
        self.min_lfd = rospy.get_param("~min_lfd")
        self.max_lfd = rospy.get_param("~max_lfd")
        
        # PID Controller
        self.road_friction = rospy.get_param("~road_friction")
        self.reference_velocity = rospy.get_param("~reference_velocity")
        self.p_gain = rospy.get_param("~p_gain")
        self.i_gain = rospy.get_param("~i_gain")
        self.d_gain = rospy.get_param("~d_gain")
        self.control_time = float(1)/float(rospy.get_param("~frequency"))

    def spin_once(self):
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

        if (self.platform == "real"):
            self.ctrl_msg.steering_angle = steering_angle # steering angle (rad)
        elif (self.platform == "sim"):
            self.ctrl_msg.steering = -steering_angle # steering angle (rad)
        elif (self.platform == "turtlebot"):
            self.ctrl_msg.angular.z = velocity * sin(steering_angle) / L  # angular velocity

    def get_control_velocity(self):
        ego_current_velocity = self.status_msg.twist.twist.linear
        target_velocity = self.cc.acc(ego_current_velocity, self.vel_profile[self.current_waypoint]) ## advanced cruise control 적용한 속도 계획
        control_input = self.pid.pid(target_velocity, ego_current_velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)

        if (self.platform == "real"):
            if control_input > 0:
                self.ctrl_msg.speed = control_input # (m/s)
            else:
                self.ctrl_msg.speed = 0
                self.ctrl_msg.acceleration = 0
                self.ctrl_msg.jerk = 0
        elif (self.platform == "sim"):
            if control_input > 0 :
                self.ctrl_msg.accel= control_input # (m/s)
                self.ctrl_msg.brake= 0
            else :
                self.ctrl_msg.accel= 0
                self.ctrl_msg.brake= -control_input
        elif (self.platform == "turtlebot"):
            if control_input > 0:
                self.ctrl_msg.linear.x = control_input # (km/h)
            else :
                self.ctrl_msg.linear.x = 0
                self.ctrl_msg.linear.y = 0
                self.ctrl_msg.linear.x = 0
                self.ctrl_msg.angular.x = 0
                self.ctrl_msg.angular.y = 0
                self.ctrl_msg.angular.x = 0

    def statusCB(self, msg): ## Vehicle Status Subscriber 
        self.is_status=True

        self.status_msg = msg
        Ego_HeadingAngle = [self.status_msg.pose.pose.orientation.x, self.status_msg.pose.pose.orientation.y, self.status_msg.pose.pose.orientation.z, self.status_msg.pose.pose.orientation.w]
        
        # Map -> gps TF Broadcaster
        self.TFsender = tf.TransformBroadcaster()
        self.TFsender.sendTransform((self.status_msg.pose.pose.position.x, self.status_msg.pose.pose.position.y, 0),
                        Ego_HeadingAngle,
                        rospy.Time.now(),
                        "gps", # child frame "base_link"
                        "map") # parent frame "map"

        # Odometry history viewer
        last_point = PoseStamped()
        last_point.pose.position.x = self.status_msg.pose.pose.position.x
        last_point.pose.position.y = self.status_msg.pose.pose.position.y
        last_point.pose.position.z = 0
        last_point.pose.orientation.x = 0
        last_point.pose.orientation.y = 0
        last_point.pose.orientation.z = 0
        last_point.pose.orientation.w = 1

        self.odometry_path_msg.header.frame_id = "map"
        self.odometry_path_msg.poses.append(last_point)
        self.odometry_path_pub.publish(self.odometry_path_msg)  

    def getEgoVel(self):
        vx = self.status_msg.twist.twist.linear.x
        vy = self.status_msg.twist.twist.linear.y
        return np.sqrt(np.power(vx, 2) + np.power(vy,2))
    
if __name__ == '__main__':
    try:
        kcity_pathtracking = pid_planner()
    except rospy.ROSInterruptException:
        pass
