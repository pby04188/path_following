from math import radians
import numpy as np
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry

class pose_to_odom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        self.status = False

        self.pose_msg = Pose()
        self.odom_msg = Odometry()
        
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.callback, QOS_RKL10V)
        self.odom_pub = self.create_publisher(Odometry, "/Ego_globalstate", QOS_RKL10V)
        
    def callback(self, msg):
        self.status = True
        self.odom_msg.pose.pose.position.x = msg.x
        self.odom_msg.pose.pose.position.y = msg.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.twist.twist.linear.x = msg.linear_velocity
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = msg.angular_velocity
        
        self.yaw = radians(msg.theta)
        quaternion = self.get_quaternion_from_euler(0.0, 0.0, self.yaw)
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
        
        self.odom_msg.pose.covariance = [0.0]*36
        self.odom_msg.twist.covariance = [0.0]*36
        self.odom_pub.publish(self.odom_msg)
        
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
        
def main():
    rclpy.init(args=None)
    try:
        node = pose_to_odom()
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