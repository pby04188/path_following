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

## turtle1/Pose 메시지를 Odometry 메시지 형식으로 변환
## deepracer에서는 optitrack으로부터 PoseStamped 메시지를 받기 때문에 PoseStamped를 Odometry 메시지로 변환하는 코드가 필요함

class pose_to_odom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
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

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.callback, QOS_RKL10V)
        self.odom_pub = self.create_publisher(Odometry, "/Ego_globalstate", QOS_RKL10V)
        
    def callback(self, msg):
        ## Pose 메시지를 받는 즉시 Odometry 메시지로 변환하여 보내기 위해 callback함수에서 바로 publish
        self.odom_msg.pose.pose.position.x = msg.x
        self.odom_msg.pose.pose.position.y = msg.y
        self.odom_msg.twist.twist.linear.x = msg.linear_velocity      ## PoseStamped에는 linear, angular velocity 정보가 없으므로 이를 알아내기 위한 알고리즘이 필요
        self.odom_msg.twist.twist.angular.z = msg.angular_velocity
        
        quaternion = self.get_quaternion_from_euler(0.0, 0.0, msg.theta)
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
                
        self.odom_pub.publish(self.odom_msg)
        
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        ## Pose 메시지의 theta를 quaternion으로 변환
        ## PoseStamped 메시지에서는 각도가 quaternion으로 구성되어 있으므로 해당 함수 불필요
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