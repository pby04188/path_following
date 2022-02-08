from math import radians
import numpy as np
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from turtlesim.msg import Pose

## turtle1/Pose 메시지를 Odometry 메시지 형식으로 변환
## deepracer에서는 optitrack으로부터 PoseStamped 메시지를 받기 때문에 PoseStamped를 Odometry 메시지로 변환하는 코드가 필요함

class pose_to_pose_stamped(Node):
    def __init__(self):
        super().__init__('pose_to_pose_stamped')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        ## Odometry 메시지 초기화
        self.pose_stamped_msg = PoseStamped()
        self.pose_stamped_msg.pose.position.x = 0.0
        self.pose_stamped_msg.pose.position.y = 0.0
        self.pose_stamped_msg.pose.position.z = 0.0
        self.pose_stamped_msg.pose.orientation.x = 0.0
        self.pose_stamped_msg.pose.orientation.y = 0.0
        self.pose_stamped_msg.pose.orientation.z = 0.0
        self.pose_stamped_msg.pose.orientation.w = 1.0

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.callback, QOS_RKL10V)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, "/Rigidbody", QOS_RKL10V)
        
    def callback(self, msg):
        ## Pose 메시지를 받는 즉시 Odometry 메시지로 변환하여 보내기 위해 callback함수에서 바로 publish
        self.pose_stamped_msg.pose.position.x = msg.x
        self.pose_stamped_msg.pose.position.y = msg.y
        
        quaternion = self.get_quaternion_from_euler(0.0, 0.0, msg.theta)
        self.pose_stamped_msg.pose.orientation.x = quaternion[0]
        self.pose_stamped_msg.pose.orientation.y = quaternion[1]
        self.pose_stamped_msg.pose.orientation.z = quaternion[2]
        self.pose_stamped_msg.pose.orientation.w = quaternion[3]
                
        self.pose_stamped_pub.publish(self.pose_stamped_msg)
        
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
        node = pose_to_pose_stamped()
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