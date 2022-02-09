import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from geometry_msgs.msg import Twist, Vector3
from math import pi, radians
from path_following.lib.utils import pidController
import time

class Core(Node):

    def __init__(self):
        super().__init__('deepracer_core')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.configure()
        
        self.target_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.sub_target_vel,
            QOS_RKL10V
        )
        
        self.current_vel_sub = self.create_subscription(
            Twist,
            '/current_vel',
            self.sub_current_vel,
            QOS_RKL10V
        )

        self.action_publisher = self.create_publisher(
            ServoCtrlMsg,
            'servo_pkg/servo_msg',
            QOS_RKL10V
        )
        
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.pid_lin_vel = pidController(self.p_gain, self.i_gain, self.d_gain, self.control_time)
        self.pid_ang_vel = pidController(self.p_gain, self.i_gain, self.d_gain, self.control_time)
        
    def configure(self):
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('min_speed', -1.5)
        self.declare_parameter('max_steer', 10)
        self.declare_parameter('min_steer', -10)
        self.declare_parameter('vehicle_length', 0.17)
        self.declare_parameter('frequency', 20)
        self.declare_parameter('p_gain', 1.0)
        self.declare_parameter('i_gain', 0.0)
        self.declare_parameter('d_gain', 0.05)
        self.declare_parameter('mode', 'keyboard')
        
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer_deg = self.get_parameter('max_steer').value
        self.min_speed = self.get_parameter('min_speed').value
        self.min_steer_deg = self.get_parameter('min_steer').value
        self.max_steer = radians(self.max_steer_deg)
        self.min_steer = radians(self.min_steer_deg)
        self.vehigle_length = self.get_parameter('vehicle_length').value
        self.frequency = self.get_parameter("frequency").value
        self.control_time = float(1)/float(self.get_parameter("frequency").value)
        self.p_gain = self.get_parameter("p_gain").value
        self.i_gain = self.get_parameter("i_gain").value
        self.d_gain = self.get_parameter("d_gain").value
        self.mode = self.get_parameter("mode").value
        
    def action_publish(self):
        #Function publishes the action and sends it to servo.
        #Args:
        #    target_steer (float): Angle value to be published to servo.
        #    target_speed (float): Throttle value to be published to servo.
        
        result = ServoCtrlMsg()
        result.angle, result.throttle = self.twist2Servo()
        self.get_logger().info(f"Publishing to servo: Steering {result.angle} | Throttle {result.throttle}")
        self.action_publisher.publish(result)

    def sub_current_vel(self, msg):
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
        
    def sub_target_vel(self, msg):
        if self.mode == 'auto':
            self.target_linear_vel = msg.linear.x
            self.target_angular_vel = msg.angular.z
            
        elif self.mode == 'keyboard':
            self.target_linear_vel = msg.linear.x * self.max_speed
            self.target_angular_vel = msg.angular.z * self.max_steer
            
        else:
            self.get_logger().warn("invalid mode")
            
    # def sub_msg(self, msg):
    #     try:
    #         lin_vel = msg.linear.x
    #         ang_vel = msg.angular.z
    #         throttle, steer = self.twist2Servo(lin_vel, ang_vel)
    #         self.get_logger().info('Subscribed Linear Velocity: {0}, Angular Velocity: {1}'.format(lin_vel, ang_vel))
    #         self.action_publish(throttle, steer)

    #     except Exception as ex:
    #         self.get_logger().error(f"Failed to publish action: {ex}")
    #         self.action_publish(0.0, 0.0)

    def twist2Servo(self):
        # steer [-1.0 , 1.0]
        # speed [-1.0 , 1.0]
        linear_vel = self.pid_lin_vel.pid(self.target_linear_vel, self.current_linear_vel)
        angular_vel = self.pid_ang_vel.pid(self.target_angular_vel, self.current_angular_vel)
        target_throttle = linear_vel / self.max_speed
        if linear_vel == 0.0:
            target_steer = self.target_angular_vel
        else:
            target_steer = (self.vehigle_length * angular_vel / linear_vel) / self.max_steer
        return target_steer, target_throttle


def main(args = None):
        rclpy.init(args=args)
        node = Core()
        try:
            while rclpy.ok():
                rclpy.spin_once(node)
                node.action_publish()
                time.sleep(node.control_time)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            

if __name__ == '__main__':
    main()
