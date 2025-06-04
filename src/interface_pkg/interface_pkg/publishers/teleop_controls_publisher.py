import rclpy
import rclpy.node

from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class Teleop_Controls_Publisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("teleop_publisher")

        # qos must match gazebo topic
        self.qos_profile = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            qos_profile = self.qos_profile
        )


    def publish_movement(self, forward_speed_mps, turning_speed_ccw):
        twist = Twist()
        twist.linear.x = forward_speed_mps
        twist.angular.z = turning_speed_ccw
        self.get_logger().info(f"publishing speed: {twist.linear.x}, angular: {twist.angular.z}")
        self.publisher.publish(twist)