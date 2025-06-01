import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DriveStraightUntilWall(Node):
    def __init__(self):
        super().__init__('drive_straight_until_wall')

        # Default parameters; you can override them from the launch file if needed
        self.declare_parameter('stop_distance', 1.0)    # meters
        self.declare_parameter('forward_speed', 0.5)    # m/s

        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value

        # Publisher to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to /lidar
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'lidar',
            self.lidar_callback,
            10
        )

        self.stopped = False
        self.get_logger().info(f"Drive node started: stop_distance = {self.stop_distance:.2f} m, forward_speed = {self.forward_speed:.2f} m/s")

    def lidar_callback(self, msg: LaserScan):
        if self.stopped:
            return

        if not msg.ranges:
            return

        # Pick the center measurement in ranges[] (assumes symmetric FoV around 0).
        center_idx = len(msg.ranges) // 2
        dist_ahead = msg.ranges[center_idx]

        # Sometimes a “0.0” or NaN appears if no return; ignore those
        if dist_ahead <= 0.0:
            return

        self.get_logger().debug(f"Distance ahead: {dist_ahead:.2f} m")

        if dist_ahead > self.stop_distance:
            twist = Twist()
            twist.linear.x = float(self.forward_speed)
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
        else:
            # We are close enough; stop once
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.stopped = True
            self.get_logger().info(f"Stopping at distance = {dist_ahead:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = DriveStraightUntilWall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ensure robot is stopped on shutdown
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()