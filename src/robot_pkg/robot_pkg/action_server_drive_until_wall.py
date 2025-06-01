import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.action import CancelResponse
from robot_pkg_interfaces.action import DriveUntilWall

from rclpy.executors import MultiThreadedExecutor

class DriveUntilWallServer(Node):
    def __init__(self):
        super().__init__('drive_until_wall_server')
        self.declare_parameter('default_stop_distance', 1.0) # default vals
        self.declare_parameter('default_forward_speed', 0.5)

        # publishr for /cmd_vel -> gazebo bridge
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.latest_scan = None # this will hold the most recent lidar reading
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'lidar',
            self.lidar_callback,
            10
        ) # creating subscription to get vals

        self._action_server = ActionServer(
            self,
            DriveUntilWall,
            'drive_until_wall',
            self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("DriveUntilWall action server started, ready for movement goals.")

    def lidar_callback(self, msg: LaserScan):
        # storing the scan so execute_callback can grab it
        self.latest_scan = msg

        # doing this to get dist forward (to not hit wall)
        center_idx = len(msg.ranges) // 2
        distance_ahead = msg.ranges[center_idx]

        if distance_ahead <= 0.0 or distance_ahead != distance_ahead:
            return

        self.get_logger().info(f"[lidar] distance ahead = {distance_ahead:.3f} m")
    
    def cancel_callback(self, goal_handle):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Received goal: stop_distance = %.2f, forward_speed = %.2f" %
                               (goal_handle.request.stop_distance, goal_handle.request.forward_speed))

        feedback_msg = DriveUntilWall.Feedback()
        result_msg = DriveUntilWall.Result()

        stop_distance = goal_handle.request.stop_distance
        forward_speed = goal_handle.request.forward_speed

        # NOTE: Couldn't get .accepted to work (come back to later?)
        # goal_handle.accepted()

        rate = self.create_rate(10)  # 10 Hz

        try:
            while rclpy.ok():
                if not goal_handle.is_active:
                    # If the goal was canceled externally, break
                    self.get_logger().info("DriveUntilWall goal canceled")
                    result_msg.success = False
                    goal_handle.canceled()
                    return result_msg

                if self.latest_scan is None:
                    # Haven't received any scan yet, so wait 1 cycle
                    rate.sleep()
                    continue

                # Center‐beam index (for front)
                center_idx = len(self.latest_scan.ranges) // 2
                distance_ahead = self.latest_scan.ranges[center_idx]

                if distance_ahead <= 0.0 or distance_ahead != distance_ahead:
                    rate.sleep()
                    continue

                # Publish feedback:
                feedback_msg.current_distance = float(distance_ahead)
                goal_handle.publish_feedback(feedback_msg)

                # Check if we need to stop:
                if distance_ahead <= stop_distance:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    # above stops the car!

                    self.get_logger().info("Goal reached: stopped at distance %.2f m" % distance_ahead)

                    result_msg.success = True
                    goal_handle.succeed()
                    return result_msg

                twist = Twist()
                twist.linear.x = float(forward_speed)
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)

                rate.sleep()
        except:
            goal_handle.canceled()

        goal_handle.canceled()
        result_msg.success = False
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    action_server = DriveUntilWallServer()

    executor = MultiThreadedExecutor(num_threads=2) # multi-threading to avoid lidar sensor not updating
    executor.add_node(action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()