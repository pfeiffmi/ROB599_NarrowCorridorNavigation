import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.action import CancelResponse, GoalResponse
from robot_pkg_interfaces.action import DriveUntilWall

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class DriveUntilWallServer(Node):
    def __init__(self):
        super().__init__('drive_until_wall_server')
        self.stop_distance = 1.0
        self.forward_speed = 0.5

        # Centering PID variables
        self.angular_speed = 0.0
        self.center_gain = 0.5

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
            cancel_callback=self.cancel_callback,
            callback_group = ReentrantCallbackGroup()
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


    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        self.get_logger().info(f"publishing speed: {twist.linear.x}, angular: {twist.angular.z}")
        twist.angular.z = self.angular_speed
        self.cmd_pub.publish(twist)


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel callback executed")
        self.forward_speed = 0.0
        self.publish_velocity()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        #self.get_logger().info("Received goal: stop_distance = %.2f, forward_speed = %.2f" %
        #                       (goal_handle.request.stop_distance, goal_handle.request.forward_speed))

        feedback_msg = DriveUntilWall.Feedback()
        result_msg = DriveUntilWall.Result()

        self.stop_distance = goal_handle.request.stop_distance
        self.forward_speed = goal_handle.request.forward_speed

        rate = self.create_rate(5)  # 10 Hz

        self.get_logger().info("Begin execution loop")

        try:
            while rclpy.ok():
                self.get_logger().info(f"if-0: {goal_handle.is_cancel_requested}")
                if not goal_handle.is_active:
                    self.get_logger().info(f"if-1: {not goal_handle.is_active}")
                    self.get_logger().info("DriveUntilWall goal canceled")
                    result_msg.success = False
                    goal_handle.canceled()
                    return result_msg

                if self.latest_scan is None:
                    self.get_logger().info(f"if-2: {self.latest_scan is None}")
                    # Haven't received any scan yet, so wait 1 cycle
                    rate.sleep()
                    continue

                # Center‐beam index (for front)
                center_idx = len(self.latest_scan.ranges) // 2
                distance_ahead = self.latest_scan.ranges[center_idx]

                    
                left_idx, right_idx = center_idx - 80, center_idx + 80
                
                if right_idx > len(self.latest_scan.ranges): right_idx = len(self.latest_scan.ranges) - 1

                if left_idx < 0: left_idx = 0

                left_dist, right_dist = self.latest_scan.ranges[left_idx], self.latest_scan.ranges[right_idx]

                if left_dist >= 10.0: left_dist = 10.0 # defaulting to a large value
                if right_dist >= 10.0: right_dist = 10.0 # defaulting to a large value

                center_error = left_dist - right_dist

                self.angular_speed = -self.center_gain * center_error
                
                self.get_logger().info(f"Left: {left_dist:.2f} m, Right: {right_dist:.2f} m")

                if distance_ahead <= 0.0 or distance_ahead != distance_ahead:
                    self.get_logger().info(f"if-3: {distance_ahead <= 0.0 or distance_ahead != distance_ahead}")
                    rate.sleep()
                    continue

                # Publish feedback:
                feedback_msg.current_distance = float(distance_ahead)
                goal_handle.publish_feedback(feedback_msg)

                # Check if we need to stop:
                if distance_ahead <= self.stop_distance:
                    self.get_logger().info(f"if-4: {distance_ahead <= self.stop_distance}")
                    self.forward_speed = 0.0
                    self.angular_speed = 0.0 # resetting angular speed to 0 since distance has been met
                    self.publish_velocity()
                    # above stops the car!

                    self.get_logger().info("Goal reached: stopped at distance %.2f m" % distance_ahead)

                    #result_msg.success = True
                    #goal_handle.succeed()
                    #return result_msg

                self.forward_speed = float(self.forward_speed)
                self.publish_velocity()

                rate.sleep()
        except:
            pass

        result_msg.success = False
        goal_handle.succeed()
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    action_server = DriveUntilWallServer()

    executor = MultiThreadedExecutor()
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