import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from robot_pkg_interfaces.action import DriveUntilWall

class Traverse_ActionClient(Node):
    def __init__(self):
        super().__init__('traverse_action')

        # Manual cmd for testing action server only: 
        # >>> ros2 action send_goal /drive_until_wall robot_pkg_interfaces/action/DriveUntilWall "{stop_distance: 1.0, forward_speed: 0.5}"
        self.client = ActionClient(self, DriveUntilWall, 'drive_until_wall')
        self.client_is_cancelled = None
        self.is_action_alive = None


    def stop_action(self):
        self.client_is_cancelled = True


    def start_action(self, stop_distance, forward_speed):
        self.get_logger().info('Starting action!')
        self.client_is_cancelled = False
        
        self.get_logger().info('Waiting for server...')
        self.client.wait_for_server()

        self.get_logger().info('Found Server!')
        goal = DriveUntilWall.Goal()
        goal.stop_distance = stop_distance
        goal.forward_speed = forward_speed

        self.get_logger().info('Sending action!')
        self.result = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        self.result.add_done_callback(self.response)


    def response(self, future):
        self.goal_handle = future.result()

        if(not self.goal_handle.accepted):
            self.get_logger().info('Goal rejected')
            return
        
        self.is_action_alive = True

        self.result_handle = self.goal_handle.get_result_async()
        self.result_handle.add_done_callback(self.process_result)


    def process_result(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')


    def feedback_cb(self, feedback_msg):
        d = feedback_msg.feedback.current_distance
        self.get_logger().info(f'd = {d} m')

        self.get_logger().info(f'client_cancelled: {self.client_is_cancelled}')
        if(self.client_is_cancelled):
            self.get_logger().info('Canceling action!')
            self.client_is_cancelled = True
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_cb)
            #rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)



    def cancel_cb(self, future):
        cancel_response = future.result()
        self.is_action_alive = False

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
    