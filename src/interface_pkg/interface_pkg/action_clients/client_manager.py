import rclpy
import rclpy.executors
import rclpy.logging
import time

import PyQt6.QtCore as QT_core
import threading

from interface_pkg.action_clients.traverse_action_client import Traverse_ActionClient

# PyQt6_ROS2_Bridge Class
class Client_Manager(QT_core.QRunnable):
    def __init__(self):
        super().__init__()

        self.traverse_action_client = Traverse_ActionClient()
        self.run_action = False

        #self.action_thread = threading.Thread(target=rclpy.spin, args=(self.traverse_action_client,))
        #self.action_thread.start()


    # run method
    def run(self):
        while(True):
            #rclpy.logging.get_logger("test").info("Check if action is running...")
            time.sleep(0.1)
            if(self.run_action):
                # Make the action call.
                self.traverse_action_client.start_action(stop_distance=1.0, forward_speed=1.0)
                rclpy.logging.get_logger("test").info("ACTION SENT!!")

                while(rclpy.ok()):
                    time.sleep(0.05)
                    rclpy.logging.get_logger("test").info("Action in progress...")
                    rclpy.spin_once(self.traverse_action_client)
                    if(not self.run_action):
                        # Give control over to ROS2.
                        rclpy.logging.get_logger("test").info("Cancelling acton!!")
                        self.traverse_action_client.stop_action()
                        # block the code until
                        while(self.traverse_action_client.is_action_alive):
                            rclpy.spin_once(self.traverse_action_client)
                            time.sleep(0.05)
                        break


