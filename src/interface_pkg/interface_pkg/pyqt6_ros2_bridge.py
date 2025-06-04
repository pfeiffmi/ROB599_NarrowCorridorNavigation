import rclpy
import rclpy.executors

import PyQt6.QtCore as QT_core

from interface_pkg.signals.lidar_signals import Lidar_Signals
from interface_pkg.subscribers.lidar_subscriber import Lidar_Subscriber
from interface_pkg.publishers.teleop_controls_publisher import Teleop_Controls_Publisher

# PyQt6_ROS2_Bridge Class
class PyQt6_ROS2_Bridge(QT_core.QRunnable):
    def __init__(self):
        
        super().__init__()

        self.lidar_signals = Lidar_Signals()
        self.lidar_subscriber = None

        self.teleop_controls_publisher = Teleop_Controls_Publisher()


    # run method
    def run(self):
        self.lidar_signals.start.emit()
        
        # Instantiate a single-threaded executor
        self.singlethread_executor = rclpy.executors.SingleThreadedExecutor()
        
        # Add the Lidar subscriber to the executor
        self.lidar_subscriber = Lidar_Subscriber(
            lidar_signals = self.lidar_signals, 
        )

        # Add the teleop controls publisher to the executor
        #self.teleop_controls_publisher = Teleop_Controls_Publisher()

        # add the nodes to the executor
        self.singlethread_executor.add_node(self.lidar_subscriber)
        self.singlethread_executor.add_node(self.teleop_controls_publisher)

        # Start the multithread executor
        self.singlethread_executor.spin()

        # Shudtown the nodes before exiting the bridge
        self.lidar_subscriber.destroy_node()
        rclpy.shutdown()


