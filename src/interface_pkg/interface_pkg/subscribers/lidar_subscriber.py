import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import cv2
import numpy as np

from sensor_msgs.msg import LaserScan

from interface_pkg.data_processing.laserscan_to_image import laserscan_to_image

from cv_bridge import CvBridge

# Lidar Subscriber Class
class Lidar_Subscriber(Node):
    def __init__(self, lidar_signals, img_dims=(500,500)):
        super().__init__("lidar_subscriber")

        # Store object variables
        self.signals = lidar_signals
        self.img_dims = img_dims

        # define the parameters for the ROS2 node subscriber
        self.lidar_sub_callback_group = MutuallyExclusiveCallbackGroup()
        # qos must match gazebo topic
        self.qos_profile = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE
        )

        # Define the subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            "/lidar",
            callback = self.listener_callback,
            callback_group = self.lidar_sub_callback_group,
            qos_profile = self.qos_profile
        )

    # Subscription callback
    def listener_callback(self, laserscan):
        image = laserscan_to_image(
            laserscan,
            robot_size_xy_m = (0.6, 0.3), 
            max_lidar_range_m = 8, 
            m_per_pxl = 0.04,
            convert_from_np_to_ROS_img = False
        )
        image = image.astype(np.uint8)
        self.signals.lidar_image.emit(image)