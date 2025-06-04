# =========================
# Modified from Benjamin Narin for use in ROS2 Jazzy
# https://github.com/OSUrobotics/laser_to_image/blob/master/scripts/laser_to_image.py
# Changes: style changes, variable names, and removed additional items used for testing to have functionality only
# =========================

import numpy as np
import math
import rclpy
import rclpy.logging
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError

def laserscan_to_image(laserscan, robot_size_xy_m=(0.6, 0.3), max_lidar_range_m=10, m_per_pxl=0.08, convert_from_np_to_ROS_img=True):
	# Store maxAngle of lidar
	max_angle = laserscan.angle_max
	# Store minAngle of lidar
	min_angle = laserscan.angle_min
	# Store angleInc of lidar
	angle_inc = laserscan.angle_increment
	# Store maxLength in lidar distances
	max_length = laserscan.range_max
	# Store array of ranges
	ranges = laserscan.ranges

	# compute the total image size to fit the laser scan based on the range of the lidar and resolution of the image
	image_size = (2*max_lidar_range_m) / m_per_pxl
	image_size = int(math.ceil(image_size))

	# Calculate the number of points in array of ranges
	num_pts = len(ranges)
	# Create Array for extracting X,Y points of each data point
	xy_scan = np.zeros((num_pts,2))
	# Create 3 Channel Blank Image
	image = np.zeros((image_size,image_size,3),dtype=np.uint8)

	# Loop through all points converting distance and angle to X,Y point
	for i in range(num_pts):
		# Check that distance is not longer than it should be
		if(not (math.isnan(ranges[i]) or (ranges[i] > max_lidar_range_m))):
			# Calculate angle of point and calculate X,Y position
			angle = min_angle + float(i)*angle_inc
			xy_scan[i][0] = float(ranges[i]*math.cos(angle))
			xy_scan[i][1] = float(ranges[i]*math.sin(angle))

	# Loop through all points plot in image
	center_offset_xy = (int(image_size/2), int(image_size/2))
	rel_pxl_rc = xy_scan/m_per_pxl
	img_pxl_rc = center_offset_xy + rel_pxl_rc
	point_display_pixel_radius = 1
	for (row,col) in img_pxl_rc:
		lower_row = int(row-point_display_pixel_radius)
		upper_row = int(row+point_display_pixel_radius)
		lower_col = int(col-point_display_pixel_radius)
		upper_col = int(col+point_display_pixel_radius)
		image[lower_row:upper_row, lower_col:upper_col] = np.array([255, 255, 255])

	center_index = int(((max_angle-min_angle)/angle_inc)/2)
	rclpy.logging.get_logger("LIDAR_IMG").info(f"{image.shape}")
	rclpy.logging.get_logger("LIDAR_IMG").info(f"{ranges[center_index]}")
	rclpy.logging.get_logger("LIDAR_IMG").info(f"{xy_scan[center_index]}")
	rclpy.logging.get_logger("LIDAR_IMG").info(f"{rel_pxl_rc[center_index]}")
	rclpy.logging.get_logger("LIDAR_IMG").info(f"{center_offset_xy}")
	rclpy.logging.get_logger("LIDAR_IMG").info(f"{img_pxl_rc[center_index]}")
	'''
	for i in range(num_pts):
			pix_x = int(math.floor(center_offset_xy[0] + m_per_pxl*xy_scan[i,0]))
			pix_y = int(math.floor(center_offset_xy[1] - m_per_pxl*xy_scan[i,1]))
			if(not ((pix_x > image_size) or (pix_y > image_size))):
				image[pix_y, pix_x] = np.array([0, 0, 255])
			else:
				raise(Exception("Error in computing LaserScan Image pixel placement"))
	'''

	# place the vehicle on the image
	front_pxl = center_offset_xy[0] - (robot_size_xy_m[0]/m_per_pxl)/2
	back_pxl = center_offset_xy[0] + (robot_size_xy_m[0]/m_per_pxl)/2
	left_pxl = center_offset_xy[1] - (robot_size_xy_m[1]/m_per_pxl)/2
	right_pxl = center_offset_xy[1] + (robot_size_xy_m[1]/m_per_pxl)/2
	image[int(front_pxl):int(back_pxl+1), int(left_pxl):int(right_pxl+1)] = np.array([255, 255, 255])
	image = np.flip(image, axis=1)

	# convert to ROS image is specified
	if(convert_from_np_to_ROS_img):
		# Convert CV2 Image to ROS Message
		image = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
	
	# return the image
	return(image)
