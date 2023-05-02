"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
	/aruco_poses (geometry_msgs.msg.PoseArray)
	   Pose of all detected markers (suitable for rviz visualization)

Parameters:
	marker_size - size of the markers in meters (default .0625)
	aruco_dictionary_id - dictionary that was used to generate markers
						  (default DICT_5X5_250)
	image_topic - image topic to subscribe to (default /camera/image_raw)
	camera_info_topic - camera info topic to subscribe to
						 (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import numpy as np
import cv2


class ArucoDetectionNode(Node):
	def __init__(self):
		super().__init__('aruco_detection_node')
		
		# Declare and read parameters [Aruco Marker Parameters & Camera Parameters]
		self.declare_parameter('marker_size', .055)
		self.declare_parameter('aruco_dictionary_id', 'DICT_5X5_250')
		self.declare_parameter('image_topic', '/camera/image_raw')
		self.declare_parameter('camera_info_topic', '/camera_info')
		self.declare_parameter('camera_frame', None)

		# Assign Parameters
		self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
		dictionary_id_name = self.get_parameter('aruco_dictionary_id').get_parameter_value().string_value
		image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
		info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
		self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
		
		# Dictionary ID Validation 
		try:
			dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
			if type(dictionary_id) != type(cv2.aruco.DICT_5X5_250):
				raise AttributeError
		except AttributeError:
			self.get_logger().error(f'bad aruco_dictionary_id: {dictionary_id_name}')
			options = '\n'.join([s for s in dir(cv2.aruco) if s.startswith('DICT')])
			self.get_logger().error(f'valid options: {options}')

		# Subscriptions - Camera Info & Image
		self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
		self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
		
		# Publishers - Estimated Pose Array
		self.image_pub = self.create_publisher(Image, 'aruco_detection', 10)

		# Camera Parameters Initialization
		self.info_msg = None
		self.intrinsic_mat = None
		self.distortion = None
		self.bridge = CvBridge()

		# Arcuo Parameters Initializatoin
		self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
		self.aruco_parameters = cv2.aruco.DetectorParameters_create()
		self.trigger = False
		self.last_detection = None

		# Service Server - Empty Trigger
		self.srv = self.create_service(Empty, 'trigger', self.trigger_callback)
		
		# Corresponding Service Client Call 
		# ros2 service call /trigger std_srvs/srv/Empty '{}'

	def trigger_callback(self, request, response):
		self.trigger = True
		return response

	# Camera Info - Callback
	def info_callback(self, info_msg):
		self.info_msg = info_msg
		self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
		self.distortion = np.array(self.info_msg.d)

	# Marker Detection - Callback
	def image_callback(self, img_msg):
		if self.info_msg is None:
			self.get_logger().warn('No camera info has been received!')
			return

		gray_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

		if self.last_detection is None:
			self.last_detection = np.zeros(gray_image.shape, dtype=np.uint8)

		if self.trigger:
			corners, marker_ids, rejected = cv2.aruco.detectMarkers(gray_image, self.aruco_dictionary, parameters=self.aruco_parameters)
			if len(corners) > 0:
				self.last_detection = self.draw_aruco_bbox(gray_image, corners, marker_ids)
			else:
				self.get_logger().warn('No aruco marker detected!')
			self.trigger = False

		image_msg = self.bridge.cv2_to_imgmsg(self.last_detection, encoding='bgr8')
		self.image_pub.publish(image_msg)

	def draw_aruco_bbox(self,image,corners,ids):
		# flatten the ArUco IDs list
		ids = ids.flatten()
		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned in
			# top-left, top-right, bottom-right, and bottom-left order)
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))
			# draw the bounding box of the Aruco detection
			cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
			cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)
			# compute and draw the center (x, y)-coordinates of the ArUco
			# marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			# draw the Aruco marker ID on the image
			cv2.putText(image, str(markerID),
				(cX, cY + 30), cv2.FONT_HERSHEY_SIMPLEX,
				1.0, (0, 0, 255), 3)
		# show the output image
		return image


def main():
	rclpy.init()
	node = ArucoDetectionNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()