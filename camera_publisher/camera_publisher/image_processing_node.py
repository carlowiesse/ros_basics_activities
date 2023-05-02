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
from cv_bridge import CvBridge
import numpy as np
import cv2


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        
        # Declare and read parameters [Camera Parameters]
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_frame', None)

        # Assign Parameters
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        # Subscriptions - Camera Info & Image
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        
        # Publishers - Processed Image
        self.image_pub = self.create_publisher(Image, 'new_image', 10)

        # Camera Parameters Initialization
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.bridge = CvBridge()

    # Camera Info - Callback
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

    # Image Processing - Callback
    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn('No camera info has been received!')
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        new_image = cv2.flip(cv_image, 0)
        new_image_msg = self.bridge.cv2_to_imgmsg(new_image, encoding='mono8')

        self.image_pub.publish(new_image_msg)


def main():
    rclpy.init()
    node = ImageProcessingNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()