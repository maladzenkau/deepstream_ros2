import rclpy
import sys
import os
import numpy as np
import pyrealsense2 as rs2
import time 

from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge, CvBridgeError
from deepstream_ros2_bridge_cpp.msg import ObjectDetection


if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.declare_parameter('depth_image_info_topic', '/aligned_depth_to_color/camera_info')
        self.declare_parameter('depth_image_topic', '/delayed_topic/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_frame', 'camera_link')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(msg_Image, self.get_parameter('depth_image_topic').get_parameter_value().string_value, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, self.get_parameter('depth_image_info_topic').get_parameter_value().string_value, self.imageDepthInfoCallback, 1)
        self.sub_boxes = self.create_subscription(ObjectDetection, 'objectDetection', self.bboxes_callback, 1)
        self.publisher_ = self.create_publisher(PoseStamped, 'objectCoordinateInCameraFrame', 10)

        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.top_left_x = None
        self.top_left_y = None
        self.bottom_right_x = None
        self.bottom_right_y = None

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            center_x = int((self.top_left_x + self.bottom_right_x)/2)
            center_y = int((self.top_left_y + self.bottom_right_y)/2)
            pix = (center_x, center_y)
            self.pix = pix

            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                message = PoseStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = self.get_parameter('camera_frame').get_parameter_value().string_value
                message.pose.position.z = ((result[0])/1000)
                message.pose.position.y = -(result[1])/1000
                message.pose.position.x = ((result[2])/1000)
                self.publisher_.publish(message)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return
        
    def bboxes_callback(self, msg):
        self.top_left_x = msg.top_left_x
        self.top_left_y = msg.top_left_y
        self.bottom_right_x = msg.bottom_right_x 
        self.bottom_right_y = msg.bottom_right_y

def main(args=None):
    rclpy.init(args=args)
    listener = ImageListener()
    listener.get_logger().info('Object coordinates in camera frame publisher node is up')
    try:
        rclpy.spin(listener)
    except (Exception, KeyboardInterrupt) as e:
        if isinstance(e,Exception):
            listener.get_logger().info(f'"Encountered error:{e}"')
        else:
            listener.get_logger().info("Keyboard interrupt detected, exiting...")
        listener.destroy_node()
        rclpy.shutdown()

