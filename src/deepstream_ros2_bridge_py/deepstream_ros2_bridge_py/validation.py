import rclpy
import cv2

from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from deepstream_ros2_bridge_cpp.msg import ObjectDetection


class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('minimal_publisher')

        self.declare_parameter('color_image_topic', 'color/image_raw')
        self.declare_parameter('depth_image_topic', 'color/image_raw')

        self.publisher_color_ = self.create_publisher(Image, 'valColorImage', 10)
        self.publisher_depth_ = self.create_publisher(Image, 'valDepthImage', 10)
        self.subscription_bboxes_ = self.create_subscription(ObjectDetection, 'objectDetection', self.bboxes_callback, 10)
        self.subscription_color_topic_ = self.create_subscription(Image, self.get_parameter('color_image_topic').get_parameter_value().string_value, self.debug_color_callback, 10)
        self.subscription_depth_topic_ = self.create_subscription(Image, self.get_parameter('depth_image_topic').get_parameter_value().string_value, self.debug_depth_callback, 10)

        self.top_left_x = None
        self.top_left_y = None
        self.bottom_right_x = None
        self.bottom_right_y = None
        self.bridge = CvBridge()

    
    def bboxes_callback(self, msg):
        self.top_left_x = msg.top_left_x
        self.top_left_y = msg.top_left_y
        self.bottom_right_x = msg.bottom_right_x 
        self.bottom_right_y = msg.bottom_right_y

    def debug_color_callback(self, msg):
        left_corner = (self.top_left_x, self.top_left_y)
        right_corner = (self.bottom_right_x, self.bottom_right_y)
        cv_bridge = CvBridge()
        val_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.rectangle(val_image, left_corner, right_corner, (0, 0, 255), 2)
        try:
            # Convert the OpenCV image back to a ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(val_image, "rgb8")
        except CvBridgeError as e:
            self.get_logger().info(f'Error: {e}')
            return
        self.publisher_color_.publish(img_msg)

    def debug_depth_callback(self, msg):
        left_corner = (self.top_left_x, self.top_left_y)
        right_corner = (self.bottom_right_x, self.bottom_right_y)
        cv_bridge = CvBridge()
        val_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        val_image_colorized = cv2.applyColorMap(cv2.convertScaleAbs(val_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.rectangle(val_image_colorized, left_corner, right_corner, (0, 0, 255), 2)
        try:
            # Convert the OpenCV image back to a ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(val_image_colorized, "rgb8")
        except CvBridgeError as e:
            self.get_logger().info(f'Error: {e}')
            return
        self.publisher_depth_.publish(img_msg)


def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_publisher.get_logger().info("Validation node is up")
    try:
        rclpy.spin(minimal_publisher)
    except (Exception, KeyboardInterrupt) as e:
        if isinstance(e,Exception):
            minimal_publisher.get_logger().info(f'"Encountered error:{e}"')
        else:
            minimal_publisher.get_logger().info("Keyboard interrupt detected, exiting...")
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
