import rclpy
import cv2
import time

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from deepstream_ros2_bridge_cpp.msg import ObjectDetection
from datetime import timedelta


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.declare_parameter('color_image_topic', 'color/image_raw')
        self.declare_parameter('frequency', '5')
        self.declare_parameter('img_path', '/home/jetson/deepstream_ros2/sync.jpg')
        self.declare_parameter('topic_encoding', 'bgr8')

        self.publisher_ = self.create_publisher(Image, self.get_parameter('color_image_topic').get_parameter_value().string_value, 10)
        self.latency_publisher_ = self.create_publisher(Int16, 'latency', 10)
        self.sub_info_ = self.create_subscription(ObjectDetection, 'objectDetection' , self.object_detection_callback, 1)

        timer_period = self.get_parameter('frequency').get_parameter_value().integer_value  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.bridge = CvBridge()
        img_path = self.get_parameter('img_path').get_parameter_value().string_value
        cv_image = cv2.imread(img_path, cv2.IMREAD_COLOR)
        self.msg = self.bridge.cv2_to_imgmsg(cv_image, self.get_parameter('topic_encoding').get_parameter_value().string_value)
        self.publish_time = None
        self.timer_callback()

    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.publish_time = self.get_clock().now()
    
    def object_detection_callback(self, data):
        if data.detected_object == "sync":
            subscription_time = self.get_clock().now()
            if self.publish_time is not None:
                time_difference = subscription_time - self.publish_time
                latency_msg = Int16()
                latency_msg.data = int(time_difference.nanoseconds / 1e6)  # Convert nanoseconds to milliseconds
                self.latency_publisher_.publish(latency_msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_publisher.get_logger().info('Latency estimator is up')
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