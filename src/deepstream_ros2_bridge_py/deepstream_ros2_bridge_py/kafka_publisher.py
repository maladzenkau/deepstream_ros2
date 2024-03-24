import rclpy
import json
import time
from rclpy.node import Node
from kafka import KafkaConsumer
from deepstream_ros2_bridge_cpp.msg import ObjectDetection

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(ObjectDetection, 'objectDetection', 10)
        timer_period = 0.12  # seconds.
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('kafka_topic', 'deepstream')
        self.declare_parameter('bootstrap_servers', '127.0.0.1:9092')
        self.declare_parameter('known_objects', ['red', 'yellow', 'green', 'rightOnly', 'leftOnly', 'stop', '50', 'noLimits', 'sync'])
        
        while True:
            try:
                self.consumer = KafkaConsumer(
                    self.get_parameter('kafka_topic').get_parameter_value().string_value,
                    bootstrap_servers = self.get_parameter('bootstrap_servers').get_parameter_value().string_value,
                    value_deserializer = lambda m: json.loads(m.decode('ascii')),
                    auto_offset_reset = 'latest'
                )
                self.get_logger().info('Kafka consumer is up')
                break
            except Exception as e:
                self.get_logger().info(f'Failed to create Kafka consumer, retrying in 5 seconds... Error: {e}')
                time.sleep(5)
                continue

    # Publish inference results if available
    def timer_callback(self):
        for message in self.consumer:
            msg = ObjectDetection()
            known_objects = self.get_parameter('known_objects').value
            detected_object = next((value for value in known_objects if value in message.value['object']))
            msg.time_stamp_kafka = message.value['@timestamp']
            msg.detected_object = detected_object
            msg.top_left_x = message.value['object']['bbox']['topleftx']
            msg.top_left_y = message.value['object']['bbox']['toplefty']
            msg.bottom_right_x = message.value['object']['bbox']['bottomrightx']
            msg.bottom_right_y = message.value['object']['bbox']['bottomrighty']
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Publishing: "Timestamp: {msg.time_stamp_kafka}", '
                                    f'Detected object:{msg.detected_object}, '
                                    f'Bounding boxes:{msg.top_left_x}, '
                                    f'{msg.top_left_y}, '
                                    f'{msg.bottom_right_x}, ' 
                                    f'{msg.bottom_right_y}" ')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
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
