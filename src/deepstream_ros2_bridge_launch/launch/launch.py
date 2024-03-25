from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    deepstream_kafka_ros2_bridge = Node(
        package="deepstream_ros2_bridge_py",
        executable="kafka_publisher",
        name="kafka_publisher",
        # Parameters to create a Kafka consumer with the information specified for Kafka server and topic in the deepstream_app_config.txt
        parameters=[
                    {'kafka_topic': 'deepstream'},
                    {'bootstrap_servers': '127.0.0.1:9092'},
                    {'known_objects': ['red', 'yellow', 'green', 'rightOnly', 'leftOnly', 'stop', '50', 'noLimits', 'sync']} # Classes from label.txt
                ]
    )
    latency_estimator = Node(
        package="deepstream_ros2_bridge_py",
        executable="latency_publisher",
        name="latency_estimator",
        # Parameters to create a Kafka consumer with the information specified for Kafka server and topic in the deepstream_app_config.txt
        parameters=[
                    {'frequency': '5'}, # How often the latency should be estimated (in seconds)
                    {'color_image_topic': 'color/image_raw'}, # topic which serves as a DeepStream source.
                    {'topic_encoding': 'bgr8'}, # encoding of this topic (could be found using 'ros2 topic info')
                    {'img_path': '/home/jetson/deepstream_ros2/sync.jpg'} # path to your synchronization image
                ]
    )

    # DeepStream Kafka - ROS2 bridge
    ld.add_action(deepstream_kafka_ros2_bridge)

    # Depth and color data syncronization
    ld.add_action(latency_estimator)

    return ld