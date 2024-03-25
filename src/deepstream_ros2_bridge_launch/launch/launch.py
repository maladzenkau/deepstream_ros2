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
        parameters=[
                    {'frequency': 5},                                     # How often the latency should be estimated (in seconds)
                    {'color_image_topic': 'color/image_raw'},               # topic which serves as a DeepStream source.
                    {'topic_encoding': 'bgr8'},                             # encoding of this topic (could be found using 'ros2 topic info')
                    {'img_path': '/home/jetson/deepstream_ros2/sync.jpg'}   # path to your synchronization image
                ]
    )

    delay_topic = Node(
        package="deepstream_ros2_bridge_cpp",
        executable="delay_topic",
        name="delay_topic",
        parameters=[
                   {'topic_to_be_delayed': '/aligned_depth_to_color/image_raw'},
                   {'fps': 30} # FPS of the topic (should be set the same for both depth and color data topics)
                   ]
    )

    # DeepStream Kafka - ROS2 bridge
    ld.add_action(deepstream_kafka_ros2_bridge)
    # Latency estimation
    ld.add_action(latency_estimator)
    # Depth data synchronization with color data
    ld.add_action(delay_topic)

    return ld