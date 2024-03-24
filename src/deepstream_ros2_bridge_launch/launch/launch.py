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

    ld.add_action(deepstream_kafka_ros2_bridge)

    return ld