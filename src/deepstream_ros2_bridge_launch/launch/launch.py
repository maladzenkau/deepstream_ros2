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
                    {'frequency': 5},                                       # How often the latency should be estimated (in seconds)
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

    validation = Node(
        package="deepstream_ros2_bridge_py",
        executable="validation_publisher",
        name="validation_publisher",
        parameters=[
                    {'color_image_topic': 'color/image_raw'},                   # topic which serves as a DeepStream source.
                    {'depth_image_topic': '/aligned_depth_to_color/image_raw'}, # depth topic
                ]
    )
    realsense_camera = Node(
        package="deepstream_ros2_bridge_py",
        executable="object_cam_coordinates_publisher",
        name="object_cam_coordinates_publisher",
        parameters=[
                    {'depth_image_info_topic': '/aligned_depth_to_color/camera_info'},        # depth info topic with intrinsic parameters.
                    {'depth_image_topic': '/delayed_topic/aligned_depth_to_color/image_raw'}, # delayed or original depth topic
                    {'camera_frame': 'camera_link'},                                          # camera frame according to an URDF model
                ]
    )

    ## DeepStream Kafka - ROS2 bridge
    ld.add_action(deepstream_kafka_ros2_bridge)

    ## Latency estimation
    ld.add_action(latency_estimator)

    ## Depth data synchronization with color data
    ld.add_action(delay_topic)
    #ld.add_action(validation)

    ## Real world coordinates with Realsense camera
    ld.add_action(realsense_camera)

    return ld