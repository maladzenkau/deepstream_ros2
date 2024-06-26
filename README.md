# deepstream_ros2 (Pre-reliese phase)
This project allows bridging DeepStream inference results (rectangular boxes) with ROS2 in its basic functionality.

Also, if you wish to utilize a ROS2 topic of type `sensor_msgs::msg::Image` for object detection, you can employ the [image2rtsp](https://github.com/45kmh/image2rtsp) package to buffer the topic into an RTSP stream. With a ROS2 topic as DeepStream input, additional functionality can be used to estimate the latency (which appears due to the inference processing time), which can be used to synchronize depth and color data or to estimate the load state of your device. For the [Intel® RealSense™ Depth Camera D435i](https://www.intelrealsense.com/depth-camera-d435i/), it is also implemented the extraction of real-world coordinates relative to the camera frame and their transformation into the map frame. (Note: The camera frame should be included in the `URDF` description.)

The development is being carried out on Ubuntu 20.04 with ROS2 Foxy at Jetson ORIN NX 16GB running JetPack 5.1.2 (DeepStream 6.3).

## To Do:
Test with ROS2 Humble

## 1. DeepStream Kafka - ROS2 bridge
## Dependencies
- ROS2 Foxy
- Some libraries
```bash
pip install -U numpy python-dateutil kafka-python pyrealsense2
sudo apt install default-jre
```

## DeepStream and Kafka server configuration
- To set up a Kafka server for use with DeepStream, you can refer to the instructions provided in [link1](https://maouriyan.medium.com/how-to-stream-messages-on-deepstream-using-kafka-d7e39de53003) and [link2](https://kafka.apache.org/quickstart). For additional reference, configuration files for DeepStream can be found in the `deepstream` folder.
- To start DeepStream application, run your own build with Kafka broker or `test5-app`:
  ```bash
  deepstream-test5-app -c deepstream_app_config.txt
  ```
## Install
- Clone the package:
  ```bashrc
  git clone https://github.com/maladzenkau/deepstream_ros2
  ```
- Change the parameters in the launch file according to your needs:
  ```bashrc
  gedit ~/deepstream_ros2/src/deepstream_ros2_bridge_launch/launch/launch.py
  ```
# Example
        parameters=[
                    {'kafka_topic': 'deepstream'}, # Name of the created Kafka topic on which inference results are published
                    {'bootstrap_servers': '127.0.0.1:9092'}, # Server adress
                    {'known_objects': ['red', 'yellow', 'green']} # Classes your model is trained to detect
- Save your configuration and navigate to `ros2_ws` colcon root, source and build the package:

  ```bashrc
  cd ~/deepstream_ros2/
  colcon build
  ```
## Run
  - Source `install` and launch the package:
    ```bashrc
    source install/setup.bash
    ros2 launch deepstream_ros2_bridge_launch launch.py 
    ```
Object detection results will be published on topic `/objectDetection`.

# Message Example
```bashrc
---
time_stamp_kafka: '2024-03-25T10:54:50.968Z'
detected_object: '50'
top_left_x: 571
top_left_y: 216
bottom_right_x: 614
bottom_right_y: 261
---
```
## 2. Latency estimation
To estimate the latency, your model should be capable of detecting a synchronization image. It is recommended to use a simple image with a typical background from your dataset. Ensure that the resolution of your synchronization image matches the resolution of the source topic used for object detection. Example of such an image is shown below.
![sync](https://github.com/45kmh/deepstream_ros2/assets/151655734/92b3b257-5ab1-4934-a189-04769182c9f8)

- Open launch file:
  ```bashrc
  gedit ~/deepstream_ros2/src/deepstream_ros2_bridge_launch/launch/launch.py
  ```
- Uncomment latency estimation node:
  ```bashrc
  ld.add_action(latency_estimator)
  ```
- Change the node parameters according to the color data topic:
# Example
        parameters=[
                    {'frequency': '5'}, # How often the latency should be estimated (in seconds)
                    {'color_image_topic': 'color/image_raw'}, # topic which serves as a DeepStream source.
                    {'topic_encoding': 'bgr8'}, # encoding of this topic (could be found using 'ros2 topic info')
                    {'img_path': '/home/jetson/deepstream_ros2/sync.jpg'} # path to your synchronization image
                ]
- Save your configuration and navigate to `ros2_ws` colcon root, source and build the package:
  ```bashrc
  colcon build --packages-select deepstream_ros2_bridge_launch
  ```
The latency will be published on topic `/latency`.

## 3. Color and depth data synchronization
The synchronization node enables the delay of a topic based on a specified latency. It's important to note that for topics with low frame rates, this functionality may not provide significant benefits. This synchronization mechanism ensures that detected boxes from the color frame correspond accurately to objects in the depth frame. Additionally, it implies that the depth sensor produces depth frames aligned with the color sensor frames.
- Open launch file:
  ```bashrc
  gedit ~/deepstream_ros2/src/deepstream_ros2_bridge_launch/launch/launch.py
  ```
- Uncomment delay node:
  ```bashrc
  ld.add_action(delay_topic)
  ```
- Change the delay_topic node parameters according to the depth data topic:
# Example
       parameters=[
                   {'topic_to_be_delayed': '/aligned_depth_to_color/image_raw'},
                   {'fps': 30} # FPS of the topic (should be set the same for both depth and color data topics)
                   ]
- Save your configuration and navigate to `ros2_ws` colcon root, source and build the package:
  ```bashrc
  colcon build --packages-select deepstream_ros2_bridge_launch
  ```
- To check if the bounding boxes of the detected object correspond to the object on the depth frame, uncomment validation node:
  ```bashrc
  ld.add_action(validation)
  ```
To make the visual validation, open rviz and show topics `valColorImage` and `valDepthImage`. Example is shown below.

![Screenshot from 2024-02-19 15-20-57](https://github.com/45kmh/deepstream_ros2/assets/151655734/1e090932-df0a-41fc-b4ce-e6d5cb2b56d2)

## 4. Object coordinates with Realsense Camera d435i
It's possible to extract real-world 3D coordinates from the Realsense Camera d435i pixel coordinates and then convert them to the map frame. This code should be applicable to other Realsense cameras as well.
- To project a pixel into the real-world camera frame, uncomment the relevant section in the code:
  ```bashrc
  ld.add_action(realsense_camera)
  ```
And change the parameters according to the explanations provided is the launch file. Coordinates will be available as a Pose message on the choosen topic.
- To project a pixel into the real-world map frame, uncomment transformation node:
    ```bashrc
  ld.add_action(tf_camera_to_map)
  ```
