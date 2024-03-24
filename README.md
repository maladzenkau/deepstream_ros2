# deepstream_ros2 (Pre-reliese phase)
This project allows bridging DeepStream inference results (rectangular boxes) with ROS2.

If you wish to utilize a ROS2 topic of type `sensor_msgs::msg::Image` for object detection, you can employ the [image2rtsp](https://github.com/45kmh/image2rtsp) package to buffer the topic into an RTSP stream.

The development is being carried out on Ubuntu 20.04 with ROS2 Foxy at Jetson ORIN NX 16GB running JetPack 5.1.2 (DeepStream 6.3).
## Dependencies
- ROS2 Foxy
- Java library
```bash
sudo apt install default-jre
```

## DeepStream and Kafka server configuration
- To set up a Kafka server for use with DeepStream, you can refer to the instructions provided in [link1](https://maouriyan.medium.com/how-to-stream-messages-on-deepstream-using-kafka-d7e39de53003) and [link2](https://kafka.apache.org/quickstart). For additional reference, configuration files for DeepStream can be found in the 'deepstream' folder.
- To start DeepStream application, run:
  ```bash
  deepstream-test5-app -c deepstream_app_config.txt
  ```
## Install
- Clone the package:
  ```bashrc
  git clone https://github.com/45kmh/deepstream_ros2
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
