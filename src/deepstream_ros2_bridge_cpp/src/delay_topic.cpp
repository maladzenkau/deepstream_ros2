#include <chrono>
#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  { 
    this->declare_parameter("topic_to_be_delayed", "/aligned_depth_to_color/image_raw");
    topic = this->get_parameter("topic_to_be_delayed").as_string();
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    topic, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    subscription_latency_ = this->create_subscription<std_msgs::msg::Int16>(
    "latency", 10, std::bind(&MinimalSubscriber::latency_callback, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("delayed_topic" + topic, 10);
    RCLCPP_INFO(this->get_logger(), "Synchronization node is up");
  }

  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (latency == 0.0f) {
    RCLCPP_DEBUG(this->get_logger(), "No latency received");
    }

    else if (buffer_counter == buffer_size){
        sensor_msgs::msg::Image::SharedPtr new_msg=buffer.front();
        buffer.push(msg);
        buffer.pop();
        RCLCPP_DEBUG(this->get_logger(), "Publishing with '%d' images in the buffer", buffer_size);
        publisher_->publish(*new_msg.get());
    }
    else if (buffer_counter > buffer_size) {
      sensor_msgs::msg::Image::SharedPtr new_msg=buffer.front();
      buffer.pop();
      buffer_counter--;
      RCLCPP_DEBUG(this->get_logger(), "Descreasing to '%d' images in the buffer", buffer_size);
      publisher_->publish(*new_msg.get());
    }

    else if (buffer_counter < buffer_size) {
      buffer.push(msg);
      buffer_counter++;
      RCLCPP_DEBUG(this->get_logger(), "Increasing to '%d' images in the buffer", buffer_size);
    }
  }

  void latency_callback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    latency = static_cast<float>(msg->data) / 1000.0f;
    if (latency > 0.5){
      RCLCPP_WARN(this->get_logger(), "Latency is too high!");
    }
    else if (latency > 1.5){
      RCLCPP_ERROR(this->get_logger(), "Jetson is overloaded! Immediately stop the object detection task!");
    }
    buffer_size = static_cast<int>(latency * 30);
    RCLCPP_DEBUG(this->get_logger(), "Latency callback for: '%d' images in the buffer", buffer_size);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_latency_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> buffer;
  float latency = 0.0;
  std::string topic;
  int buffer_size = 0;
  int buffer_counter = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
