#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"


class TfListener : public rclcpp::Node{
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::PoseStamped pose_in_;
    geometry_msgs::msg::PoseStamped pose_out_;
    std::string coordinate_topic;
    std::string coordinate_map_topic;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        try {
            pose_in_ = *msg;
            tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in_, pose_out_, "map",
                tf2::Duration(std::chrono::seconds(1)));
            pose_out_.pose.position.z = 0;
            publisher_->publish(pose_out_);
        } catch (const std::exception &ex) {
            RCLCPP_DEBUG(get_logger(), "An error occurred: %s", ex.what());
            return;
        }
    }

  public:
    TfListener(const std::string & name):
    Node(name){
        this->declare_parameter("coordinate_topic", "/objectCoordinateInCameraFrame");
        this->declare_parameter("coordinate_map_topic", "/objectCoordinateInMapFrame");
        coordinate_topic = this->get_parameter("coordinate_topic").as_string();
        coordinate_map_topic = this->get_parameter("coordinate_map_topic").as_string();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>
            (coordinate_topic, 10,
            std::bind(&TfListener::poseCallback, this, std::placeholders::_1));
        publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(coordinate_map_topic, 10);
    }
    ~TfListener(){}
};

int main(int argc, char const *argv[]){
   rclcpp::init(argc, argv);
   auto node = std::make_shared<TfListener>("tf_listener");
   RCLCPP_INFO(node->get_logger(), "Object coordinates in map frame publisher node is up");
   try
   {
     rclcpp::spin(node);
   }
   catch (const std::exception& e)
   {
     RCLCPP_ERROR(node->get_logger(), "Encountered error: %s", e.what());
   }
   catch (...)
   {
     RCLCPP_ERROR(node->get_logger(), "Unknown error occurred");
   }
   rclcpp::shutdown();
   return 0;
}
