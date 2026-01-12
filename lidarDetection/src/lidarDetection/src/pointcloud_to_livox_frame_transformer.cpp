#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

using std::placeholders::_1;

class CloudToLivox : public rclcpp::Node
{
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_body_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_body_pub_;

  std::string input_cloud_registered_topic_;
  std::string output_cloud_livox_topic_;
  std::string input_cloud_registered_body_topic_;
  std::string output_cloud_livox_body_topic_;

public:
  CloudToLivox() : Node("cloud_to_livox_frame"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    input_cloud_registered_topic_ =
      this->declare_parameter<std::string>("input_cloud_registered_topic", "/cloud_registered");
    output_cloud_livox_topic_ =
      this->declare_parameter<std::string>("output_cloud_livox_topic", "/cloud_registered_livox_frame");
    input_cloud_registered_body_topic_ =
      this->declare_parameter<std::string>("input_cloud_registered_body_topic", "/cloud_registered_body");
    output_cloud_livox_body_topic_ =
      this->declare_parameter<std::string>("output_cloud_livox_body_topic", "/cloud_registered_body_livox_frame");

    cloud_registered_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_registered_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudToLivox::CloudRegisteredCallback, this, std::placeholders::_1));
    cloud_registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_livox_topic_, 50);

    cloud_registered_body_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_registered_body_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudToLivox::CloudRegisteredBodyCallback, this, std::placeholders::_1));
    cloud_registered_body_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_livox_body_topic_, 50);
  }

  void CloudRegisteredCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform("livox_frame", msg->header.frame_id, msg->header.stamp);
      sensor_msgs::msg::PointCloud2 out;
      tf2::doTransform(*msg, out, tf);
      out.header.frame_id = "livox_frame";
      cloud_registered_pub_->publish(out);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cloud_registered TF: %s", ex.what());
    }
  }
  void CloudRegisteredBodyCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform("livox_frame", msg->header.frame_id, msg->header.stamp);
      sensor_msgs::msg::PointCloud2 out;
      tf2::doTransform(*msg, out, tf);
      out.header.frame_id = "livox_frame";
      cloud_registered_body_pub_->publish(out);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cloud_registered_body TF: %s", ex.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToLivox>());
  rclcpp::shutdown();
  return 0;
}