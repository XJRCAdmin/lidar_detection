#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PcdMapPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map_;
  sensor_msgs::msg::PointCloud2 pc2_msg_;
  std::string pcd_file_path_;
  std::string output_topic_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  double repeat_sec_ = 0.0;

public:
  PcdMapPublisher() : Node("map_launcher_node")
  {
    pcd_file_path_ = this->declare_parameter<std::string>("pcd_file_path", "map.pcd");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/global_map");
    repeat_sec_ = this->declare_parameter<double>("repeat_sec", 0.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

    pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    repeat_sec_ = this->get_parameter("repeat_sec").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    if (pcd_file_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "pcd_file_path parameter is empty. Set pcd_file_path to a .pcd file.");
      throw std::runtime_error("pcd_file_path empty");
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    pub_global_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

    if (!loadPcdToMsg(pcd_file_path_, pc2_msg_)) {
      throw std::runtime_error("Failed to load PCD file: " + pcd_file_path_);
    }

    pc2_msg_.header.frame_id = frame_id_;
    pc2_msg_.header.stamp = this->now();

    pub_global_map_->publish(pc2_msg_);
    RCLCPP_INFO(
      this->get_logger(), "Published PCD to topic '%s' (frame: %s)", output_topic_.c_str(), frame_id_.c_str());

    // if repeat_sec_ > 0.0, set up a timer to republish
    if (repeat_sec_ > 0.0) {
      timer_ = this->create_wall_timer(std::chrono::duration<double>(repeat_sec_), [this]() {
        pc2_msg_.header.stamp = this->now();
        pub_global_map_->publish(pc2_msg_);
        RCLCPP_DEBUG(this->get_logger(), "Re-published PCD");
      });
    }
  }
  ~PcdMapPublisher() = default;

  /**
   * @brief 从PCD文件加载点云数据到ROS2消息
   * 
   * 该函数从指定路径加载PCD格式的点云文件，将其转换为sensor_msgs::msg::PointCloud2格式
   * 如果加载失败，则记录错误日志并返回false
   * 
   * @param path PCD文件的路径
   * @param msg 输出的ROS2点云消息
   * @return 成功返回true，失败返回false
   */
  bool loadPcdToMsg(const std::string & path, sensor_msgs::msg::PointCloud2 & msg)
  {
    pcl::PCLPointCloud2 cloud2;
    if (pcl::io::loadPCDFile(path, cloud2) < 0) {
      RCLCPP_ERROR(this->get_logger(), "pcl::io::loadPCDFile failed for %s", path.c_str());
      return false;
    }
    pcl_conversions::fromPCL(cloud2, msg);
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcdMapPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
