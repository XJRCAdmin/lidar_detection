// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>

// class StaticMapFilterNode : public rclcpp::Node
// {
// public:
//   StaticMapFilterNode() : Node("static_map_filter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
//   {
//     this->declare_parameter<std::string>("static_map_path", "");
//     this->declare_parameter<double>("distance_threshold", 0.3);
//     this->declare_parameter<std::string>("target_frame", "map");
//     this->declare_parameter<std::string>("source_frame", "lidar_body");
//     this->declare_parameter<std::string>("input_cloud_topic", "/cloud_registered_body");
//     this->declare_parameter<std::string>("output_cloud_topic", "/map_filtered_cloud");
//     this->declare_parameter<double>("downsample_resolution", 0.2);  // 静态地图下采样
//     this->declare_parameter<bool>("enable_filter", true);
//     this->declare_parameter<bool>("publish_downsampled_map", false);
//     this->declare_parameter<std::string>("downsampled_map_topic", "/static_map_downsampled");
//     this->declare_parameter<bool>("publish_original_map", false);  // 可选
//     this->declare_parameter<std::string>("original_map_topic", "/static_map_original");

//     // 获取参数（使用更安全的方式）
//     this->get_parameter_or("static_map_path", static_map_path_, std::string(""));
//     this->get_parameter_or("distance_threshold", distance_threshold_, 0.3);
//     this->get_parameter_or("target_frame", target_frame_, std::string("map"));
//     this->get_parameter_or("source_frame", source_frame_, std::string("lidar_body"));
//     this->get_parameter_or("input_cloud_topic", input_cloud_topic_, std::string("/cloud_registered_body"));
//     this->get_parameter_or("output_cloud_topic", output_cloud_topic_, std::string("/map_filtered_cloud"));
//     this->get_parameter_or("downsample_resolution", downsample_resolution_, 0.2);
//     this->get_parameter_or("enable_filter", enable_filter_, true);
//     this->get_parameter_or("publish_downsampled_map", publish_downsampled_map_, false);
//     this->get_parameter_or("downsampled_map_topic", downsampled_map_topic_, std::string("/static_map_downsampled"));
//     this->get_parameter_or("publish_original_map", publish_original_map_, false);
//     this->get_parameter_or("original_map_topic", original_map_topic_, std::string("/static_map_original"));

//     cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);

//     if (publish_downsampled_map_) {
//       downsampled_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(downsampled_map_topic_, 10);
//     }

//     if (publish_original_map_) {
//       original_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(original_map_topic_, 10);
//     }
//     if (!loadStaticMap()) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to load static map, filter disabled");
//       enable_filter_ = false;
//     }
//     cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//       input_cloud_topic_, 10, std::bind(&StaticMapFilterNode::cloudCallback, this, std::placeholders::_1));

//     RCLCPP_INFO(this->get_logger(), "Static Map Filter Node started");
//     RCLCPP_INFO(this->get_logger(), "  Static map: %s", static_map_path_.c_str());
//     RCLCPP_INFO(this->get_logger(), "  Distance threshold: %.2f m", distance_threshold_);
//     RCLCPP_INFO(this->get_logger(), "  Target frame: %s", target_frame_.c_str());
//     RCLCPP_INFO(this->get_logger(), "  Filter enabled: %s", enable_filter_ ? "true" : "false");
//   }

// private:
//   bool loadStaticMap()
//   {
//     if (static_map_path_.empty()) {
//       RCLCPP_WARN(this->get_logger(), "Static map path is empty");
//       return false;
//     }

//     // 加载 PCD 文件
//     pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     if (pcl::io::loadPCDFile<pcl::PointXYZ>(static_map_path_, *original_cloud) == -1) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", static_map_path_.c_str());
//       return false;
//     }

//     RCLCPP_INFO(this->get_logger(), "Loaded static map with %zu points", original_cloud->size());

//     if (publish_original_map_) {
//       sensor_msgs::msg::PointCloud2 original_msg;
//       pcl::toROSMsg(*original_cloud, original_msg);
//       original_msg.header.frame_id = target_frame_;
//       original_msg.header.stamp = this->now();
//       original_map_pub_->publish(original_msg);
//       RCLCPP_INFO(this->get_logger(), "Original map published");
//     }

//     // 下采样以提高性能
//     static_map_cloud_ = original_cloud;  // 默认使用原始地图
//     if (downsample_resolution_ > 0.0) {
//       pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
//       pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
//       voxel_filter.setInputCloud(original_cloud);
//       voxel_filter.setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
//       voxel_filter.filter(*downsampled);
//       static_map_cloud_ = downsampled;
//       RCLCPP_INFO(this->get_logger(), "Downsampled to %zu points", static_map_cloud_->size());
//     }

//     // 如果需要发布下采样地图
//     if (publish_downsampled_map_ && downsampled_map_pub_) {
//       sensor_msgs::msg::PointCloud2 downsampled_msg;
//       pcl::toROSMsg(*static_map_cloud_, downsampled_msg);
//       downsampled_msg.header.frame_id = target_frame_;
//       downsampled_msg.header.stamp = this->now();
//       downsampled_map_pub_->publish(downsampled_msg);
//       RCLCPP_INFO(this->get_logger(), "Downsampled map published");
//     }

//     // 构建 KD-Tree
//     kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
//     kdtree_->setInputCloud(static_map_cloud_);
//     RCLCPP_INFO(this->get_logger(), "KD-Tree built successfully");

//     return true;
//   }

//   void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//   {
//     // 如果过滤器未启用，直接转发
//     if (!enable_filter_) {
//       cloud_pub_->publish(*msg);
//       return;
//     }

//     auto start_time = this->now();

//     // 转换为 PCL 点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::fromROSMsg(*msg, *input_cloud);

//     // 获取 TF 变换：从 source_frame 到 target_frame
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     try {
//       transform_stamped = tf_buffer_.lookupTransform(
//         target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
//     } catch (tf2::TransformException & ex) {
//       RCLCPP_WARN_THROTTLE(
//         this->get_logger(), *this->get_clock(), 1000, "Could not transform from %s to %s: %s",
//         msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
//       cloud_pub_->publish(*msg);
//       return;
//     }

//     // 将点云变换到 map 坐标系
//     pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     sensor_msgs::msg::PointCloud2 transformed_msg;
//     tf2::doTransform(*msg, transformed_msg, transform_stamped);
//     pcl::fromROSMsg(transformed_msg, *transformed_cloud);

//     // 过滤点云：保留距离静态地图较远的点（动态物体）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_map(new pcl::PointCloud<pcl::PointXYZ>());
//     std::vector<int> pointIdxNKNSearch(1);
//     std::vector<float> pointNKNSquaredDistance(1);

//     int removed_count = 0;
//     for (const auto & point : transformed_cloud->points) {
//       // 检查点是否有效
//       if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
//         continue;
//       }

//       // KD-Tree 最近邻搜索
//       if (kdtree_->nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
//         float distance = std::sqrt(pointNKNSquaredDistance[0]);

//         // 距离大于阈值 → 动态物体，保留
//         if (distance > distance_threshold_) {
//           filtered_cloud_map->points.push_back(point);
//         } else {
//           removed_count++;
//         }
//       }
//     }

//     filtered_cloud_map->width = filtered_cloud_map->points.size();
//     filtered_cloud_map->height = 1;
//     filtered_cloud_map->is_dense = false;

//     // 将过滤后的点云转回 source_frame
//     sensor_msgs::msg::PointCloud2 filtered_msg_map;
//     pcl::toROSMsg(*filtered_cloud_map, filtered_msg_map);
//     filtered_msg_map.header.stamp = msg->header.stamp;
//     filtered_msg_map.header.frame_id = target_frame_;

//     // 获取逆变换
//     geometry_msgs::msg::TransformStamped inverse_transform;
//     try {
//       inverse_transform = tf_buffer_.lookupTransform(
//         msg->header.frame_id, target_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
//     } catch (tf2::TransformException & ex) {
//       RCLCPP_WARN_THROTTLE(
//         this->get_logger(), *this->get_clock(), 1000, "Could not get inverse transform: %s", ex.what());
//       cloud_pub_->publish(*msg);
//       return;
//     }

//     sensor_msgs::msg::PointCloud2 output_msg;
//     tf2::doTransform(filtered_msg_map, output_msg, inverse_transform);
//     output_msg.header.stamp = msg->header.stamp;
//     output_msg.header.frame_id = msg->header.frame_id;

//     // 发布过滤后的点云
//     cloud_pub_->publish(output_msg);

//     auto end_time = this->now();
//     auto duration = (end_time - start_time).seconds();

//     RCLCPP_DEBUG(
//       this->get_logger(), "Filtered %d/%zu points (%.1f%%) in %.3f ms", removed_count, input_cloud->size(),
//       100.0 * removed_count / input_cloud->size(), duration * 1000.0);
//   }

//   // TF2
//   tf2_ros::Buffer tf_buffer_;
//   tf2_ros::TransformListener tf_listener_;

//   // ROS2 通信
//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_map_pub_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_map_pub_;

//   // 参数
//   std::string static_map_path_;
//   double distance_threshold_;
//   std::string target_frame_;
//   std::string source_frame_;
//   std::string input_cloud_topic_;
//   std::string output_cloud_topic_;
//   double downsample_resolution_;
//   bool enable_filter_;
//   bool publish_downsampled_map_;
//   std::string downsampled_map_topic_;
//   bool publish_original_map_;
//   std::string original_map_topic_;

//   // 静态地图和 KD-Tree
//   pcl::PointCloud<pcl::PointXYZ>::Ptr static_map_cloud_;
//   pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<StaticMapFilterNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }