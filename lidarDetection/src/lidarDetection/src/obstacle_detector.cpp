#include "lidar_detection/obstacle_detector.hpp"

#include <tf2/utils.h>

#include <boost/make_shared.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_detection/msg/obstacle_detection.hpp"
#include "lidar_detection/msg/obstacle_detection_array.hpp"

namespace lidar_detection
{
double ROTATE_X;
double ROTATE_Y;
double ROTATE_Z;
double ROTATE_ROLL;
double ROTATE_PITCH;
double ROTATE_YAW;

bool USE_PCA_BOX;
bool USE_TRACKING;

std::string GROUND_SEGMENT_TYPE;
double GROUND_THRESH;

double VOXEL_GRID_SIZE;
Eigen::Vector4f ROI_MAX_POINT(0, 0, 0, 1);
Eigen::Vector4f ROI_MIN_POINT(0, 0, 0, 1);

double CLUSTER_THRESH;
int CLUSTER_MAX_SIZE;
int CLUSTER_MIN_SIZE;

double DISPLACEMENT_THRESH;
double IOU_THRESH;

bool ENABLE_STATISTICAL_FILTER;
int STATISTICAL_NB_NEIGHBORS;
double STATISTICAL_STD_RATIO;

bool ENABLE_HEIGHT_RANGE_FILTER;
double GROUND_DISTANCE_THRESH;
double MIN_HEIGHT_AND_LONGEST_EDGE_RATIO;
double BOX_MAX_LENGTH_THRESHOLD;

bool ENABLE_WALL_SEGMENTATION;
double WALL_DISTANCE_THRESH;
double GROUND_NORMAL_ANGLE_THRESH_RAD;
double WALL_NORMAL_ANGLE_THRESH_RAD;

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();
  ~ObstacleDetectorNode() override = default;

private:
  Eigen::Affine3f install_transform;
  std::shared_ptr<ObstacleDetector<pcl::PointXYZ>> obstacle_detector;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  Eigen::Vector4f MIN_POINT, MAX_POINT;
  std::vector<Box> prev_boxes_, curr_boxes_;
  size_t obstacle_id_{0};
  size_t next_obstacle_id_{0};
  std::unordered_map<int, std::vector<geometry_msgs::msg::Point32>> corner_history_;
  float size_smooth_factor_ = 0.3f;
  float corner_smooth_alpha_ = 0.3f;
  double veh_x_{0.0}, veh_y_{0.0}, veh_z_{0.0}, veh_course_{0.0};

  std::string bbox_target_frame_;
  std::string bbox_source_frame_;
  std::unordered_map<int, Eigen::Vector3f> bbox_size_history_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_topic_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_bboxes_;
  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr pub_objects_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_after_voxel_ROI;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_after_statistical_removal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_after_segmentation_first_pointer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_after_segmentation_second_pointer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_clustered_original_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_representative_corners_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_walls_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_segmentation_wall_pointer;

  void lidarPointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_points);

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  void publishClouds(
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> && segmented_clouds,
    const std_msgs::msg::Header & header);

  lidar_detection::msg::ObstacleDetection boxToDetection3D(
    const Box & box, const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster, const std_msgs::msg::Header & header);

  visualization_msgs::msg::MarkerArray transformMarker(
    const Box & box, const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose_transformed);

  std::vector<geometry_msgs::msg::Point32> calculateBoxVertices(
    const Eigen::Vector3f & position, const Eigen::Vector3f & dimension, const Eigen::Quaternionf & quaternion);

  void publishDetectedObjects(
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> && cloud_clusters,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> && convex_clusters, const std_msgs::msg::Header & header);

  pcl::PointCloud<pcl::PointXYZ>::Ptr applyStatisticalOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud);

  bool applyHeightAndRangeFilter(const Box & box);
  void publishCornerMarkers(const std::vector<Box> & boxes, const std_msgs::msg::Header & header);

  std::vector<geometry_msgs::msg::Point32> selectRepresentativeCorners(
    const std::vector<geometry_msgs::msg::Point32> & all_points, const Eigen::Vector3f & position,
    const Eigen::Quaternionf & quaternion);
};

ObstacleDetectorNode::ObstacleDetectorNode() : rclcpp::Node("obstacle_detector")
{
  // Declare parameters with defaults
  auto lidar_points_topic = this->declare_parameter<std::string>("lidar_points_topic", "/livox/lidar");
  auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom");
  auto cloud_ground_topic = this->declare_parameter<std::string>("cloud_ground_topic", "/cloud_ground");
  auto cloud_clusters_topic = this->declare_parameter<std::string>("cloud_clusters_topic", "/cloud_clusters");
  auto marker_bboxes_topic = this->declare_parameter<std::string>("marker_bboxes_topic", "/marker_bboxes");
  auto objects_topic = this->declare_parameter<std::string>("objects_topic", "/detected_objects");
  bbox_target_frame_ = this->declare_parameter<std::string>("bbox_target_frame", "lidar_body");

  // MIN / MAX points for self bbox
  double min_x = this->declare_parameter<double>("min_x", -0.2);
  double min_y = this->declare_parameter<double>("min_y", -0.2);
  double min_z = this->declare_parameter<double>("min_z", -0.2);
  double max_x = this->declare_parameter<double>("max_x", 0.1);
  double max_y = this->declare_parameter<double>("max_y", 0.2);
  double max_z = this->declare_parameter<double>("max_z", 0.1);

  MIN_POINT = Eigen::Vector4f(min_x, min_y, min_z, 1.0f);
  MAX_POINT = Eigen::Vector4f(max_x, max_y, max_z, 1.0f);

  RCLCPP_INFO(this->get_logger(), "Obstacle Detector Node Started");

  // Initialize detector
  obstacle_id_ = 0;
  obstacle_detector = std::make_shared<ObstacleDetector<pcl::PointXYZ>>();

  // Subscriptions
  sub_lidar_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_points_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { this->lidarPointsCallback(msg); });

  sub_odom_topic_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->odomCallback(msg); });

  // Publishers
  pub_cloud_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_ground_topic, rclcpp::QoS(1));
  pub_cloud_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_clusters_topic, rclcpp::QoS(1));
  pub_marker_bboxes_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_bboxes_topic, rclcpp::QoS(1));
  pub_objects_ = this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>(objects_topic, rclcpp::QoS(1));
  pub_cloud_clustered_original_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_clustered_original", rclcpp::QoS(1));
  pub_cloud_after_voxel_ROI =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_after_voxel_ROI", rclcpp::QoS(1));
  pub_cloud_after_statistical_removal =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_after_statistical_removal", rclcpp::QoS(1));
  pub_cloud_after_segmentation_first_pointer =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_segmentation_first", rclcpp::QoS(1));
  pub_cloud_after_segmentation_second_pointer =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_segmentation_second", rclcpp::QoS(1));
  // 在ObstacleDetectorNode构造函数中的Publishers部分添加
  pub_representative_corners_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/representative_corners", rclcpp::QoS(1));
  pub_cloud_walls_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_walls", rclcpp::QoS(1));
  pub_cloud_segmentation_wall_pointer =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_segmentation_wall", rclcpp::QoS(1));

  // Declare dynamic parameters - 默认值与YAML文件保持一致
  this->declare_parameter<double>("rotate_x", 0.0);
  this->declare_parameter<double>("rotate_y", 0.0);
  this->declare_parameter<double>("rotate_z", 0.0);
  this->declare_parameter<double>("rotate_roll", 0.0);
  this->declare_parameter<double>("rotate_pitch", 0.0);
  this->declare_parameter<double>("rotate_yaw", 0.0);

  this->declare_parameter<bool>("use_pca_box", true);
  this->declare_parameter<bool>("use_tracking", true);
  this->declare_parameter<std::string>("ground_segment", "RANSAC");
  this->declare_parameter<double>("voxel_grid_size", 0.2);
  this->declare_parameter<double>("roi_max_x", 5.0);
  this->declare_parameter<double>("roi_max_y", 5.0);
  this->declare_parameter<double>("roi_max_z", 2.0);
  this->declare_parameter<double>("roi_min_x", -4.0);
  this->declare_parameter<double>("roi_min_y", -4.0);
  this->declare_parameter<double>("roi_min_z", -0.5);
  this->declare_parameter<double>("ground_threshold", 0.3);
  this->declare_parameter<double>("cluster_threshold", 0.6);
  this->declare_parameter<int>("cluster_max_size", 500);
  this->declare_parameter<int>("cluster_min_size", 10);
  this->declare_parameter<double>("displacement_threshold", 1.0);
  this->declare_parameter<double>("iou_threshold", 1.0);
  this->declare_parameter<bool>("enable_statistical_filter", true);
  this->declare_parameter<int>("statistical_nb_neighbors", 20);
  this->declare_parameter<double>("statistical_std_ratio", 1.0);
  this->declare_parameter<double>("max_dimension_ratio", 5.0);
  this->declare_parameter<bool>("enable_height_range_filter", false);
  this->declare_parameter<double>("height_limit", 2.0);
  this->declare_parameter<double>("range_length", 5.0);
  this->declare_parameter<double>("range_width", 5.0);
  this->declare_parameter<double>("ground_distance_threshold", 1.8);
  this->declare_parameter<double>("min_height_and_longest_edge_ratio", 0.1);
  this->declare_parameter<double>("box_max_length_threshold", 3.0);
  this->declare_parameter<bool>("enable_wall_segmentation", true);
  this->declare_parameter<double>("wall_distance_threshold", 0.2);
  this->declare_parameter<double>("ground_normal_angle_thresh_rad", 0.34906585);
  this->declare_parameter<double>("wall_normal_angle_thresh_rad", 0.34906585);

  // Read initial dynamic parameters into globals
  this->get_parameter("rotate_x", ROTATE_X);
  this->get_parameter("rotate_y", ROTATE_Y);
  this->get_parameter("rotate_z", ROTATE_Z);
  this->get_parameter("rotate_roll", ROTATE_ROLL);
  this->get_parameter("rotate_pitch", ROTATE_PITCH);
  this->get_parameter("rotate_yaw", ROTATE_YAW);

  this->get_parameter("use_pca_box", USE_PCA_BOX);
  this->get_parameter("use_tracking", USE_TRACKING);
  this->get_parameter("ground_segment", GROUND_SEGMENT_TYPE);
  this->get_parameter("voxel_grid_size", VOXEL_GRID_SIZE);

  double roi_max_x, roi_max_y, roi_max_z, roi_min_x, roi_min_y, roi_min_z;
  this->get_parameter("roi_max_x", roi_max_x);
  this->get_parameter("roi_max_y", roi_max_y);
  this->get_parameter("roi_max_z", roi_max_z);
  this->get_parameter("roi_min_x", roi_min_x);
  this->get_parameter("roi_min_y", roi_min_y);
  this->get_parameter("roi_min_z", roi_min_z);
  ROI_MAX_POINT = Eigen::Vector4f(roi_max_x, roi_max_y, roi_max_z, 1.0f);
  ROI_MIN_POINT = Eigen::Vector4f(roi_min_x, roi_min_y, roi_min_z, 1.0f);

  this->get_parameter("ground_threshold", GROUND_THRESH);
  this->get_parameter("cluster_threshold", CLUSTER_THRESH);
  this->get_parameter("cluster_max_size", CLUSTER_MAX_SIZE);
  this->get_parameter("cluster_min_size", CLUSTER_MIN_SIZE);
  this->get_parameter("displacement_threshold", DISPLACEMENT_THRESH);
  this->get_parameter("iou_threshold", IOU_THRESH);

  this->get_parameter("enable_statistical_filter", ENABLE_STATISTICAL_FILTER);
  this->get_parameter("statistical_nb_neighbors", STATISTICAL_NB_NEIGHBORS);
  this->get_parameter("statistical_std_ratio", STATISTICAL_STD_RATIO);

  this->get_parameter("enable_height_range_filter", ENABLE_HEIGHT_RANGE_FILTER);
  this->get_parameter("ground_distance_threshold", GROUND_DISTANCE_THRESH);
  this->get_parameter("min_height_and_longest_edge_ratio", MIN_HEIGHT_AND_LONGEST_EDGE_RATIO);
  this->get_parameter("box_max_length_threshold", BOX_MAX_LENGTH_THRESHOLD);

  this->get_parameter("enable_wall_segmentation", ENABLE_WALL_SEGMENTATION);
  this->get_parameter("wall_distance_threshold", WALL_DISTANCE_THRESH);
  this->get_parameter("ground_normal_angle_thresh_rad", GROUND_NORMAL_ANGLE_THRESH_RAD);
  this->get_parameter("wall_normal_angle_thresh_rad", WALL_NORMAL_ANGLE_THRESH_RAD);

  /**
   * @brief 动态参数更新回调函数
   */
  // Register parameter change callback (replacement for dynamic_reconfigure)
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) -> rcl_interfaces::msg::SetParametersResult {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";
      for (const auto & param : params) {
        RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed to new value.", param.get_name().c_str());
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          RCLCPP_INFO(this->get_logger(), "+++++++++++++++ Value: %f +++++++++++", param.as_double());
        }
      }
      // Re-read parameters
      this->get_parameter("rotate_x", ROTATE_X);
      this->get_parameter("rotate_y", ROTATE_Y);
      this->get_parameter("rotate_z", ROTATE_Z);
      this->get_parameter("rotate_roll", ROTATE_ROLL);
      this->get_parameter("rotate_pitch", ROTATE_PITCH);
      this->get_parameter("rotate_yaw", ROTATE_YAW);

      this->get_parameter("use_pca_box", USE_PCA_BOX);
      this->get_parameter("use_tracking", USE_TRACKING);
      this->get_parameter("ground_segment", GROUND_SEGMENT_TYPE);
      this->get_parameter("voxel_grid_size", VOXEL_GRID_SIZE);

      double rmx, rmy, rmz, rnx, rny, rnz;
      this->get_parameter("roi_max_x", rmx);
      this->get_parameter("roi_max_y", rmy);
      this->get_parameter("roi_max_z", rmz);
      this->get_parameter("roi_min_x", rnx);
      this->get_parameter("roi_min_y", rny);
      this->get_parameter("roi_min_z", rnz);
      ROI_MAX_POINT = Eigen::Vector4f(rmx, rmy, rmz, 1.0f);
      ROI_MIN_POINT = Eigen::Vector4f(rnx, rny, rnz, 1.0f);

      this->get_parameter("ground_threshold", GROUND_THRESH);
      this->get_parameter("cluster_threshold", CLUSTER_THRESH);
      this->get_parameter("cluster_max_size", CLUSTER_MAX_SIZE);
      this->get_parameter("cluster_min_size", CLUSTER_MIN_SIZE);
      this->get_parameter("displacement_threshold", DISPLACEMENT_THRESH);
      this->get_parameter("iou_threshold", IOU_THRESH);

      this->get_parameter("enable_statistical_filter", ENABLE_STATISTICAL_FILTER);
      this->get_parameter("statistical_nb_neighbors", STATISTICAL_NB_NEIGHBORS);
      this->get_parameter("statistical_std_ratio", STATISTICAL_STD_RATIO);

      this->get_parameter("enable_height_range_filter", ENABLE_HEIGHT_RANGE_FILTER);
      this->get_parameter("ground_distance_threshold", GROUND_DISTANCE_THRESH);
      this->get_parameter("min_height_and_longest_edge_ratio", MIN_HEIGHT_AND_LONGEST_EDGE_RATIO);
      this->get_parameter("box_max_length_threshold", BOX_MAX_LENGTH_THRESHOLD);

      this->get_parameter("enable_wall_segmentation", ENABLE_WALL_SEGMENTATION);
      this->get_parameter("wall_distance_threshold", WALL_DISTANCE_THRESH);
      this->get_parameter("ground_normal_angle_thresh_rad", GROUND_NORMAL_ANGLE_THRESH_RAD);
      this->get_parameter("wall_normal_angle_thresh_rad", WALL_NORMAL_ANGLE_THRESH_RAD);

      return result;
    });
}

void ObstacleDetectorNode::lidarPointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_points)
{
  auto logger = this->get_logger();
  RCLCPP_INFO(
    logger, "Input cloud: frame=%s, width=%u, height=%u, fields=%zu", lidar_points->header.frame_id.c_str(),
    static_cast<unsigned int>(lidar_points->width), static_cast<unsigned int>(lidar_points->height),
    lidar_points->fields.size());

  const auto start_time = std::chrono::steady_clock::now();
  const auto pointcloud_header = lidar_points->header;
  bbox_source_frame_ = lidar_points->header.frame_id;

  // Convert ROS2 PointCloud2 to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_points, *raw_cloud);
  RCLCPP_INFO(logger, "Raw points: %zu", raw_cloud->size());

  if (raw_cloud->empty()) {
    RCLCPP_WARN(logger, "Raw cloud is empty, skipping processing");
    return;
  }

  // Apply installation transform
  install_transform = Eigen::Affine3f::Identity();
  install_transform.rotate(Eigen::AngleAxisf(ROTATE_ROLL, Eigen::Vector3f::UnitX()));
  install_transform.rotate(Eigen::AngleAxisf(ROTATE_PITCH, Eigen::Vector3f::UnitY()));
  install_transform.rotate(Eigen::AngleAxisf(ROTATE_YAW, Eigen::Vector3f::UnitZ()));
  install_transform.translate(Eigen::Vector3f(ROTATE_X, ROTATE_Y, ROTATE_Z));

  pcl::transformPointCloud(*raw_cloud, *raw_cloud, install_transform);

  // Statistical outlier filter
  auto statistical_filtered_cloud = applyStatisticalOutlierFilter(raw_cloud);
  RCLCPP_INFO(
    logger, "After statistical filter: %zu points (neighbors=%d, std_ratio=%.2f)", statistical_filtered_cloud->size(),
    STATISTICAL_NB_NEIGHBORS, STATISTICAL_STD_RATIO);
  sensor_msgs::msg::PointCloud2 cloud_statistical_msg;
  pcl::toROSMsg(*statistical_filtered_cloud, cloud_statistical_msg);
  cloud_statistical_msg.header = pointcloud_header;
  pub_cloud_after_statistical_removal->publish(cloud_statistical_msg);

  // Downsampleing, ROI
  auto filtered_cloud = obstacle_detector->filterCloud(
    statistical_filtered_cloud, VOXEL_GRID_SIZE, ROI_MIN_POINT, ROI_MAX_POINT, MIN_POINT, MAX_POINT);
  RCLCPP_INFO(logger, "After voxel+ROI filter: %zu points", filtered_cloud->size());
  sensor_msgs::msg::PointCloud2 cloud_after_msg;
  pcl::toROSMsg(*filtered_cloud, cloud_after_msg);
  cloud_after_msg.header = pointcloud_header;
  pub_cloud_after_voxel_ROI->publish(cloud_after_msg);

  // Prepare segmented cloud pointers
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds_ptr;
  segmented_clouds_ptr.first = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();   // obstacle cloud
  segmented_clouds_ptr.second = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();  // ground cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud_ptr;

  if (GROUND_SEGMENT_TYPE == "RANSAC") {
    if (ENABLE_WALL_SEGMENTATION) {
      RCLCPP_INFO(logger, "Using RANSAC with wall segmentation");
      auto [obstacle, ground, wall] = obstacle_detector->segmentPlaneWithWalls(
        filtered_cloud, 30, static_cast<float>(GROUND_THRESH), static_cast<float>(WALL_DISTANCE_THRESH),
        static_cast<float>(GROUND_NORMAL_ANGLE_THRESH_RAD), static_cast<float>(WALL_NORMAL_ANGLE_THRESH_RAD));

      segmented_clouds_ptr.first = obstacle;
      segmented_clouds_ptr.second = ground;
      wall_cloud_ptr = wall;

      // 发布墙体点云,调试使用
      sensor_msgs::msg::PointCloud2 cloud_segmented_wall_msg;
      pcl::toROSMsg(*wall_cloud_ptr, cloud_segmented_wall_msg);
      cloud_segmented_wall_msg.header = pointcloud_header;
      pub_cloud_segmentation_wall_pointer->publish(cloud_segmented_wall_msg);

    } else {
      RCLCPP_INFO(logger, "Using RANSAC for ground segmentation only");
      auto segmented_clouds = obstacle_detector->segmentPlane(filtered_cloud, 30, GROUND_THRESH);
      segmented_clouds_ptr.first = segmented_clouds.first;
      segmented_clouds_ptr.second = segmented_clouds.second;
    }
  }

  // 发布分割后的第一个点云（障碍物云）,调试使用 等于 /cloud_clusters
  sensor_msgs::msg::PointCloud2 cloud_segmented_first_msg;
  pcl::toROSMsg(*segmented_clouds_ptr.first, cloud_segmented_first_msg);
  cloud_segmented_first_msg.header = pointcloud_header;
  pub_cloud_after_segmentation_first_pointer->publish(cloud_segmented_first_msg);
  // 发布分割后的第二个点云（地面云）,调试使用 等于 /cloud_ground
  sensor_msgs::msg::PointCloud2 cloud_segmented_second_msg;
  pcl::toROSMsg(*segmented_clouds_ptr.second, cloud_segmented_second_msg);
  cloud_segmented_second_msg.header = pointcloud_header;
  pub_cloud_after_segmentation_second_pointer->publish(cloud_segmented_second_msg);

  auto cloud_clusters =
    obstacle_detector->clustering(segmented_clouds_ptr.first, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_clusters_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & cluster : cloud_clusters) {
    *merged_clusters_cloud += *cluster;
  }

  // 发布合并后的聚类点云
  sensor_msgs::msg::PointCloud2 clusters_msg;
  pcl::toROSMsg(*merged_clusters_cloud, clusters_msg);
  clusters_msg.header = pointcloud_header;
  pub_cloud_clustered_original_->publish(clusters_msg);
  RCLCPP_INFO(
    this->get_logger(), "Published clusters cloud: %zu points, %zu clusters", merged_clusters_cloud->size(),
    cloud_clusters.size());

  ClusteringDebug(cloud_clusters, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE, this->get_logger());

  auto convex_clusters = obstacle_detector->computeConvexHulls(cloud_clusters);
  ConvexHullDebug(convex_clusters, this->get_logger());

  publishClouds(std::move(segmented_clouds_ptr), pointcloud_header);
  publishDetectedObjects(std::move(cloud_clusters), std::move(convex_clusters), pointcloud_header);

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  FrequencyDebug(start_time, end_time, logger);
}

void ObstacleDetectorNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  veh_x_ = odom->pose.pose.position.x;
  veh_y_ = odom->pose.pose.position.y;
  veh_z_ = odom->pose.pose.position.z;
  veh_course_ = tf2::getYaw(odom->pose.pose.orientation);
}

/**
 * @brief 发布分割后的点云数据（地面和障碍物）
 * 
 * 该函数接收分割后的点云对（障碍物云和地面云）和消息头，将PCL点云转换为ROS消息格式，
 * 并通过对应的发布器发布出去。同时会记录发布的点云信息日志。
 * 
 * @param segmented_clouds 右值引用的点云对，包含障碍物云(第一个元素)和地面云(第二个元素)
 * @param header 点云数据的消息头，包含时间戳和坐标系信息
 */
void ObstacleDetectorNode::publishClouds(
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> && segmented_clouds,
  const std_msgs::msg::Header & header)
{
  auto ground_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*(segmented_clouds.second), *ground_cloud);
  ground_cloud->header = header;

  auto obstacle_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*(segmented_clouds.first), *obstacle_cloud);
  obstacle_cloud->header = header;

  if (segmented_clouds.first) {
    auto obstacle_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*(segmented_clouds.first), *obstacle_cloud);
    obstacle_cloud->header = header;
    pub_cloud_clusters_->publish(std::move(obstacle_cloud));
    RCLCPP_INFO(
      this->get_logger(), "Published obstacles cloud: %zu pts, frame=%s", segmented_clouds.first->size(),
      header.frame_id.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Obstacle cloud pointer is null, not publishing.");
  }

  if (segmented_clouds.second) {
    auto ground_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*(segmented_clouds.second), *ground_cloud);
    ground_cloud->header = header;
    pub_cloud_ground_->publish(std::move(ground_cloud));
    RCLCPP_INFO(
      this->get_logger(), "Published ground cloud: %zu pts, frame=%s", segmented_clouds.second->size(),
      header.frame_id.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Ground cloud pointer is null, not publishing.");
  }

  RCLCPP_INFO(
    this->get_logger(), "Publish clouds: ground=%u pts, obstacles=%u pts, frame=%s",
    ground_cloud->width * ground_cloud->height, obstacle_cloud->width * obstacle_cloud->height,
    header.frame_id.c_str());
}
/**
 * @brief 发布检测到的障碍物信息，包括边界框标记和检测对象数组
 * 
 * 该函数接收点云簇和对应的凸包簇，为每个障碍物构建边界框，
 * 可选地进行障碍物跟踪，并将结果发布为ROS标记和自定义检测消息。
 * 
 * @param cloud_clusters 右值引用的点云簇向量，每个元素代表一个障碍物的点云
 * @param convex_clusters 右值引用的凸包簇向量，每个元素代表对应障碍物点云的凸包
 * @param header 点云数据的消息头，包含时间戳和坐标系信息
 */
void ObstacleDetectorNode::publishDetectedObjects(
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> && cloud_clusters,
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> && convex_clusters, const std_msgs::msg::Header & header)
{
  RCLCPP_INFO(this->get_logger(), "Building bounding boxes for %zu clusters", cloud_clusters.size());

  std::vector<Box> temp_boxes;
  std::vector<size_t> cluster_indices;

  for (size_t i = 0; i < cloud_clusters.size(); ++i) {
    auto & cluster = cloud_clusters[i];
    auto & convex_cluster = convex_clusters[i];

    int temp_id = -static_cast<int>(i + 1);
    Box box = USE_PCA_BOX ? obstacle_detector->pcaBoundingBoxSmoothed(cluster, obstacle_id_)
                          : obstacle_detector->axisAlignedBoundingBox(cluster, obstacle_id_);
    obstacle_detector->cleanupBoxHistory();
    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "Created Box ID %zu: position=(%.2f,%.2f,%.2f), dimension=(%.2f,%.2f,%.2f), cluster_points=%zu", obstacle_id_,
    //   box.position[0], box.position[1], box.position[2], box.dimension[0], box.dimension[1], box.dimension[2],
    //   cluster->size());

    std::vector<geometry_msgs::msg::Point32> all_candidate_points =
      calculateBoxVertices(box.position, box.dimension, box.quaternion);

    for (const auto & point : convex_cluster->points) {
      geometry_msgs::msg::Point32 convex_point;
      convex_point.x = point.x;
      convex_point.y = point.y;
      convex_point.z = box.position[2] - box.dimension[2] / 2;
      all_candidate_points.push_back(convex_point);
    }

    auto representative_corners = selectRepresentativeCorners(all_candidate_points, box.position, box.quaternion);

    if (!representative_corners.empty()) {
      representative_corners.push_back(representative_corners.front());
    }
    box = Box(temp_id, box.position, box.dimension, box.quaternion, representative_corners);

    if (applyHeightAndRangeFilter(box)) {
      continue;
    }
    temp_boxes.emplace_back(box);
    cluster_indices.push_back(i);
    // curr_boxes_.emplace_back(box);
  }

  if (USE_TRACKING) {
    obstacle_detector->obstacleTracking(prev_boxes_, temp_boxes, DISPLACEMENT_THRESH, IOU_THRESH);
  }
  for (auto & box : temp_boxes) {
    if (box.id < 0) {
      box.id = static_cast<int>(next_obstacle_id_++);
      if (next_obstacle_id_ > 100000) next_obstacle_id_ = 0;  // 防止溢出
    }
  }

  auto bbox_header = header;
  bbox_header.frame_id = bbox_target_frame_;

  visualization_msgs::msg::MarkerArray bboxes_marker_array;
  lidar_detection::msg::ObstacleDetectionArray detected_objects;
  detected_objects.header = bbox_header;

  curr_boxes_.clear();

  for (size_t i = 0; i < temp_boxes.size(); ++i) {
    if (i >= cloud_clusters.size()) {
      RCLCPP_WARN(
        this->get_logger(), "Box index %zu exceeds cloud_clusters size %zu, skipping", i, cloud_clusters.size());
      continue;
    }

    auto & box = temp_boxes[i];
    size_t orig_idx = cluster_indices[i];

    float size_smooth_factor_ = 0.3f;  // smaller value means more smoothing
    auto it = bbox_size_history_.find(box.id);
    if (it != bbox_size_history_.end()) {
      const Eigen::Vector3f prev = it->second;
      box.dimension = size_smooth_factor_ * box.dimension + (1.0f - size_smooth_factor_) * prev;
      box.dimension = box.dimension.cwiseMax(Eigen::Vector3f::Constant(1e-3f));  // avoid zero dimension
    }
    bbox_size_history_[box.id] = box.dimension;
    curr_boxes_.emplace_back(box);

    auto detection = boxToDetection3D(box, cloud_clusters[orig_idx], bbox_header);
    auto marker_array = transformMarker(box, bbox_header, detection.detection.bbox.center);
    bboxes_marker_array.markers.insert(
      bboxes_marker_array.markers.end(), marker_array.markers.begin(), marker_array.markers.end());
    detected_objects.detections.emplace_back(detection);
  }

  pub_marker_bboxes_->publish(bboxes_marker_array);
  pub_objects_->publish(detected_objects);
  publishCornerMarkers(curr_boxes_, bbox_header);

  RCLCPP_INFO(this->get_logger(), "Boxes built: %zu, target frame=%s", curr_boxes_.size(), bbox_target_frame_.c_str());

  RCLCPP_INFO(
    this->get_logger(), "Published: %zu markers, %zu detections, target_frame=%s", bboxes_marker_array.markers.size(),
    detected_objects.detections.size(), bbox_target_frame_.c_str());

  prev_boxes_.swap(curr_boxes_);
  curr_boxes_.clear();
}

/**
 * @brief 从所有点中选择代表性的角点
 * 
 * 该函数实现了一个基于方向投影的角点选择算法。通过将点云转换到局部坐标系，
 * 并在8个主要方向上寻找最大投影点，从而选择出能够代表物体轮廓的关键角点。
 * 
 * @param all_points 输入的所有点集
 * @param position 物体的中心位置
 * @param quaternion 物体的方向四元数
 * @return std::vector<geometry_msgs::msg::Point32> 选中的代表性角点
 */
std::vector<geometry_msgs::msg::Point32> ObstacleDetectorNode::selectRepresentativeCorners(
  const std::vector<geometry_msgs::msg::Point32> & all_points, const Eigen::Vector3f & position,
  const Eigen::Quaternionf & quaternion)
{
  if (all_points.size() <= 8) {
    return all_points;
  }

  Eigen::Quaternionf inv_quat = quaternion.inverse();
  std::vector<Eigen::Vector3f> local_points;
  local_points.reserve(all_points.size());

  for (const auto & p : all_points) {
    Eigen::Vector3f global_pt(p.x, p.y, p.z);
    Eigen::Vector3f local_pt = inv_quat * (global_pt - position);
    local_points.push_back(local_pt);
  }

  std::vector<size_t> corner_indices(8, 0);
  std::vector<float> max_projections(8, -std::numeric_limits<float>::max());

  const std::vector<Eigen::Vector3f> directions = {
    Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(-1, -1, 1), Eigen::Vector3f(-1, 1, -1), Eigen::Vector3f(-1, 1, 1),
    Eigen::Vector3f(1, -1, -1),  Eigen::Vector3f(1, -1, 1),  Eigen::Vector3f(1, 1, -1),  Eigen::Vector3f(1, 1, 1)};

  for (size_t i = 0; i < local_points.size(); ++i) {
    for (size_t d = 0; d < directions.size(); ++d) {
      float projection = local_points[i].dot(directions[d]);
      if (projection > max_projections[d]) {
        max_projections[d] = projection;
        corner_indices[d] = i;
      }
    }
  }

  std::set<size_t> unique_indices(corner_indices.begin(), corner_indices.end());
  std::vector<geometry_msgs::msg::Point32> result;
  result.reserve(unique_indices.size());
  for (size_t idx : unique_indices) {
    result.push_back(all_points[idx]);
  }

  // RCLCPP_INFO(
  //   this->get_logger(), "Selected %zu representative corners from %zu total points", result.size(), all_points.size());
  return result;
}

/**
 * @brief 应用统计离群点滤波器去除噪声点
 * 
 * 该函数使用PCL库中的统计离群点去除算法，基于点与其邻域点的距离统计特性
 * 来识别和去除离群点。算法假设点云中的点是局部平滑的，离群点通常与邻域
 * 点的距离分布有明显差异。
 * 
 * @param input_cloud 输入的点云数据
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 滤波后的点云（如果滤波失败则返回原始点云）
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr ObstacleDetectorNode::applyStatisticalOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud)
{
  if (!ENABLE_STATISTICAL_FILTER || input_cloud->size() < static_cast<size_t>(STATISTICAL_NB_NEIGHBORS)) {
    return input_cloud;
  }
  try {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(STATISTICAL_NB_NEIGHBORS);
    sor.setStddevMulThresh(STATISTICAL_STD_RATIO);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_cloud);

    RCLCPP_DEBUG(
      this->get_logger(), "Statistical outlier filter: %zu -> %zu points", input_cloud->size(), filtered_cloud->size());
    return filtered_cloud;

  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Statistical outlier filter failed: %s", e.what());
    return input_cloud;
  }
}

bool ObstacleDetectorNode::applyHeightAndRangeFilter(const Box & box)
{
  if (!ENABLE_HEIGHT_RANGE_FILTER) {
    RCLCPP_DEBUG(this->get_logger(), "Height and range filter disabled");
    return false;  // 不进行过滤
  }

  const float h = box.dimension[2];
  const float longest = std::max({box.dimension[0], box.dimension[1], box.dimension[2]});

  // 高度太矮（相对最长边），滤掉
  if (h < static_cast<float>(MIN_HEIGHT_AND_LONGEST_EDGE_RATIO) * longest) {
    RCLCPP_DEBUG(
      this->get_logger(), "Filter box(id=%d): height %.2f < %.2f * longest_edge %.2f", box.id, h,
      MIN_HEIGHT_AND_LONGEST_EDGE_RATIO, longest);
    return true;
  }

  // 离地太远（底面 z 与地面距离大于阈值），滤掉
  const float bottom_z = box.position(2) - h / 2.0f;
  if (bottom_z > static_cast<float>(GROUND_DISTANCE_THRESH)) {
    RCLCPP_INFO(
      this->get_logger(), "Filter box(id=%d): bottom_z %.2f > ground_distance_thresh %.2f", box.id, bottom_z,
      GROUND_DISTANCE_THRESH);
    return true;
  }

  if (longest > static_cast<float>(BOX_MAX_LENGTH_THRESHOLD)) {
    RCLCPP_INFO(
      this->get_logger(), "Filter box(id=%d): longest edge %.2f > box_max_length_threshold %.2f", box.id, longest,
      BOX_MAX_LENGTH_THRESHOLD);
    return true;
  }
  return false;
}
/**
 * @brief 创建并返回一个包含多种障碍物可视化标记的标记数组
 * 
 * 该函数接收障碍物边界框信息、消息头和转换后的位姿，
 * 调用三个辅助函数分别创建足迹/立方体标记、速度箭头标记和文本标记，
 * 并将这些标记组合成一个标记数组返回。
 * 
 * @param box 障碍物边界框对象的引用，包含位置、尺寸、旋转等信息
 * @param header 消息头，包含时间戳和坐标系信息，用于所有创建的标记
 * @param pose_transformed 转换后的位姿信息，用于标记在目标坐标系中的定位
 * @return visualization_msgs::msg::MarkerArray 包含多种障碍物可视化标记的数组
 */
visualization_msgs::msg::MarkerArray ObstacleDetectorNode::transformMarker(
  const Box & box, const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose_transformed)
{
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.emplace_back(makeFootprintOrCubeMarker(box, header, pose_transformed));
  arr.markers.emplace_back(makeVelocityArrowMarker(box, header, pose_transformed));
  arr.markers.emplace_back(makeTextMarker(box, header, pose_transformed));
  return arr;
}
lidar_detection::msg::ObstacleDetection ObstacleDetectorNode::boxToDetection3D(
  const Box & box, const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster, const std_msgs::msg::Header & header)
{
  lidar_detection::msg::ObstacleDetection obstacle_detection;
  obstacle_detection.detection.header = header;

  obstacle_detection.id = std::to_string(box.id);
  obstacle_detection.detection.bbox.center.position.x = box.position(0);
  obstacle_detection.detection.bbox.center.position.y = box.position(1);
  obstacle_detection.detection.bbox.center.position.z = box.position(2);
  obstacle_detection.detection.bbox.center.orientation.w = box.quaternion.w();
  obstacle_detection.detection.bbox.center.orientation.x = box.quaternion.x();
  obstacle_detection.detection.bbox.center.orientation.y = box.quaternion.y();
  obstacle_detection.detection.bbox.center.orientation.z = box.quaternion.z();

  obstacle_detection.detection.bbox.size.x = box.dimension(0);
  obstacle_detection.detection.bbox.size.y = box.dimension(1);
  obstacle_detection.detection.bbox.size.z = box.dimension(2);

  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.id = std::to_string(box.id);
  hypothesis.pose.pose.position = obstacle_detection.detection.bbox.center.position;
  hypothesis.pose.pose.orientation = obstacle_detection.detection.bbox.center.orientation;
  obstacle_detection.detection.results.push_back(hypothesis);

  pcl::toROSMsg(*cluster, obstacle_detection.detection.source_cloud);
  obstacle_detection.detection.source_cloud.header = header;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = box.velocity(0);
  twist.linear.y = box.velocity(1);
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  obstacle_detection.twist = twist;

  return obstacle_detection;
}

/**
 * @brief 计算边界框在全局坐标系中的8个顶点坐标
 * 
 * 该函数接收边界框的位置、尺寸和旋转四元数，首先在局部坐标系下定义边界框的8个顶点，
 * 然后通过四元数旋转和位置平移将这些顶点转换到全局坐标系中，最后返回转换后的顶点列表。
 * 
 * @param position 边界框中心点在全局坐标系中的位置 (x, y, z)
 * @param dimension 边界框的尺寸 (长度, 宽度, 高度)，分别对应x, y, z轴方向的尺寸
 * @param quaternion 边界框的旋转四元数，用于表示边界框在全局坐标系中的朝向
 * @return std::vector<geometry_msgs::msg::Point32> 包含8个顶点的向量，每个顶点在全局坐标系中的位置
 */
std::vector<geometry_msgs::msg::Point32> ObstacleDetectorNode::calculateBoxVertices(
  const Eigen::Vector3f & position, const Eigen::Vector3f & dimension, const Eigen::Quaternionf & quaternion)
{
  float length = dimension[0];
  float width = dimension[1];
  float height = dimension[2];

  Eigen::Vector3f local_corners[8] = {
    Eigen::Vector3f(-length / 2, -width / 2, -height / 2), Eigen::Vector3f(-length / 2, width / 2, -height / 2),
    Eigen::Vector3f(length / 2, width / 2, -height / 2),   Eigen::Vector3f(length / 2, -width / 2, -height / 2),
    Eigen::Vector3f(-length / 2, -width / 2, height / 2),  Eigen::Vector3f(-length / 2, width / 2, height / 2),
    Eigen::Vector3f(length / 2, width / 2, height / 2),    Eigen::Vector3f(length / 2, -width / 2, height / 2)};

  std::vector<geometry_msgs::msg::Point32> global_corners;
  global_corners.reserve(8);

  for (int i = 0; i < 8; ++i) {
    Eigen::Vector3f global_corner = quaternion * local_corners[i] + position;
    geometry_msgs::msg::Point32 corner;
    corner.x = global_corner.x();
    corner.y = global_corner.y();
    corner.z = global_corner.z();
    global_corners.push_back(corner);
  }

  return global_corners;
}

void ObstacleDetectorNode::publishCornerMarkers(const std::vector<Box> & boxes, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray corners_vis;
  int mid = 0;
  for (const auto & box : boxes) {
    // 1) 代表性角点 POINTS（来自 Box::convex_hull）
    visualization_msgs::msg::Marker pts;
    pts.header = header;
    pts.ns = "rep_corners";
    pts.id = mid++;
    pts.type = visualization_msgs::msg::Marker::POINTS;
    pts.action = visualization_msgs::msg::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.05;
    pts.scale.y = 0.05;
    pts.color.g = 1.0;
    pts.color.a = 1.0;
    for (const auto & p : box.convex_hull) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      pts.points.push_back(gp);
    }
    corners_vis.markers.push_back(pts);

    // 2) 8 个顶点连线 LINE_STRIP（来自 calculateBoxVertices ）
    auto full_corners = calculateBoxVertices(box.position, box.dimension, box.quaternion);
    visualization_msgs::msg::Marker lines;
    lines.header = header;
    lines.ns = "full_vertices";
    lines.id = mid++;
    lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.03;
    lines.color.b = 1.0;
    lines.color.a = 1.0;
    for (const auto & p : full_corners) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      lines.points.push_back(gp);
    }
    if (!lines.points.empty()) {
      lines.points.push_back(lines.points.front());  // 闭合
    }
    corners_vis.markers.push_back(lines);
  }
  pub_representative_corners_->publish(corners_vis);
}

}  // namespace lidar_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_detection::ObstacleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}