#ifndef LIDAR_DETECTION_OBSTACLE_DETECTOR_HPP_
#define LIDAR_DETECTION_OBSTACLE_DETECTOR_HPP_

// ROS2
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <pcl_ros/transforms.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Dense"
#include "box.hpp"
#include "lidar_detection/Marker_visual.hpp"
#include "lidar_detection/ukf.hpp"
#include "lidar_detection/utils.hpp"

namespace lidar_detection
{
template <typename PointT>
class ObstacleDetector
{
public:
  ObstacleDetector();
  virtual ~ObstacleDetector();

  // ****************** Detection ***********************
  typename pcl::PointCloud<PointT>::Ptr filterCloud(
    const typename pcl::PointCloud<PointT>::ConstPtr & cloud, const float filter_res, const Eigen::Vector4f & min_pt,
    const Eigen::Vector4f & max_pt, const Eigen::Vector4f & MIN_POINT, const Eigen::Vector4f & MAX_POINT);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentPlane(
    const typename pcl::PointCloud<PointT>::ConstPtr & cloud, const int max_iterations, const float distance_thresh);

  std::tuple<
    typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
  segmentPlaneWithWalls(
    const typename pcl::PointCloud<PointT>::ConstPtr & cloud, int max_iterations, float ground_distance_thresh,
    float wall_distance_thresh, float ground_normal_angle_thresh_rad, float wall_normal_angle_thresh_rad);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clustering(
    const typename pcl::PointCloud<PointT>::ConstPtr & cloud, const float cluster_tolerance, const int min_size,
    const int max_size);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> computeConvexHulls(
    const std::vector<typename pcl::PointCloud<PointT>::Ptr> & cloud_clusters);

  Box axisAlignedBoundingBox(const typename pcl::PointCloud<PointT>::ConstPtr & cluster, const int id);

  Box pcaBoundingBox(typename pcl::PointCloud<PointT>::Ptr & cluster, const int id);

  Box pcaBoundingBoxSmoothed(typename pcl::PointCloud<PointT>::Ptr & cluster, const int id);
  void cleanupBoxHistory();

  // ****************** Tracking ***********************
  void obstacleTracking(
    const std::vector<Box> & prev_boxes, std::vector<Box> & curr_boxes, const float displacement_thresh,
    const float iou_thresh);

private:
  // ****************** Detection ***********************
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateClouds(
    const pcl::PointIndices::ConstPtr & inliers, const typename pcl::PointCloud<PointT>::ConstPtr & cloud);

  // ****************** Tracking ***********************
  std::unordered_map<int, UKF> ukf_states;
  std::unordered_map<int, int> match_counts;
  std::unordered_map<int, Eigen::Vector2f> velocity_history_;
  const float static_speed_gate_ = 0.15f;
  const float velocity_alpha_ = 0.35f;

  bool compareBoxes(const Box & a, const Box & b, const float displacement_thresh, const float iou_thresh);
  // ****************** BBox smoothing ***********************
  struct BoxHistory
  {
    Eigen::Vector3f position;
    Eigen::Vector3f dimension;
    Eigen::Quaternionf quaternion;
    int frame_count;
    std::chrono::steady_clock::time_point last_update;
  };
  std::vector<BoxHistory> box_history_pool_;

  const float position_match_thresh_ = 0.5f;     // 位置匹配阈值（米）
  const float dimension_smooth_alpha_ = 0.3f;    // 尺寸平滑系数
  const float orientation_smooth_alpha_ = 0.4f;  // 朝向平滑系数
  const float position_smooth_alpha_ = 0.5f;     // 位置平滑系数
  const int max_history_age_ms_ = 500;           // 历史记录最大存活时间（毫秒）

  // Link nearby bounding boxes between the previous and previous frame
  std::vector<std::vector<int>> associateBoxes(
    const std::vector<Box> & prev_boxes, const std::vector<Box> & curr_boxes, const float displacement_thresh,
    const float iou_thresh);

  // Connection Matrix
  std::vector<std::vector<int>> connectionMatrix(
    const std::vector<std::vector<int>> & connection_pairs, std::vector<int> & left, std::vector<int> & right);

  // Helper function for Hungarian Algorithm
  bool hungarianFind(
    const int i, const std::vector<std::vector<int>> & connection_matrix, std::vector<bool> & right_connected,
    std::vector<int> & right_pair);

  // Customized Hungarian Algorithm
  std::vector<int> hungarian(const std::vector<std::vector<int>> & connection_matrix);

  // Helper function for searching the box index in boxes given an id
  int searchBoxIndex(const std::vector<Box> & Boxes, const int id);

  // Helper function for checking if a point is inside a bounding box
  bool isPointInBoundingBox(const Eigen::Vector3f & point, const Box & box);

  int findMatchingHistory(const Eigen::Vector3f & position, const Eigen::Vector3f & dimension);

  Eigen::Quaternionf alignQuaternion(const Eigen::Quaternionf & prev, const Eigen::Quaternionf & curr);
};

// constructor:
template <typename PointT>
ObstacleDetector<PointT>::ObstacleDetector()
{
}

// de-constructor:
template <typename PointT>
ObstacleDetector<PointT>::~ObstacleDetector()
{
}
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ObstacleDetector<PointT>::filterCloud(
  const typename pcl::PointCloud<PointT>::ConstPtr & cloud, const float filter_res, const Eigen::Vector4f & min_pt,
  const Eigen::Vector4f & max_pt, const Eigen::Vector4f & MIN_POINT, const Eigen::Vector4f & MAX_POINT)
{
  // logger
  auto logger = rclcpp::get_logger("obstacle_detector");

  // Create the filtering object: downsample the dataset using a leaf size
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filter_res, filter_res, filter_res);
  vg.filter(*cloud_filtered);

  // Cropping the ROI
  typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(false);
  region.setMin(min_pt);
  region.setMax(max_pt);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_roi);

  // Crop out points in a smaller exclusion box (MIN_POINT..MAX_POINT)
  std::vector<int> indices;
  pcl::CropBox<PointT> Box(true);
  Box.setMin(MIN_POINT);
  Box.setMax(MAX_POINT);
  Box.setInputCloud(cloud_roi);
  Box.filter(indices);

  // Convert indices to PointIndices
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto & point_idx : indices) {
    inliers->indices.push_back(point_idx);
  }

  // Extract indices (setNegative = true means remove those indices)
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_roi);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_roi);

  return cloud_roi;
}

template <typename PointT>
std::tuple<
  typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::segmentPlaneWithWalls(
  const typename pcl::PointCloud<PointT>::ConstPtr & cloud, int max_iterations, float ground_distance_thresh,
  float wall_distance_thresh, float ground_normal_angle_thresh_rad, float wall_normal_angle_thresh_rad)
{
  // segment plane 有可能会分割出墙面,也有可能是地面
  // 因为场景中墙面不一定存在,但是一定有地面,因而需要判断分割出的平面究竟是墙面还是地面
  // 对于分割出的地面平面,判断其法向量与Z轴的夹角是否小于ground_normal_angle_thresh_rad,如果小于,则是.说明分割的是地面,则退出处理.返回
  // 否则,说明分割出来的是墙面,则需要进一步分割地面
  // 再次运行segmentPlane在剩余点云中分割地面
  auto logger = rclcpp::get_logger("obstacle_detector");

  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr wall_cloud(new pcl::PointCloud<PointT>());

  typename pcl::PointCloud<PointT>::Ptr current_remaining(new pcl::PointCloud<PointT>(*cloud));
  /** 分割地面 ***/
  // Find inliers for the cloud.
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(ground_distance_thresh);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(ground_normal_angle_thresh_rad);
  // Segment the largest planar component from the input cloud
  seg.setInputCloud(current_remaining);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty()) {
    RCLCPP_WARN(logger, "Could not estimate a planar model for the given dataset.");
  }
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(current_remaining);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*ground_cloud);
  typename pcl::PointCloud<PointT>::Ptr next_remaining(new pcl::PointCloud<PointT>());
  extract.setNegative(true);
  extract.filter(*next_remaining);
  current_remaining = next_remaining;
  RCLCPP_INFO(
    logger, "After ground segmentation: obstacle points=%zu, ground points=%zu", current_remaining->size(),
    ground_cloud->size());
  if (current_remaining->size() > 300) {
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setEpsAngle(wall_normal_angle_thresh_rad);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(wall_distance_thresh);

    seg.setInputCloud(current_remaining);
    seg.segment(*inliers, *coefficients);
    RCLCPP_INFO(logger, "Wall segmentation found %zu inliers", inliers->indices.size());
    if (inliers->indices.size() > 200) {
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(current_remaining);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*wall_cloud);
      extract.setNegative(true);
      extract.filter(*obstacle_cloud);
    } else {
      obstacle_cloud = current_remaining;
    }
  } else {
    obstacle_cloud = current_remaining;
  }
  // Eigen::Vector4f centroid;
  // Eigen::Matrix3f covariance;
  // pcl::computeMeanAndCovarianceMatrix(*candidate_plane, covariance, centroid);
  // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  // Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);
  // float angle_diff = std::acos(std::abs(normal.dot(Eigen::Vector3f::UnitZ())));
  // RCLCPP_INFO(logger, "Angle between candidate plane normal and Z axis: %f", angle_diff);
  // RCLCPP_INFO(
  //   logger, "Candidate plane size: %d, ground_normal_angle_thresh_rad: %f", candidate_plane->size(),
  //   ground_normal_angle_thresh_rad);

  // if (angle_diff < ground_normal_angle_thresh_rad) {
  //   ground_cloud = candidate_plane;
  //   obstacle_cloud = remaining_cloud;
  // } else if (angle_diff >= wall_normal_angle_thresh_rad && candidate_plane->size() > 150) {
  //   RCLCPP_INFO(
  //     // 说明分割出来的是墙面,需要进一步分割地面
  //     logger, "Wall normal angle threshold: %f, wall distance threshold: %f", wall_normal_angle_thresh_rad,
  //     wall_distance_thresh);
  //   wall_cloud = candidate_plane;
  //   auto second_segmentation = segmentPlane(remaining_cloud, max_iterations, ground_distance_thresh);
  //   obstacle_cloud = second_segmentation.first;
  //   ground_cloud = second_segmentation.second;
  // }

  return {obstacle_cloud, ground_cloud, wall_cloud};
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::separateClouds(
  const pcl::PointIndices::ConstPtr & inliers, const typename pcl::PointCloud<PointT>::ConstPtr & cloud)
{
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());

  // Pushback all the inliers into the ground_cloud
  for (int index : inliers->indices) {
    ground_cloud->points.push_back(cloud->points[index]);
  }

  // Extract the points that are not in the inliers to obstacle_cloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
    obstacle_cloud, ground_cloud);
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::segmentPlane(
  const typename pcl::PointCloud<PointT>::ConstPtr & cloud, const int max_iterations, const float distance_thresh)
{
  auto logger = rclcpp::get_logger("obstacle_detector");

  // Find inliers for the cloud.
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(distance_thresh);

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty()) {
    RCLCPP_WARN(logger, "Could not estimate a planar model for the given dataset.");
  }

  return separateClouds(inliers, cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::clustering(
  const typename pcl::PointCloud<PointT>::ConstPtr & cloud, const float cluster_tolerance, const int min_size,
  const int max_size)
{
  auto logger = rclcpp::get_logger("obstacle_detector");

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Perform euclidean clustering to group detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (auto & getIndices : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    for (auto & index : getIndices.indices) {
      cluster->points.push_back(cloud->points[index]);
    }

    cluster->width = static_cast<uint32_t>(cluster->points.size());
    cluster->height = 1;
    cluster->is_dense = true;

    clusters.push_back(cluster);
  }

  RCLCPP_DEBUG(logger, "clustering found %zu clusters", clusters.size());
  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::computeConvexHulls(
  const std::vector<typename pcl::PointCloud<PointT>::Ptr> & cloud_clusters)
{
  std::vector<typename pcl::PointCloud<PointT>::Ptr> convex_hulls;

  for (const auto & cluster : cloud_clusters) {
    if (cluster->points.size() < 4u)  // unsigned 4
    {
      typename pcl::PointCloud<PointT>::Ptr hull(new pcl::PointCloud<PointT>);
      convex_hulls.push_back(hull);
      continue;
    }
    const double min_eps = 10 * std::numeric_limits<double>::epsilon();
    const double diff_x = cluster->points[1].x - cluster->points[0].x;
    const double diff_y = cluster->points[1].y - cluster->points[0].y;
    size_t idx = 0;
    for (idx = 2; idx < cluster->points.size(); ++idx) {
      const double tdiff_x = cluster->points[idx].x - cluster->points[0].x;
      const double tdiff_y = cluster->points[idx].y - cluster->points[0].y;
      if ((diff_x * tdiff_y - tdiff_x * diff_y) > min_eps) {
        break;
      }
    }
    if (idx >= cluster->points.size()) {
      // 为了避免共线点，微调前两个点
      cluster->points[0].x += min_eps;
      cluster->points[0].y += min_eps;
      cluster->points[1].x -= min_eps;
    }
    // Create a new point cloud to store the convex hull
    typename pcl::PointCloud<PointT>::Ptr hull(new pcl::PointCloud<PointT>);

    // Create a ConvexHull object
    pcl::ConvexHull<PointT> chull;
    chull.setInputCloud(cluster);
    chull.setDimension(2);  // Restrict to 2D (xy-plane)

    // Compute the convex hull and store it in hull
    chull.reconstruct(*hull);

    // Add the convex hull to the result vector
    convex_hulls.push_back(hull);
  }

  return convex_hulls;
}

// -------------------- axisAlignedBoundingBox --------------------
template <typename PointT>
Box ObstacleDetector<PointT>::axisAlignedBoundingBox(
  const typename pcl::PointCloud<PointT>::ConstPtr & cluster, const int id)
{
  // Find bounding box for one of the clusters
  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);

  const Eigen::Vector3f position(
    (max_pt.x + min_pt.x) / 2.0f, (max_pt.y + min_pt.y) / 2.0f, (max_pt.z + min_pt.z) / 2.0f);
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), (max_pt.z - min_pt.z));

  return Box(id, position, dimension);
}

// -------------------- pcaBoundingBox --------------------
template <typename PointT>
Box ObstacleDetector<PointT>::pcaBoundingBox(typename pcl::PointCloud<PointT>::Ptr & cluster, const int id)
{
  // Compute the bounding box height (to be used later for recreating the box)
  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  const float box_height = max_pt.z - min_pt.z;
  // const float box_z = (max_pt.z + min_pt.z) / 2.0f;

  // Compute the cluster centroid
  Eigen::Vector4f pca_centroid;
  pcl::compute3DCentroid(*cluster, pca_centroid);

  // Squash the cluster to x-y plane with z = centroid z
  for (size_t i = 0; i < cluster->size(); ++i) {
    cluster->points[i].z = pca_centroid(2);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster);
  pca.project(*cluster, *pca_projected_cloud);

  const auto eigen_vectors = pca.getEigenVectors();

  // Get the minimum and maximum points of the transformed cloud.
  pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
  const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf quaternion(eigen_vectors);
  const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid.head<3>();
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), box_height);

  return Box(id, position, dimension, quaternion);
}

// ************************* Tracking ***************************
template <typename PointT>
void ObstacleDetector<PointT>::obstacleTracking(
  const std::vector<Box> & prev_boxes, std::vector<Box> & curr_boxes, const float displacement_thresh,
  const float iou_thresh)
{
  auto logger = rclcpp::get_logger("obstacle_detector");
  // RCLCPP_INFO(
  //   logger, "box tracking between %zu previous boxes and %zu current boxes", prev_boxes.size(), curr_boxes.size());

  if (curr_boxes.empty() || prev_boxes.empty()) {
    return;
  } else {
    // vectors containing the id of boxes in left and right sets
    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    // Associate Boxes that are similar in two frames
    auto connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) return;

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);

    RCLCPP_INFO(
      logger, "Connection matrix dimensions: %zu x %zu", connection_matrix.size(),
      (connection_matrix.empty() ? 0 : connection_matrix[0].size()));
    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);
    RCLCPP_INFO(logger, "Hungarian algorithm returned %zu matches.", matches.size());

    // Update the unmatched count for each UKF state
    std::unordered_set<int> matched_ids;
    for (int j = 0; j < static_cast<int>(matches.size()); ++j) {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);

      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j];  // right and matches has the same size
      const auto cur_index = searchBoxIndex(curr_boxes, cur_id);

      if (pre_index > -1 && cur_index > -1) {
        // change the id of the current box to the same as the previous box
        curr_boxes[cur_index].id = prev_boxes[pre_index].id;
        // UKF prediction and update
        if (ukf_states.find(prev_boxes[pre_index].id) == ukf_states.end()) {
          ukf_states[prev_boxes[pre_index].id] = UKF();
          match_counts[prev_boxes[pre_index].id] = 1;
        } else {
          match_counts[prev_boxes[pre_index].id]++;
        }

        MeasurementPackage meas_package;
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        meas_package.timestamp_ = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        meas_package.sensor_type_ = MeasurementPackage::LASER;

        meas_package.raw_measurements_ = VectorXd(2);
        meas_package.raw_measurements_ << curr_boxes[cur_index].position[0], curr_boxes[cur_index].position[1];
        ukf_states[prev_boxes[pre_index].id].ProcessMeasurement(meas_package);

        const int uid = prev_boxes[pre_index].id;
        auto & ukf = ukf_states[uid];
        double p_x = ukf.x_(0);
        double p_y = ukf.x_(1);
        double v_abs = ukf.x_(2);
        double psi = ukf.x_(3);
        double vx = v_abs * std::cos(psi);
        double vy = v_abs * std::sin(psi);

        // Eigen::Vector2f measured_velocity(vx, vy);
        // if (measured_velocity.norm() < static_speed_gate_) {
        //   measured_velocity.setZero();
        //   ukf.x_(2) = 0.0;
        //   ukf.x_(4) = 0.0;
        // }
        // auto history = velocity_history_[uid];
        // Eigen::Vector2f smoothed = velocity_alpha_ * measured_velocity + (1.0f - velocity_alpha_) * history;
        // velocity_history_[uid] = smoothed;

        curr_boxes[cur_index].velocity[0] = vx;
        curr_boxes[cur_index].velocity[1] = vy;

        // double p_x = ukf_states[prev_boxes[pre_index].id].x_(0);
        // double p_y = ukf_states[prev_boxes[pre_index].id].x_(1);
        Eigen::Vector3f ukf_position(p_x, p_y, curr_boxes[cur_index].position[2]);
        if (isPointInBoundingBox(ukf_position, curr_boxes[cur_index])) {
          RCLCPP_INFO(
            logger, "track target id is %d, detect position is (%.3f, %.3f), track position is (%.3f, %.3f)",
            curr_boxes[cur_index].id, curr_boxes[cur_index].position[0], curr_boxes[cur_index].position[1], p_x, p_y);
          // ============================================
          curr_boxes[cur_index].position[0] = p_x;
          curr_boxes[cur_index].position[1] = p_y;
        }
        matched_ids.insert(prev_boxes[pre_index].id);
        RCLCPP_DEBUG_STREAM(
          logger, "Matched Prev ID: " << prev_boxes[pre_index].id << " with Curr ID: " << curr_boxes[cur_index].id);
      }
    }

    // delete unmatched UKF states
    for (auto it = ukf_states.begin(); it != ukf_states.end();) {
      if (matched_ids.find(it->first) == matched_ids.end()) {
        it = ukf_states.erase(it);
      } else {
        ++it;
      }
    }

    for (auto it = match_counts.begin(); it != match_counts.end();) {
      if (matched_ids.find(it->first) == matched_ids.end()) {
        it = match_counts.erase(it);
      } else {
        ++it;
      }
    }
  }
}

// template <typename PointT>
// bool ObstacleDetector<PointT>::compareBoxes(
//   const Box & a, const Box & b, const float displacement_thresh, const float iou_thresh)
// {
//   auto logger = rclcpp::get_logger("obstacle_detector");
//   const float dis = std::sqrt(
//     (a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) +
//     (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) +
//     (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

//   const float a_max_dim = std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
//   const float b_max_dim = std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
//   const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

//   const float x_dim = std::abs(2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]));
//   const float y_dim = std::abs(2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]));
//   const float z_dim = std::abs(2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]));

//   // RCLCPP_WARN(
//   //   logger,
//   //   "Comparing Box A ID: %d with Box B ID: %d | Center Displacement: %.3f (thresh: %.3f) | "
//   //   "X Dim Diff: %.3f (thresh: %.3f) | Y Dim Diff: %.3f (thresh: %.3f) | "
//   //   "Z Dim Diff: %.3f (thresh: %.3f)",
//   //   a.id, b.id, ctr_dis, displacement_thresh, x_dim, iou_thresh, y_dim, iou_thresh, z_dim, iou_thresh);

//   return (
//     ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh &&
//     isPointInBoundingBox(a.position, b));
// }

template <typename PointT>
bool ObstacleDetector<PointT>::compareBoxes(
  const Box & a, const Box & b, const float displacement_thresh, const float iou_thresh)
{
  auto logger = rclcpp::get_logger("obstacle_detector");

  const float dis = std::sqrt(
    (a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) +
    (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) +
    (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

  const float a_max_dim = std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
  const float b_max_dim = std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
  const float max_dim = std::max(a_max_dim, b_max_dim);
  const float ctr_dis = dis / max_dim;

  const float x_dim = std::abs(a.dimension[0] - b.dimension[0]) / std::max(a.dimension[0], b.dimension[0]);
  const float y_dim = std::abs(a.dimension[1] - b.dimension[1]) / std::max(a.dimension[1], b.dimension[1]);
  const float z_dim = std::abs(a.dimension[2] - b.dimension[2]) / std::max(a.dimension[2], b.dimension[2]);

  // RCLCPP_INFO(
  //   logger,
  //   "Comparing Box A ID: %d with Box B ID: %d | Center Displacement: %.3f (thresh: %.3f) | "
  //   "X Dim Diff: %.3f (thresh: %.3f) | Y Dim Diff: %.3f (thresh: %.3f) | "
  //   "Z Dim Diff: %.3f (thresh: %.3f)",
  //   a.id, b.id, ctr_dis, displacement_thresh, x_dim, iou_thresh, y_dim, iou_thresh, z_dim, iou_thresh);

  bool match = (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh);

  return match;
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::associateBoxes(
  const std::vector<Box> & prev_boxes, const std::vector<Box> & curr_boxes, const float displacement_thresh,
  const float iou_thresh)
{
  std::vector<std::vector<int>> connection_pairs;
  auto logger = rclcpp::get_logger("obstacle_detector");
  for (auto & prev_box : prev_boxes) {
    for (auto & curBox : curr_boxes) {
      // Add the indecies of a pair of similiar boxes to the matrix
      if (this->compareBoxes(curBox, prev_box, displacement_thresh, iou_thresh)) {
        connection_pairs.push_back({prev_box.id, curBox.id});
        RCLCPP_WARN(logger, "Connected Prev ID: %d with Curr ID: %d", prev_box.id, curBox.id);
      }
    }
  }
  RCLCPP_WARN(logger, "Total connection pairs found: %zu", connection_pairs.size());
  return connection_pairs;
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::connectionMatrix(
  const std::vector<std::vector<int>> & connection_pairs, std::vector<int> & left, std::vector<int> & right)
{
  // Hash the box ids in the connection_pairs to two vectors(sets), left and right
  for (auto & pair : connection_pairs) {
    bool left_found = false;
    for (auto i : left) {
      if (i == pair[0]) left_found = true;
    }
    if (!left_found) left.push_back(pair[0]);

    bool right_found = false;
    for (auto j : right) {
      if (j == pair[1]) right_found = true;
    }
    if (!right_found) right.push_back(pair[1]);
  }

  std::vector<std::vector<int>> connection_matrix(left.size(), std::vector<int>(right.size(), 0));

  for (auto & pair : connection_pairs) {
    int left_index = -1;
    for (size_t i = 0; i < left.size(); ++i) {
      if (pair[0] == left[i]) left_index = i;
    }

    int right_index = -1;
    for (size_t i = 0; i < right.size(); ++i) {
      if (pair[1] == right[i]) right_index = i;
    }

    if (left_index != -1 && right_index != -1) connection_matrix[left_index][right_index] = 1;
  }

  return connection_matrix;
}

template <typename PointT>
bool ObstacleDetector<PointT>::hungarianFind(
  const int i, const std::vector<std::vector<int>> & connection_matrix, std::vector<bool> & right_connected,
  std::vector<int> & right_pair)
{
  for (size_t j = 0; j < connection_matrix[0].size(); ++j) {
    if (connection_matrix[i][j] == 1 && right_connected[j] == false) {
      right_connected[j] = true;

      if (right_pair[j] == -1 || hungarianFind(right_pair[j], connection_matrix, right_connected, right_pair)) {
        right_pair[j] = i;
        return true;
      }
    }
  }
  return false;
}
template <typename PointT>
std::vector<int> ObstacleDetector<PointT>::hungarian(const std::vector<std::vector<int>> & connection_matrix)
{
  std::vector<bool> right_connected(connection_matrix[0].size(), false);
  std::vector<int> right_pair(connection_matrix[0].size(), -1);
  auto logger = rclcpp::get_logger("obstacle_detector");
  int count = 0;
  for (int i = 0; i < static_cast<int>(connection_matrix.size()); ++i) {
    if (hungarianFind(i, connection_matrix, right_connected, right_pair)) {
      count++;
    }
  }

  RCLCPP_INFO(
    logger, "For: %zu current frame bounding boxes, found: %d matches in previous frame!", right_pair.size(), count);
  return right_pair;
}

template <typename PointT>
int ObstacleDetector<PointT>::searchBoxIndex(const std::vector<Box> & boxes, const int id)
{
  for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
    if (boxes[i].id == id) {
      return i;
    }
  }
  return -1;
}

template <typename PointT>
bool ObstacleDetector<PointT>::isPointInBoundingBox(const Eigen::Vector3f & point, const Box & box)
{
  Eigen::Vector3f local_point = box.quaternion.inverse() * (point - box.position);

  return (
    std::abs(local_point.x()) <= box.dimension.x() / 2 && std::abs(local_point.y()) <= box.dimension.y() / 2 &&
    std::abs(local_point.z()) <= box.dimension.z() / 2);
}

template <typename PointT>
Eigen::Quaternionf ObstacleDetector<PointT>::alignQuaternion(
  const Eigen::Quaternionf & prev, const Eigen::Quaternionf & curr)
{
  // 如果点积为负，翻转当前四元数以确保最短路径插值
  if (prev.dot(curr) < 0.0f) {
    return Eigen::Quaternionf(-curr.w(), -curr.x(), -curr.y(), -curr.z());
  }
  return curr;
}

template <typename PointT>
int ObstacleDetector<PointT>::findMatchingHistory(const Eigen::Vector3f & position, const Eigen::Vector3f & dimension)
{
  int best_idx = -1;
  float best_score = std::numeric_limits<float>::max();

  auto now = std::chrono::steady_clock::now();

  for (size_t i = 0; i < box_history_pool_.size(); ++i) {
    auto & hist = box_history_pool_[i];

    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - hist.last_update).count();
    if (age_ms > max_history_age_ms_) {
      continue;
    }

    // 计算位置距离
    float pos_dist = (position - hist.position).norm();

    // 计算尺寸相似度（用最大尺寸归一化）
    float max_dim = std::max(
      {dimension.x(), dimension.y(), dimension.z(), hist.dimension.x(), hist.dimension.y(), hist.dimension.z()});
    float dim_diff = (dimension - hist.dimension).norm() / max_dim;

    // 综合评分：位置权重更高
    float score = pos_dist + 0.3f * dim_diff;

    if (pos_dist < position_match_thresh_ && score < best_score) {
      best_score = score;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

template <typename PointT>
Box ObstacleDetector<PointT>::pcaBoundingBoxSmoothed(typename pcl::PointCloud<PointT>::Ptr & cluster, const int id)
{
  // 先用原始 PCA 计算当前帧的 box
  Box raw_box = pcaBoundingBox(cluster, id);

  // 查找匹配的历史记录
  int hist_idx = findMatchingHistory(raw_box.position, raw_box.dimension);

  auto now = std::chrono::steady_clock::now();

  if (hist_idx >= 0) {
    // 找到匹配，进行平滑
    auto & hist = box_history_pool_[hist_idx];

    // 平滑位置
    Eigen::Vector3f smoothed_position =
      position_smooth_alpha_ * raw_box.position + (1.0f - position_smooth_alpha_) * hist.position;

    // 平滑尺寸
    Eigen::Vector3f smoothed_dimension =
      dimension_smooth_alpha_ * raw_box.dimension + (1.0f - dimension_smooth_alpha_) * hist.dimension;

    // 平滑朝向（使用 slerp）
    Eigen::Quaternionf aligned_quat = alignQuaternion(hist.quaternion, raw_box.quaternion);
    Eigen::Quaternionf smoothed_quaternion = hist.quaternion.slerp(orientation_smooth_alpha_, aligned_quat);
    smoothed_quaternion.normalize();

    // 更新历史记录
    hist.position = smoothed_position;
    hist.dimension = smoothed_dimension;
    hist.quaternion = smoothed_quaternion;
    hist.frame_count++;
    hist.last_update = now;

    return Box(id, smoothed_position, smoothed_dimension, smoothed_quaternion, raw_box.convex_hull);
  } else {
    // 没有匹配，创建新的历史记录
    BoxHistory new_hist;
    new_hist.position = raw_box.position;
    new_hist.dimension = raw_box.dimension;
    new_hist.quaternion = raw_box.quaternion;
    new_hist.frame_count = 1;
    new_hist.last_update = now;
    box_history_pool_.push_back(new_hist);

    return raw_box;
  }
}

template <typename PointT>
void ObstacleDetector<PointT>::cleanupBoxHistory()
{
  auto now = std::chrono::steady_clock::now();

  box_history_pool_.erase(
    std::remove_if(
      box_history_pool_.begin(), box_history_pool_.end(),
      [this, now](const BoxHistory & hist) {
        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - hist.last_update).count();
        return age_ms > max_history_age_ms_;
      }),
    box_history_pool_.end());
}

}  // namespace lidar_detection

#endif  // LIDAR_DETECTION_OBSTACLE_DETECTOR_HPP_