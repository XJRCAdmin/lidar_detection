#include <memory>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lidar_detection/Marker_visual.hpp"
#include "lidar_detection/msg/obstacle_detection.hpp"
#include "lidar_detection/msg/obstacle_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class ObstacleToOtherFrameNode : public rclcpp::Node
{
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string obstacle_lidar_topic_;
  std::string obstacle_baselink_topic_;
  std::string dynamic_obstacle_baselink_topic_;
  std::string static_obstacle_baselink_topic_;
  std::string source_frame_;
  std::string target_frame_;
  std::string transition_frame_;
  double static_velocity_thresh_;
  bool AlignObstacleToFrame_;
  double smoothing_alpha_;
  double match_position_thresh_;
  double match_size_ratio_thresh_;
  std::string visualization_mode_;

  lidar_detection::msg::ObstacleDetectionArray static_obstacle_cache_;
  lidar_detection::msg::ObstacleDetectionArray dynamic_obstacle_cache_;

  rclcpp::Subscription<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr obstacle_lidar_sub_;
  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr static_obstacle_to_baselink_pub_;
  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr dynamic_obstacle_to_baselink_pub_;
  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr static_obstacle_to_map_pub_;
  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr dynamic_obstacle_to_map_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_obstacle_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacle_text_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacle_cube_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacle_array_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_obstacle_markers_to_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacle_cube_markers_to_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacle_text_markers_to_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacle_array_markers_to_map_pub_;

  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr
    static_obstacle_pub_;  //lidar body 系的静态障碍物
  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr dynamic_obstacle_pub_;

public:
  ObstacleToOtherFrameNode()
  : Node("obstacle_to_other_frame_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    obstacle_lidar_topic_ = this->declare_parameter<std::string>("obstacle_lidar_topic", "/detected_objects");
    dynamic_obstacle_baselink_topic_ =
      this->declare_parameter<std::string>("dynamic_obstacle_baselink_topic", "/dynamic_obstacle_in_baselink");
    static_obstacle_baselink_topic_ =
      this->declare_parameter<std::string>("static_obstacle_baselink_topic", "/static_obstacle_in_baselink");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "lidar_body");
    transition_frame_ =
      this->declare_parameter<std::string>("transition_frame", "map");  // 借助map系滤去相对于map系静止的物体
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    static_velocity_thresh_ = this->declare_parameter<double>("static_velocity_thresh", 0.1);
    AlignObstacleToFrame_ = this->declare_parameter<bool>("AlignObstacleToFrame", true);
    smoothing_alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.7);
    match_position_thresh_ = this->declare_parameter<double>("match_position_thresh", 0.5);
    match_size_ratio_thresh_ = this->declare_parameter<double>("match_size_ratio_thresh", 0.3);
    visualization_mode_ = this->declare_parameter<std::string>("visualization_mode", "map");

    this->get_parameter("smoothing_alpha", smoothing_alpha_);
    this->get_parameter("match_position_thresh", match_position_thresh_);
    this->get_parameter("match_size_ratio_thresh", match_size_ratio_thresh_);
    this->get_parameter("visualization_mode", visualization_mode_);
    this->get_parameter("AlignObstacleToFrame", AlignObstacleToFrame_);
    this->get_parameter("static_velocity_thresh", static_velocity_thresh_);

    obstacle_lidar_sub_ = this->create_subscription<lidar_detection::msg::ObstacleDetectionArray>(
      obstacle_lidar_topic_, 10, std::bind(&ObstacleToOtherFrameNode::ObstacleCallback, this, std::placeholders::_1));
    dynamic_obstacle_to_baselink_pub_ =
      this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>(dynamic_obstacle_baselink_topic_, 100);
    static_obstacle_to_baselink_pub_ =
      this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>(static_obstacle_baselink_topic_, 100);
    // 供rviz使用的可视化话题
    static_obstacle_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/static_obstacle_markers", 1000);
    dynamic_obstacle_text_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacle_text_markers", 1000);
    dynamic_obstacle_array_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacle_array_markers", 1000);
    dynamic_obstacle_cube_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacle_cube_markers", 1000);

    // map系下的静态障碍物可视化话题
    static_obstacle_markers_to_map_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/static_obstacle_markers_in_map", 1000);
    dynamic_obstacle_cube_markers_to_map_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacle_cube_in_map", 1000);
    dynamic_obstacle_text_markers_to_map_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacle_text_in_map", 1000);
    dynamic_obstacle_array_markers_to_map_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacle_array_in_map", 1000);
    static_obstacle_to_map_pub_ =
      this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>("/static_obstacle_in_map", 1000);
    dynamic_obstacle_to_map_pub_ =
      this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>("/dynamic_obstacle_in_map", 1000);

    static_obstacle_pub_ =
      this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>("/static_obstacle_in_lidar_body", 100);
    dynamic_obstacle_pub_ =
      this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>("/dynamic_obstacle_in_lidar_body", 100);

    RCLCPP_INFO(this->get_logger(), "Obstacle To Other Frame Node Started");
    RCLCPP_INFO(this->get_logger(), "Obstacle lidar topic: %s", obstacle_lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Obstacle baselink topic: %s", obstacle_baselink_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Transform from '%s' to '%s'", source_frame_.c_str(), target_frame_.c_str());
  }

  void ObstacleCallback(const lidar_detection::msg::ObstacleDetectionArray::SharedPtr obstacle_msg)
  {
    lidar_detection::msg::ObstacleDetectionArray custom_obstacle_array_msg;
    custom_obstacle_array_msg.header = obstacle_msg->header;
    custom_obstacle_array_msg.header.frame_id = transition_frame_;
    // transition to map and judge static/dynamic
    TransformObstacle(source_frame_, transition_frame_, *obstacle_msg, custom_obstacle_array_msg);

    lidar_detection::msg::ObstacleDetectionArray static_obstacle_msgs;
    lidar_detection::msg::ObstacleDetectionArray dynamic_obstacle_msgs;
    lidar_detection::msg::ObstacleDetectionArray baselink_static_obstacle_msgs;
    lidar_detection::msg::ObstacleDetectionArray baselink_dynamic_obstacle_msgs;

    visualization_msgs::msg::MarkerArray static_obstacle_markers;
    visualization_msgs::msg::MarkerArray dynamic_obstacle_text_markers;
    visualization_msgs::msg::MarkerArray dynamic_obstacle_array_markers;
    visualization_msgs::msg::MarkerArray dynamic_obstacle_cube_markers;

    static_obstacle_msgs.header = custom_obstacle_array_msg.header;
    dynamic_obstacle_msgs.header = custom_obstacle_array_msg.header;
    for (auto & detection : custom_obstacle_array_msg.detections) {
      // if the object is static in map frame, 将其标记为另一种类型
      if (
        abs(detection.twist.linear.x) <= static_velocity_thresh_ &&
        abs(detection.twist.linear.y) <= static_velocity_thresh_ &&
        abs(detection.twist.linear.z) <= static_velocity_thresh_)  // 其实z都是0
      {
        detection.twist.linear.x = 0.0;
        detection.twist.linear.y = 0.0;
        detection.twist.linear.z = 0.0;
        static_obstacle_msgs.detections.push_back(detection);
      } else {
        dynamic_obstacle_msgs.detections.push_back(detection);
      }
    }

    if (visualization_mode_ == "map") {
      if (AlignObstacleToFrame_) {
        AlignObstacleToFrame(static_obstacle_msgs);
        AlignObstacleToFrame(dynamic_obstacle_msgs);
      }

      ApplySmoothing(static_obstacle_msgs, static_obstacle_cache_);
      ApplySmoothing(dynamic_obstacle_msgs, dynamic_obstacle_cache_);

      static_obstacle_markers = StaticObstacleCubeMarker(static_obstacle_msgs);
      dynamic_obstacle_text_markers = DynamicObstacleTextMarker(dynamic_obstacle_msgs);
      dynamic_obstacle_array_markers = DynamicObstacleArrayMarker(dynamic_obstacle_msgs);
      dynamic_obstacle_cube_markers = DynamicObstacleCubeMarker(dynamic_obstacle_msgs);

      static_obstacle_to_map_pub_->publish(static_obstacle_msgs);
      dynamic_obstacle_to_map_pub_->publish(dynamic_obstacle_msgs);

      // 可视化
      ScaleMarkerAlpha(static_obstacle_markers, 0.5f);
      static_obstacle_markers_to_map_pub_->publish(static_obstacle_markers);
      ScaleMarkerAlpha(dynamic_obstacle_cube_markers, 0.5f);
      dynamic_obstacle_text_markers_to_map_pub_->publish(dynamic_obstacle_text_markers);
      dynamic_obstacle_array_markers_to_map_pub_->publish(dynamic_obstacle_array_markers);
      dynamic_obstacle_cube_markers_to_map_pub_->publish(dynamic_obstacle_cube_markers);

    } else if (visualization_mode_ == "baselink") {
      TransformObstacle(transition_frame_, target_frame_, static_obstacle_msgs, baselink_static_obstacle_msgs);
      TransformObstacle(transition_frame_, target_frame_, dynamic_obstacle_msgs, baselink_dynamic_obstacle_msgs);

      if (AlignObstacleToFrame_) {
        AlignObstacleToFrame(baselink_static_obstacle_msgs);
        AlignObstacleToFrame(baselink_dynamic_obstacle_msgs);
      }
      ApplySmoothing(baselink_static_obstacle_msgs, static_obstacle_cache_);
      ApplySmoothing(baselink_dynamic_obstacle_msgs, dynamic_obstacle_cache_);

      static_obstacle_markers = StaticObstacleCubeMarker(baselink_static_obstacle_msgs);
      dynamic_obstacle_text_markers = DynamicObstacleTextMarker(baselink_dynamic_obstacle_msgs);
      dynamic_obstacle_array_markers = DynamicObstacleArrayMarker(baselink_dynamic_obstacle_msgs);
      dynamic_obstacle_cube_markers = DynamicObstacleCubeMarker(baselink_dynamic_obstacle_msgs);

      static_obstacle_to_baselink_pub_->publish(baselink_static_obstacle_msgs);
      dynamic_obstacle_to_baselink_pub_->publish(baselink_dynamic_obstacle_msgs);

      // 可视化
      ScaleMarkerAlpha(static_obstacle_markers, 0.5f);
      static_obstacle_markers_pub_->publish(static_obstacle_markers);
      ScaleMarkerAlpha(dynamic_obstacle_cube_markers, 0.5f);
      dynamic_obstacle_cube_markers_pub_->publish(dynamic_obstacle_cube_markers);
      dynamic_obstacle_text_markers_pub_->publish(dynamic_obstacle_text_markers);
      dynamic_obstacle_array_markers_pub_->publish(dynamic_obstacle_array_markers);

    } else if (visualization_mode_ == "lidar_body") {
      ApplySmoothing(static_obstacle_msgs, static_obstacle_cache_);
      ApplySmoothing(dynamic_obstacle_msgs, dynamic_obstacle_cache_);

      static_obstacle_markers = StaticObstacleCubeMarker(static_obstacle_msgs);
      dynamic_obstacle_text_markers = DynamicObstacleTextMarker(dynamic_obstacle_msgs);
      dynamic_obstacle_array_markers = DynamicObstacleArrayMarker(dynamic_obstacle_msgs);
      dynamic_obstacle_cube_markers = DynamicObstacleCubeMarker(dynamic_obstacle_msgs);

      static_obstacle_pub_->publish(static_obstacle_msgs);
      dynamic_obstacle_pub_->publish(dynamic_obstacle_msgs);

      // 可视化
      ScaleMarkerAlpha(static_obstacle_markers, 0.5f);
      static_obstacle_markers_pub_->publish(static_obstacle_markers);
      ScaleMarkerAlpha(dynamic_obstacle_cube_markers, 0.5f);
      dynamic_obstacle_cube_markers_pub_->publish(dynamic_obstacle_cube_markers);
      dynamic_obstacle_text_markers_pub_->publish(dynamic_obstacle_text_markers);
      dynamic_obstacle_array_markers_pub_->publish(dynamic_obstacle_array_markers);
    }
  }

  /**
 * @brief 将障碍物检测消息从源坐标系转换到目标坐标系
 * 
 * 该函数接收一个障碍物检测数组消息，将其中每个障碍物的位置、线速度和角速度
 * 从源坐标系转换到目标坐标系，并将转换后的结果存储到输出消息中。
 * 转换过程使用TF2库进行坐标变换，确保在不同坐标系间的准确转换。
 * 
 * @param source_frame_ 源坐标系名称
 * @param target_frame_ 目标坐标系名称
 * @param obstacle_msg 输入的障碍物检测数组消息（源坐标系下）
 * @param custom_obstacle_array_msg 输出的障碍物检测数组消息（目标坐标系下）
 */
  void TransformObstacle(
    std::string source_frame_, std::string target_frame_,
    const lidar_detection::msg::ObstacleDetectionArray & obstacle_msg,
    lidar_detection::msg::ObstacleDetectionArray & custom_obstacle_array_msg)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Lookup transform failed: %s", ex.what());
      return;
    }

    for (const auto & detection : obstacle_msg.detections) {
      lidar_detection::msg::ObstacleDetection custom_obstacle_msg;
      custom_obstacle_msg = detection;  // copy detection (we'll overwrite fields as needed)

      // Transform bbox.center (use PoseStamped for safe transform)
      geometry_msgs::msg::PoseStamped in_pose, out_pose;
      in_pose.header.frame_id = source_frame_;
      in_pose.header.stamp = this->now();
      in_pose.pose = detection.detection.bbox.center;
      try {
        tf2::doTransform(in_pose, out_pose, transform_stamped);
        custom_obstacle_msg.detection.bbox.center = out_pose.pose;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform bbox center: %s", ex.what());
        continue;
      }

      // Transform linear velocity (Vector3Stamped)
      geometry_msgs::msg::Vector3Stamped linear_in, linear_out;
      linear_in.vector = detection.twist.linear;
      linear_in.header.frame_id = source_frame_;
      linear_in.header.stamp = this->now();
      try {
        tf2::doTransform(linear_in, linear_out, transform_stamped);
        custom_obstacle_msg.twist.linear = linear_out.vector;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform linear velocity: %s", ex.what());
      }

      // Transform angular velocity (Vector3Stamped)
      geometry_msgs::msg::Vector3Stamped angular_in, angular_out;
      angular_in.vector = detection.twist.angular;
      angular_in.header.frame_id = source_frame_;
      angular_in.header.stamp = this->now();
      try {
        tf2::doTransform(angular_in, angular_out, transform_stamped);
        custom_obstacle_msg.twist.angular = angular_out.vector;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform angular velocity: %s", ex.what());
      }

      custom_obstacle_msg.detection.header.frame_id = target_frame_;
      custom_obstacle_array_msg.detections.push_back(custom_obstacle_msg);
    }
  }

  void AlignObstacleToFrame(lidar_detection::msg::ObstacleDetectionArray & obstacle_msg_Array)
  {
    for (auto & detection_msg : obstacle_msg_Array.detections) {
      auto & bbox = detection_msg.detection.bbox;

      Eigen::Vector3d center(bbox.center.position.x, bbox.center.position.y, bbox.center.position.z);
      Eigen::Quaterniond q(
        bbox.center.orientation.w, bbox.center.orientation.x, bbox.center.orientation.y, bbox.center.orientation.z);
      if (q.norm() < 1e-6) {
        q = Eigen::Quaterniond::Identity();
      } else {
        q.normalize();
      }

      const Eigen::Matrix3d rotation = q.toRotationMatrix();
      const Eigen::Vector3d half_size(bbox.size.x * 0.5, bbox.size.y * 0.5, bbox.size.z * 0.5);

      Eigen::Vector3d min_pt(
        std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
      Eigen::Vector3d max_pt(
        std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest());

      for (int i = 0; i < 8; ++i) {
        Eigen::Vector3d corner(
          (i & 1) ? half_size.x() : -half_size.x(), (i & 2) ? half_size.y() : -half_size.y(),
          (i & 4) ? half_size.z() : -half_size.z());
        const Eigen::Vector3d world_corner = center + rotation * corner;
        min_pt = min_pt.cwiseMin(world_corner);
        max_pt = max_pt.cwiseMax(world_corner);
      }

      const Eigen::Vector3d aligned_center = 0.5 * (max_pt + min_pt);
      Eigen::Vector3d aligned_size = max_pt - min_pt;
      aligned_size = aligned_size.cwiseMax(Eigen::Vector3d::Zero());

      bbox.center.position.x = aligned_center.x();
      bbox.center.position.y = aligned_center.y();
      bbox.center.position.z = aligned_center.z();

      bbox.center.orientation.x = 0.0;
      bbox.center.orientation.y = 0.0;
      bbox.center.orientation.z = 0.0;
      bbox.center.orientation.w = 1.0;

      bbox.size.x = aligned_size.x();
      bbox.size.y = aligned_size.y();
      bbox.size.z = aligned_size.z();
    }
  }

  /**
 * @brief 计算两个bbox之间的匹配分数（基于位置和尺寸）
 */
  double ComputeMatchScore(const vision_msgs::msg::BoundingBox3D & bbox1, const vision_msgs::msg::BoundingBox3D & bbox2)
  {
    double dx = bbox1.center.position.x - bbox2.center.position.x;
    double dy = bbox1.center.position.y - bbox2.center.position.y;
    double dz = bbox1.center.position.z - bbox2.center.position.z;
    double position_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    auto compute_size_diff = [](double s1, double s2) {
      double max_size = std::max(s1, s2);
      if (max_size < 1e-6) return 0.0;
      return std::abs(s1 - s2) / max_size;
    };

    double size_diff_x = compute_size_diff(bbox1.size.x, bbox2.size.x);
    double size_diff_y = compute_size_diff(bbox1.size.y, bbox2.size.y);
    double size_diff_z = compute_size_diff(bbox1.size.z, bbox2.size.z);
    double avg_size_diff = (size_diff_x + size_diff_y + size_diff_z) / 3.0;

    // 综合分数：位置和尺寸都要接近
    if (position_dist > match_position_thresh_ || avg_size_diff > match_size_ratio_thresh_) {
      return std::numeric_limits<double>::max();  // 不匹配
    }

    return position_dist + avg_size_diff * 0.5;  // 位置权重更高
  }
  /**
 * @brief 对当前帧障碍物应用平滑，基于上一帧缓存
 */
  void ApplySmoothing(
    lidar_detection::msg::ObstacleDetectionArray & current_obstacles,
    lidar_detection::msg::ObstacleDetectionArray & obstacle_cache)
  {
    if (obstacle_cache.detections.empty()) {
      obstacle_cache = current_obstacles;
      return;
    }

    lidar_detection::msg::ObstacleDetectionArray smoothed_obstacles;
    smoothed_obstacles.header = current_obstacles.header;

    for (auto & current_det : current_obstacles.detections) {
      int best_match_idx = -1;
      double best_score = std::numeric_limits<double>::max();

      for (size_t i = 0; i < obstacle_cache.detections.size(); ++i) {
        double score = ComputeMatchScore(current_det.detection.bbox, obstacle_cache.detections[i].detection.bbox);

        if (score < best_score) {
          best_score = score;
          best_match_idx = static_cast<int>(i);
        }
      }

      lidar_detection::msg::ObstacleDetection smoothed_det = current_det;

      if (best_match_idx >= 0 && best_score < std::numeric_limits<double>::max()) {
        const auto & prev_bbox = obstacle_cache.detections[best_match_idx].detection.bbox;
        auto & curr_bbox = smoothed_det.detection.bbox;

        // 指数移动平均：smoothed = alpha * prev + (1 - alpha) * curr
        curr_bbox.center.position.x =
          smoothing_alpha_ * prev_bbox.center.position.x + (1.0 - smoothing_alpha_) * curr_bbox.center.position.x;
        curr_bbox.center.position.y =
          smoothing_alpha_ * prev_bbox.center.position.y + (1.0 - smoothing_alpha_) * curr_bbox.center.position.y;
        curr_bbox.center.position.z =
          smoothing_alpha_ * prev_bbox.center.position.z + (1.0 - smoothing_alpha_) * curr_bbox.center.position.z;

        curr_bbox.size.x = smoothing_alpha_ * prev_bbox.size.x + (1.0 - smoothing_alpha_) * curr_bbox.size.x;
        curr_bbox.size.y = smoothing_alpha_ * prev_bbox.size.y + (1.0 - smoothing_alpha_) * curr_bbox.size.y;
        curr_bbox.size.z = smoothing_alpha_ * prev_bbox.size.z + (1.0 - smoothing_alpha_) * curr_bbox.size.z;
      }

      smoothed_obstacles.detections.push_back(smoothed_det);
    }

    current_obstacles = smoothed_obstacles;
    obstacle_cache = smoothed_obstacles;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleToOtherFrameNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
