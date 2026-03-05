/*
 * Copyright 2026 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LIFECYCLE_OFFLINE_REFLECTOR_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LIFECYCLE_OFFLINE_REFLECTOR_NODE_H

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/publisher.hpp"
#include <rclcpp/rclcpp.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

#include "cartographer/mapping/map_builder_interface.h"
// cartographer_ros进行加载lua配置文件
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros_msgs/msg/bagfile_progress.hpp"

#include "byd_mapbuilder_msgs/msg/map_build_process.hpp"
#include "byd_mapbuilder_msgs/msg/map_build_status.hpp"
#include "byd_mapbuilder_msgs/srv/map_build.hpp"

// Reflector detection headers
#include "amr_reflector_noise_handling/circle_fitter.hpp"
#include "amr_reflector_noise_handling/common/time_order_queue.hpp"
#include "amr_reflector_noise_handling/distort_corrector.hpp"
#include "amr_reflector_noise_handling/fixed_dbscan.hpp"
#include "amr_reflector_noise_handling/global_reflector_tracker.hpp"
#include "amr_reflector_noise_handling/pca_shape_classifier.hpp"
#include "amr_reflector_noise_handling/reflector_detector.hpp"
#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/types/msg_conversion.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"

using namespace std::chrono_literals;

namespace cartographer_ros {

using MapBuilderFactory =
    std::function<std::unique_ptr<::cartographer::mapping::MapBuilderInterface>(
        const ::cartographer::mapping::proto::MapBuilderOptions &)>;

/**
 * @brief Frame data structure for storing laser scan and odometry data
 */
struct ReflectorFrameData {
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  int64_t timestamp; // nanoseconds
  std::vector<int64_t> between_next_odoms;
  std::vector<amr_reflector_noise_handling::TimeRigid3d> between_odoms;
  size_t odom_count;
  size_t frame_index; // laserscan index

  // Compensated point cloud (after distortion correction)
  std::vector<amr_reflector_noise_handling::Point> compensated_points;
  std::vector<amr_reflector_noise_handling::Point> filtered_points;

  // Detected reflectors
  std::vector<amr_reflector_noise_handling::DetectedReflector> reflectors;

  // Global pose in world frame
  transforms::Rigid3d global_pose;

  ReflectorFrameData() : frame_index(0) {}
};

/**
 * @brief Lifecycle node for offline mapping with reflector detection and
 * landmark integration
 *
 * This node combines Cartographer's SLAM capabilities with reflector detection
 * to:
 * 1. Process rosbag data offline for mapping
 * 2. Detect and track reflectors from laser scan data
 * 3. Write tracked reflectors as landmarks into Cartographer's pose graph
 * 4. Support lifecycle management for ROS2
 */
class LifecycleOfflineReflectorNode : public rclcpp_lifecycle::LifecycleNode {
  enum MapBuildStatus {
    STATUS_IDEL = 0,
    STATUS_BUILDING,
    STATUS_COMPLETE,
    STATUS_CANCEL,
    STATUS_ERROR,
  };

public:
  explicit LifecycleOfflineReflectorNode(
      const std::string &node_name, bool intra_process_comms = false,
      rclcpp::Executor::SharedPtr executor = nullptr,
      const MapBuilderFactory &map_builder_factory = nullptr);

  /// @brief Configuration callback
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state) override;

  /// @brief Activation callback - starts the mapping thread
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state) override;

  /// @brief Deactivation callback - stops mapping and saves results
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state) override;

  /// @brief Cleanup callback
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  /// @brief Shutdown callback
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state);

private:
  // Create configuration
  bool create_options();

  // Bag progress callback
  void bagProcessDataCallback(
      const cartographer_ros_msgs::msg::BagfileProgress::SharedPtr msg);

  // Service callback for map building
  void map_build_callback(
      const std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Request>
          request,
      std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Response> response);

  // ==================== Reflector Detection Methods ====================

  /**
   * @brief Configure detection modules from ROS parameters
   */
  void configureDetectionModules();

  /**
   * @brief Configure distortion corrector
   */
  void configureDistortionCorrector();

  /**
   * @brief Configure DBSCAN clustering
   */
  void configureFixedDBSCAN();

  /**
   * @brief Configure PCA shape classification
   */
  void configurePCAClassification();

  /**
   * @brief Configure circle fitting
   */
  void configureCircleFit();

  /**
   * @brief Configure global reflector tracker
   */
  void configureGlobalTracker();

  /**
   * @brief Store laser scan data
   */
  void setLaserScan(int64_t timestamp,
                    sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  // 重新加载所有帧数据
  bool loadAllFrames();
  /**
   * @brief Process a single frame for reflector detection
   */
  void
  processFrame(size_t frame_index, cartographer_ros::Node &node,
               int trajectory_id,
               const std::vector<TrajectoryOptions> &bag_trajectory_options);
  void writeReflectorsToPbstream(cartographer_ros::Node& node);

  /**
   * @brief Convert tracked reflectors to LandmarkList message
   */
  cartographer_ros_msgs::msg::LandmarkList::SharedPtr
  convertTrackedReflectorsToLandmarkList(
      const std::vector<amr_reflector_noise_handling::TrackedReflector>
          &tracked_reflectors,
      int64_t timestamp, const std::string &frame_id = "map");

  /**
   * @brief Convert LaserScan to timed points
   */
  static std::vector<amr_reflector_noise_handling::TimePoint>
  convertScanToTimedPoints(
      const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  /**
   * @brief Convert LaserScan to points
   */
  static std::vector<amr_reflector_noise_handling::Point>
  convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  /**
   * @brief Filter points by intensity
   */
  std::vector<amr_reflector_noise_handling::Point> filterByIntensity(
      const std::vector<amr_reflector_noise_handling::Point> &points,
      double threshold);

  // ==================== Member Variables ====================

  // Lifecycle publisher for process status
  rclcpp_lifecycle::LifecyclePublisher<
      byd_mapbuilder_msgs::msg::MapBuildProcess>::SharedPtr process_pub_;
  rclcpp::Subscription<cartographer_ros_msgs::msg::BagfileProgress>::SharedPtr
      bag_process_sub_;

  // ROS node for cartographer
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Executor::SharedPtr executor_;

  // File paths
  std::string bag_file_path_;
  std::string output_pbstream_path_;
  std::string urdf_path_;
  std::string cartographer_install_dir_;
  std::string cartographer_output_dir_;
  std::string rosbag_dir_;
  std::string cartoConfig_dir_;
  std::string cartographer_shared_dir_;
  std::string default_configuration_basename_;

  // Map builder factory
  MapBuilderFactory map_builder_factory_;

  // Landmark weights for pose graph
  double landmark_translation_weight_;
  double landmark_rotation_weight_;

  // Mapping thread
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr carto_executor_;

  // Build status
  MapBuildStatus map_build_status_;
  bool enable_mapping_;

  // Service server
  rclcpp::Service<byd_mapbuilder_msgs::srv::MapBuild>::SharedPtr map_build_srv_;

  // Map resolution and filestem
  float resolution_;
  std::string map_filestem_;

  // ==================== Reflector Detection Members ====================
  std::string reflector_param_file_;
  std::string reflector_scan_frame_;
  std::string reflector_base_frame_;

  // Parameters
  std::string scan_topic_;
  std::string odom_topic_;
  std::string classification_method_;
  double raw_intensity_threshold_;
  double min_confidence_;

  // Laser and odometry data storage
  std::unordered_map<int64_t, sensor_msgs::msg::LaserScan::SharedPtr> scan_map_;
  std::vector<int64_t> odom_timestamps_;
  amr_reflector_noise_handling::TimeOrderQueue<transforms::Rigid3d> odom_queue_;
  amr_reflector_noise_handling::TimeOrderQueue<transforms::Rigid3d>
      globalpose_queue_;
  std::vector<int64_t> scan_timestamps_;
  std::vector<std::shared_ptr<ReflectorFrameData>> frames_;
  size_t current_frame_index_;

  // Laser to base transform
  geometry_msgs::msg::TransformStamped laser_to_base_;
  bool has_laser_to_base_{false};

  // Distortion corrector
  bool use_distort_corrector_;
  std::string distortcorrect_method_;

  // Clustering
  amr_reflector_noise_handling::FixedDBSCAN fixed_dbscan_;

  // Detection modules
  bool use_short_tracker_;
  amr_reflector_noise_handling::RefelctorDetector reflector_detector_;

  // TF buffer for transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Map to odom transform
  transforms::Rigid3d map_odom_;
  bool map_odom_initialized_{false};
};

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LIFECYCLE_OFFLINE_REFLECTOR_NODE_H