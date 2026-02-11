/*
Author: LeonZack055 (Gmail)
offline_reflector_mapping_node.cpp (c) 2026
Desc: 基于反光柱的离线建图
@copyright Copyright (c)  <LeonZack055> All rights reserved.
@license BSD 2-Clause License
Created:  2026-02-08T09:26:46.049Z
Modified: !date!
*/

#include <errno.h>

#include <string>
#ifndef WIN32
#include <sys/resource.h>
#endif
#include <time.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/playable_bag.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/urdf_reader.h"
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "gflags/gflags.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#ifdef USE_URDF_H_FILES
#include "urdf/model.h"
#else
#include "urdf/model.hpp"
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <termios.h>  // 终端控制头文件
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>  // STDIN_FILENO

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

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
#include "amr_reflector_noise_handling/visualization_helper.hpp"
#include "rclcpp/exceptions.hpp"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(
    configuration_basenames, "",
    "Comma-separated list of basenames, i.e. not containing any "
    "directory prefix, of the configuration files for each trajectory. "
    "The first configuration file will be used for node options. "
    "If less configuration files are specified than trajectories, the "
    "first file will be used the remaining trajectories.");
DEFINE_string(
    bag_filenames, "",
    "Comma-separated list of bags to process. One bag per trajectory. "
    "Any combination of simultaneous and sequential bags is supported.");
DEFINE_string(urdf_filenames, "",
              "Comma-separated list of one or more URDF files that contain "
              "static links for the sensor configuration(s).");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish transforms from bags.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_string(save_state_filename, "",
              "Explicit name of the file to which the serialized state will be "
              "written before shutdown. If left empty, the filename will be "
              "inferred from the first bagfile's name as: "
              "<bag_filenames[0]>.pbstream");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");
DEFINE_double(skip_seconds, 0,
              "Optional amount of seconds to skip from the beginning "
              "(i.e. when the earliest bag starts.). ");

using namespace amr_reflector_noise_handling;
using namespace cartographer::mapping;

/**
 * @brief 记录激光雷达历史队列
 */
struct FrameData {
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  int64_t timestamp;  // nanoseconds
  std::vector<int64_t> between_next_odoms;
  std::vector<TimeRigid3d> between_odoms;
  size_t odom_count;
  size_t frame_index;  // laserscan的索引

  // Compensated point cloud (after distortion correction)
  std::vector<Point> compensated_points;
  std::vector<Point> filtered_points;

  // Detected reflectors
  std::vector<DetectedReflector> reflectors;

  // Global pose in world frame
  transforms::Rigid3d global_pose;

  FrameData() : frame_index(0) {}
};

// 关闭终端行缓冲和回显，实现无回车读单个字符
char get_char_without_enter() {
  struct termios old_attr, new_attr;
  tcgetattr(STDIN_FILENO, &old_attr);  // 获取原有终端属性
  new_attr = old_attr;
  new_attr.c_lflag &= ~(ICANON | ECHO);  // 关闭行缓冲(ICANON)、关闭回显(ECHO)
  tcsetattr(STDIN_FILENO, TCSANOW, &new_attr);  // 立即应用新属性

  char c = getchar();  // 此时无需回车，输入单个字符立即返回

  tcsetattr(STDIN_FILENO, TCSANOW, &old_attr);  // 恢复原有终端属性（必做！）
  return c;
}

namespace cartographer_ros {
constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "/tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;
// We publish tf messages one second earlier than other messages. Under
// the assumption of higher frequency tf this should ensure that tf can
// always interpolate.
const rclcpp::Duration kDelay(1.0, 0);

// 创建
transforms::Rigid3d convert_carto_transform(
    const cartographer::transform::Rigid3d& transform) {
  return transforms::Rigid3d(transform.translation(), transform.rotation());
}

/**
 * @brief Interactive bag processing node
 */
class ReflectorNoiseBagNode : public rclcpp::Node {
 public:
  ReflectorNoiseBagNode()
      : Node("reflector_carto_bag_mapping"),
        current_frame_index_(0),
        auto_mode_(false),
        should_exit_(false) {
    // Declare parameters
    this->declare_parameter("bag_path", "");
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("odom_topic", "/odom_combined");
    this->declare_parameter("classification_method", "pca");
    this->declare_parameter("raw_intensity_threshold", 1000.0);
    this->declare_parameter("min_confidence", 0.5);
    // Get parameters
    bag_path_ = this->get_parameter("bag_path").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    classification_method_ =
        this->get_parameter("classification_method").as_string();
    raw_intensity_threshold_ =
        this->get_parameter("raw_intensity_threshold").as_double();
    min_confidence_ = this->get_parameter("min_confidence").as_double();

    // 配置点云矫正器
    configureDistortionCorrector();
    // 配置聚类器
    configureFixedDBSCAN();
    // Configure detection modules
    configureDetectionModules();

    RCLCPP_INFO(this->get_logger(), "反光柱逐帧检测节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "Bag路径: %s", bag_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "扫描话题: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "里程计话题: %s", odom_topic_.c_str());
  }

  /**
   * @brief Run the bag processing
   */
  void run() {
    // 运行离线建图
    const cartographer_ros::MapBuilderFactory map_builder_factory =
        [](const ::cartographer::mapping::proto::MapBuilderOptions&
               map_builder_options) {
          return ::cartographer::mapping::CreateMapBuilder(map_builder_options);
        };
    this->RunOfflineNode(map_builder_factory, shared_from_this());
    // Pre-load all frames
    if (!loadAllFrames()) {
      RCLCPP_ERROR(this->get_logger(), "回溯激光里程失败");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "加载完成，共 %zu 帧", frames_.size());

    // 创建反光柱可视化器
    visualization_helper_ =
        std::make_shared<VisualizationHelper>(shared_from_this());
    // Create timer for continuous publishing (10 Hz)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ReflectorNoiseBagNode::publishTimerCallback, this));

    // Start keyboard input thread
    std::thread input_thread(&ReflectorNoiseBagNode::keyboardInputThread, this);

    // Process first frame
    processFrame(0);

    // Spin the node (this will run the timer)
    rclcpp::spin(shared_from_this());

    // Wait for input thread to finish
    should_exit_ = true;
    // input_thread.join();

    RCLCPP_INFO(this->get_logger(), "处理完成");
  }

  // 离线激光雷达离线处理
  void RunOfflineNode(const MapBuilderFactory& map_builder_factory,
                      rclcpp::Node::SharedPtr cartographer_offline_node) {
    RunOfflineNodeImpl(map_builder_factory, cartographer_offline_node);
  }

 private:
  /**
   * @brief Implementation of RunOfflineNode - processes bag files for offline
   * SLAM
   */
  void RunOfflineNodeImpl(const MapBuilderFactory& map_builder_factory,
                          rclcpp::Node::SharedPtr cartographer_offline_node) {
    // Validate configuration parameters
    CHECK(!FLAGS_configuration_directory.empty())
        << "-configuration_directory is missing.";
    LOG(WARNING) << "FLAGS_configuration_directory "
                 << FLAGS_configuration_directory;
    CHECK(!FLAGS_configuration_basenames.empty())
        << "-configuration_basenames is missing.";
    LOG(WARNING) << "FLAGS_configuration_basenames "
                 << FLAGS_configuration_basenames;
    CHECK(!(FLAGS_bag_filenames.empty() && FLAGS_load_state_filename.empty()))
        << "-bag_filenames and -load_state_filename cannot both be "
           "unspecified.";

    // Parse bag filenames
    std::vector<std::string> bag_filenames;
    if (!FLAGS_bag_filenames.empty()) {
      std::regex regex(",");
      std::vector<std::string> if_bag_filenames(
          std::sregex_token_iterator(FLAGS_bag_filenames.begin(),
                                     FLAGS_bag_filenames.end(), regex, -1),
          std::sregex_token_iterator());
      bag_filenames = if_bag_filenames;
    }

    // Load node options and trajectory configurations
    cartographer_ros::NodeOptions node_options;
    std::regex regex(",");
    std::vector<std::string> configuration_basenames(
        std::sregex_token_iterator(FLAGS_configuration_basenames.begin(),
                                   FLAGS_configuration_basenames.end(), regex,
                                   -1),
        std::sregex_token_iterator());

    // Load trajectory options for each bag
    std::vector<TrajectoryOptions> bag_trajectory_options(1);
    std::tie(node_options, bag_trajectory_options.at(0)) = LoadOptions(
        FLAGS_configuration_directory, configuration_basenames.at(0));

    // Handle multiple bags with different configurations
    for (size_t bag_index = 1; bag_index < bag_filenames.size(); ++bag_index) {
      TrajectoryOptions current_trajectory_options;
      if (bag_index < configuration_basenames.size()) {
        std::tie(std::ignore, current_trajectory_options) =
            LoadOptions(FLAGS_configuration_directory,
                        configuration_basenames.at(bag_index));
      } else {
        current_trajectory_options = bag_trajectory_options.at(0);
      }
      bag_trajectory_options.push_back(current_trajectory_options);
    }

    if (bag_filenames.size() > 0) {
      CHECK_EQ(bag_trajectory_options.size(), bag_filenames.size());
    }

    // Configure map builder
    node_options.lookup_transform_timeout_sec = 0.;
    auto map_builder = map_builder_factory(node_options.map_builder_options);

    // Initialize timing
    const std::chrono::time_point<std::chrono::steady_clock> start_time =
        std::chrono::steady_clock::now();

    // Initialize TF buffer with member variable
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        cartographer_offline_node->get_clock(), tf2::durationFromSec(10),
        cartographer_offline_node);

    // Load static transforms from URDF files
    std::vector<geometry_msgs::msg::TransformStamped> urdf_transforms;
    if (!FLAGS_urdf_filenames.empty()) {
      std::vector<std::string> urdf_filenames(
          std::sregex_token_iterator(FLAGS_urdf_filenames.begin(),
                                     FLAGS_urdf_filenames.end(), regex, -1),
          std::sregex_token_iterator());
      for (const auto& urdf_filename : urdf_filenames) {
        const auto current_urdf_transforms =
            ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer_);
        urdf_transforms.insert(urdf_transforms.end(),
                               current_urdf_transforms.begin(),
                               current_urdf_transforms.end());
      }
    }

    // Enable dedicated thread for TF processing
    tf_buffer_->setUsingDedicatedThread(true);

    // Create cartographer node
    cartographer_node_ = std::make_unique<cartographer_ros::Node>(
        node_options, std::move(map_builder), tf_buffer_,
        cartographer_offline_node, FLAGS_collect_metrics);
    if (!FLAGS_load_state_filename.empty()) {
      cartographer_node_->LoadState(FLAGS_load_state_filename,
                                    FLAGS_load_frozen_state);
    }

    // Initialize publishers
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher =
        cartographer_offline_node->create_publisher<tf2_msgs::msg::TFMessage>(
            kTfTopic, kLatestOnlyPublisherQueueSize);
    ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(
        cartographer_offline_node);

    // Initialize clock publisher with member variable
    clock_publisher_ =
        cartographer_offline_node->create_publisher<rosgraph_msgs::msg::Clock>(
            kClockTopic, kLatestOnlyPublisherQueueSize);

    // Publish static transforms
    if (urdf_transforms.size() > 0) {
      static_tf_broadcaster.sendTransform(urdf_transforms);
    }

    // Compute expected sensor IDs for each bag
    std::vector<
        std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
        bag_expected_sensor_ids;
    if (configuration_basenames.size() == 1) {
      const auto current_bag_expected_sensor_ids =
          cartographer_node_->ComputeDefaultSensorIdsForMultipleBags(
              {bag_trajectory_options.front()});
      bag_expected_sensor_ids = {bag_filenames.size(),
                                 current_bag_expected_sensor_ids.front()};
    } else {
      bag_expected_sensor_ids =
          cartographer_node_->ComputeDefaultSensorIdsForMultipleBags(
              bag_trajectory_options);
    }
    CHECK_EQ(bag_expected_sensor_ids.size(), bag_filenames.size());

    // Create bag topic to sensor ID mapping
    std::map<std::pair<int /* bag_index */, std::string>,
             cartographer::mapping::TrajectoryBuilderInterface::SensorId>
        bag_topic_to_sensor_id;
    PlayableBagMultiplexer playable_bag_multiplexer(cartographer_offline_node);

    // Bind sensor IDs for each bag
    for (size_t current_bag_index = 0; current_bag_index < bag_filenames.size();
         ++current_bag_index) {
      const std::string& bag_filename = bag_filenames.at(current_bag_index);
      if (!rclcpp::ok()) {
        return;
      }
      for (const auto& expected_sensor_id :
           bag_expected_sensor_ids.at(current_bag_index)) {
        LOG(INFO) << "expected_sensor_id.id " << expected_sensor_id.id;
        const auto bag_resolved_topic = std::make_pair(
            static_cast<int>(current_bag_index), "/" + expected_sensor_id.id);
        if (bag_topic_to_sensor_id.count(bag_resolved_topic) != 0) {
          LOG(ERROR) << "Sensor /" << expected_sensor_id.id << " of bag "
                     << current_bag_index << " resolves to topic "
                     << bag_resolved_topic.second
                     << " which is already used by "
                     << " sensor "
                     << bag_topic_to_sensor_id.at(bag_resolved_topic).id;
        }
        bag_topic_to_sensor_id[bag_resolved_topic] = expected_sensor_id;
      }

      // Add playable bag with TF message handler
      auto serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
      playable_bag_multiplexer.AddPlayableBag(PlayableBag(
          bag_filename, current_bag_index, kDelay,
          [this, &tf_publisher, cartographer_offline_node, serializer](
              std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {
            if (msg->topic_name == kTfTopic ||
                msg->topic_name == kTfStaticTopic) {
              if (FLAGS_use_bag_transforms) {
                tf2_msgs::msg::TFMessage tf_message;
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                try {
                  serializer.deserialize_message(&serialized_msg, &tf_message);
                  for (auto& transform : tf_message.transforms) {
                    try {
                      tf_buffer_->setTransform(
                          transform, "unused_authority",
                          msg->topic_name == kTfStaticTopic);
                    } catch (const tf2::TransformException& ex) {
                      LOG(WARNING) << ex.what();
                    }
                  }
                  tf_publisher->publish(tf_message);
                } catch (const rclcpp::exceptions::RCLError& rcl_error) {
                  return true;
                }
              }
              return false;
            } else {
              return true;
            }
          }));
    }

    // Verify topic bindings
    std::set<std::string> bag_topics;
    std::stringstream bag_topics_string;
    for (const auto& topic : playable_bag_multiplexer.topics()) {
      std::string resolved_topic =
          cartographer_offline_node->get_node_base_interface()
              ->resolve_topic_or_service_name(topic, false);
      bag_topics.insert(resolved_topic);
      bag_topics_string << resolved_topic << ",";
    }

    bool print_topics = false;
    for (const auto& entry : bag_topic_to_sensor_id) {
      const std::string& resolved_topic = entry.first.second;
      if (bag_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected resolved topic \"" << resolved_topic
                     << "\" not found in bag file(s).";
        print_topics = true;
      }
    }
    if (print_topics) {
      LOG(WARNING) << "Available topics in bag file(s) are "
                   << bag_topics_string.str();
    }

    // Process messages and generate trajectories
    std::unordered_map<int, int> bag_index_to_trajectory_id;
    const rclcpp::Time begin_time =
        playable_bag_multiplexer.IsMessageAvailable()
            ? playable_bag_multiplexer.PeekMessageTime()
            : rclcpp::Time();

    // Initialize message serializers
    auto laser_scan_serializer =
        rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
    auto multi_echo_laser_scan_serializer =
        rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
    auto pcl2_serializer =
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
    auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
    auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
    auto nav_sat_fix_serializer =
        rclcpp::Serialization<sensor_msgs::msg::NavSatFix>();
    auto landmark_list_serializer =
        rclcpp::Serialization<cartographer_ros_msgs::msg::LandmarkList>();

    // Process bag messages
    while (playable_bag_multiplexer.IsMessageAvailable()) {
      if (!::rclcpp::ok()) {
        return;
      }

      const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
      const rosbag2_storage::SerializedBagMessage& msg =
          std::get<0>(next_msg_tuple);
      const int bag_index = std::get<1>(next_msg_tuple);
      const std::string topic_type = std::get<2>(next_msg_tuple);
      const bool is_last_message_in_bag = std::get<3>(next_msg_tuple);

      // Skip messages before start time
#ifdef PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME
      if (msg.time_stamp <
          (begin_time.nanoseconds() +
           rclcpp::Duration(FLAGS_skip_seconds, 0).nanoseconds())) {
        continue;
      }
#else
      if (msg.recv_timestamp <
          (begin_time.nanoseconds() +
           rclcpp::Duration(FLAGS_skip_seconds, 0).nanoseconds())) {
        continue;
      }
#endif

      // Get or create trajectory ID
      int trajectory_id;
      if (bag_index_to_trajectory_id.count(bag_index) == 0) {
        trajectory_id = cartographer_node_->AddOfflineTrajectory(
            bag_expected_sensor_ids.at(bag_index),
            bag_trajectory_options.at(bag_index));
        CHECK(bag_index_to_trajectory_id
                  .emplace(std::piecewise_construct,
                           std::forward_as_tuple(bag_index),
                           std::forward_as_tuple(trajectory_id))
                  .second);
        LOG(INFO) << "Assigned trajectory " << trajectory_id << " to bag "
                  << bag_filenames.at(bag_index);
      } else {
        trajectory_id = bag_index_to_trajectory_id.at(bag_index);
      }

      // Map bag topic to sensor ID
      const auto bag_topic = std::make_pair(
          bag_index,
          cartographer_offline_node->get_node_base_interface()
              ->resolve_topic_or_service_name(msg.topic_name, false));
      auto it = bag_topic_to_sensor_id.find(bag_topic);

      // Process message based on type
      if (it != bag_topic_to_sensor_id.end()) {
        const std::string& sensor_id = it->second.id;

        if (topic_type == "sensor_msgs/msg/LaserScan") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg =
              std::make_shared<sensor_msgs::msg::LaserScan>();
          laser_scan_serializer.deserialize_message(&serialized_msg,
                                                    laser_scan_msg.get());
          cartographer_node_->HandleLaserScanMessage(trajectory_id, sensor_id,
                                                     laser_scan_msg);
          if (msg.topic_name == this->scan_topic_) {
            int64_t laserTimeStamp =
                rclcpp::Time(laser_scan_msg->header.stamp).nanoseconds();
            this->setLaserScan(laserTimeStamp, laser_scan_msg);
          }
        } else if (topic_type == "sensor_msgs/msg/MultiEchoLaserScan") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::MultiEchoLaserScan::SharedPtr
              multi_echo_laser_scan_msg =
                  std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
          multi_echo_laser_scan_serializer.deserialize_message(
              &serialized_msg, multi_echo_laser_scan_msg.get());
          cartographer_node_->HandleMultiEchoLaserScanMessage(
              trajectory_id, sensor_id, multi_echo_laser_scan_msg);
        } else if (topic_type == "sensor_msgs/msg/PointCloud2") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::PointCloud2::SharedPtr pcl2_scan_msg =
              std::make_shared<sensor_msgs::msg::PointCloud2>();
          pcl2_serializer.deserialize_message(&serialized_msg,
                                              pcl2_scan_msg.get());
          cartographer_node_->HandlePointCloud2Message(trajectory_id, sensor_id,
                                                       pcl2_scan_msg);
        } else if (topic_type == "sensor_msgs/msg/Imu") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::Imu::SharedPtr imu_scan_msg =
              std::make_shared<sensor_msgs::msg::Imu>();
          imu_serializer.deserialize_message(&serialized_msg,
                                             imu_scan_msg.get());
          cartographer_node_->HandleImuMessage(trajectory_id, sensor_id,
                                               imu_scan_msg);
        } else if (topic_type == "nav_msgs/msg/Odometry") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          nav_msgs::msg::Odometry::SharedPtr odom_scan_msg =
              std::make_shared<nav_msgs::msg::Odometry>();
          odom_serializer.deserialize_message(&serialized_msg,
                                              odom_scan_msg.get());
          cartographer_node_->HandleOdometryMessage(trajectory_id, sensor_id,
                                                    odom_scan_msg);
          if (msg.topic_name == this->odom_topic_) {
            int64_t odom_timestamp =
                rclcpp::Time(odom_scan_msg->header.stamp).nanoseconds();
            odom_timestamps_.push_back(odom_timestamp);
            odom_queue_.push(
                TimeRigid3d(transforms::ToRigid3d(odom_scan_msg->pose.pose),
                            odom_timestamp));
          }
        } else if (topic_type == "sensor_msgs/msg/NavSatFix") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_msg =
              std::make_shared<sensor_msgs::msg::NavSatFix>();
          nav_sat_fix_serializer.deserialize_message(&serialized_msg,
                                                     nav_sat_fix_msg.get());
          cartographer_node_->HandleNavSatFixMessage(trajectory_id, sensor_id,
                                                     nav_sat_fix_msg);
        } else if (topic_type == "cartographer_ros_msgs/msg/LandmarkList") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          cartographer_ros_msgs::msg::LandmarkList::SharedPtr
              landmark_list_msg =
                  std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
          landmark_list_serializer.deserialize_message(&serialized_msg,
                                                       landmark_list_msg.get());
          cartographer_node_->HandleLandmarkMessage(trajectory_id, sensor_id,
                                                    landmark_list_msg);
        }
      }

      // Publish clock message using member variable
#ifdef PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME
      clock_msg_.clock = rclcpp::Time(msg.time_stamp);
#else
      clock_msg_.clock = rclcpp::Time(msg.recv_timestamp);
#endif
      clock_publisher_->publish(clock_msg_);

      // Spin the node for message processing
      rclcpp::spin_some(cartographer_offline_node);

      // Finish trajectory if this is the last message
      if (is_last_message_in_bag) {
        cartographer_node_->FinishTrajectory(trajectory_id);
      }
    }

    // Create clock republish timer using member variable to keep it alive
    clock_republish_timer_ = cartographer_offline_node->create_wall_timer(
        std::chrono::milliseconds(int(kClockPublishFrequencySec)),
        [this]() { clock_publisher_->publish(clock_msg_); });

    // Run final optimization
    cartographer_node_->RunFinalOptimization();

    // Log timing statistics
    const std::chrono::time_point<std::chrono::steady_clock> end_time =
        std::chrono::steady_clock::now();
    const double wall_clock_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                                  start_time)
            .count();

    LOG(INFO) << "Elapsed wall clock time: " << wall_clock_seconds << " s";
#ifdef __linux__
    timespec cpu_timespec = {};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec);
    LOG(INFO) << "Elapsed CPU time: "
              << (cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec) << " s";
    rusage usage;
    CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0) << strerror(errno);
    LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
#endif

    // Serialize state if needed
    if (rclcpp::ok() &&
        !(bag_filenames.empty() && FLAGS_save_state_filename.empty())) {
      LOG(INFO) << "完成Cartographer离线建图..... 准备进反光柱检测回溯......";
    }

    // Extract trajectory node poses for reflector detection
    auto ros_mapbuilder_bridge = cartographer_node_->map_builder_bridge_;
    cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr
        poses_requeset = std::make_shared<
            cartographer_ros_msgs::srv::TrajectoryQuery::Request>();
    poses_requeset->trajectory_id = 0;
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr
        poses_res_ptr = std::make_shared<
            cartographer_ros_msgs::srv::TrajectoryQuery::Response>();
    ros_mapbuilder_bridge->HandleTrajectoryQuery(poses_requeset, poses_res_ptr);

    int odom_indx = 0;
    for (const auto& geo_msg : poses_res_ptr->trajectory) {
      int64_t timestamp = rclcpp::Time(geo_msg.header.stamp).nanoseconds();
      // Add odometry to the time-ordered queue
      globalpose_queue_.push(
          TimeRigid3d(transforms::ToRigid3d(geo_msg.pose), timestamp));
    }
    // 完成反光柱回溯
    LOG(INFO) << "完成轨迹回灌，进行放光柱检测回溯...... GlobalPose队列长度: "
              << globalpose_queue_.size();
  }

  // 配置算法相关参数，从ros2的参数服务器中提取，只提取一次
  void configureDetectionModules() {
    // Configure PCA classifier
    configurePCAClassification();
    // Configure circle fittercircle_fitter_
    configureCicrleFit();
    // Configure GlobalTracker
    configureGlobalTracker();
  }

  void setLaserScan(int64_t timestamp,
                    sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    if (scan_map_.find(timestamp) == scan_map_.end()) {
      scan_map_[timestamp] = scan_msg;
      scan_timestamps_.push_back(timestamp);
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "scan消息: 存在重复时间戳");
    }
  }

  /**
   * @brief 使用RunOptimization进行代码加载
   * WARN: 由于里程计前后时间跳变, 这里选用录包时刻的系统时间戳为Laser时间
   */
  bool loadAllFrames() {
    LOG(INFO) << "双拍回溯激光时间序列与里程计时间戳";
    // TODO: 时间队列处理，利用RunOfflineNode()中multiplery进行数据加载
    size_t frame_idx = 0;
    std::vector<int64_t>::const_iterator odom_peek = odom_timestamps_.cbegin();
    std::sort(scan_timestamps_.begin(), scan_timestamps_.end());
    std::sort(odom_timestamps_.begin(), odom_timestamps_.end());
    for (std::vector<int64_t>::const_iterator scan_iterator =
             scan_timestamps_.cbegin();
         scan_iterator < scan_timestamps_.cend() - 1; ++scan_iterator) {
      const auto& next_scan_iterator = scan_iterator + 1;
      const auto& scan_time = *scan_iterator;
      const auto& next_scan_time = *next_scan_iterator;
      auto frame = std::make_shared<FrameData>();
      frame->scan = scan_map_[scan_time];
      frame->timestamp = scan_time;
      frame->frame_index = frame_idx++;
      // Find closest odometry
      std::vector<int64_t>::const_iterator keep_odom_peek = odom_peek;
      for (; odom_peek < odom_timestamps_.cend(); ++odom_peek) {
        if (*odom_peek <= scan_time) {
          frame->between_next_odoms.push_back(*odom_peek);
          keep_odom_peek++;  // 保留当前的odom_peek，用于下一次迭代，查找相邻帧的数据
        }
        if (*odom_peek > scan_time && *odom_peek < next_scan_time) {
          frame->between_next_odoms.push_back(*odom_peek);
        }
      }
      odom_peek = keep_odom_peek;
      // 记录数据
      frames_.push_back(frame);
    }
    RCLCPP_INFO(this->get_logger(), "激光雷达数据为: %d帧", frames_.size());
    // 补偿最后一帧激光雷达的odom数据虽然它可能只有一半
    if (odom_peek < odom_timestamps_.cend()) {
      for (std::vector<int64_t>::const_iterator scan_iterator =
               scan_timestamps_.cend() - 1;
           scan_iterator < scan_timestamps_.cend(); ++scan_iterator) {
        const auto& scan_time = *scan_iterator;
        auto frame = std::make_shared<FrameData>();
        frame->scan = scan_map_[scan_time];
        frame->timestamp = scan_time;
        frame->frame_index = frame_idx++;
        for (; odom_peek < odom_timestamps_.cend(); ++odom_peek) {
          if (*odom_peek <= scan_time) {
            frame->between_next_odoms.push_back(*odom_peek);
          }
          if (*odom_peek > scan_time) {
            frame->between_next_odoms.push_back(*odom_peek);
          }
        }
        frames_.push_back(frame);
        RCLCPP_INFO(this->get_logger(),
                    "补偿激光雷达数据为: %d帧, 使用的里程计数据: %ld帧",
                    frames_.size(), frame->between_next_odoms.size());
      }
    }

    // 激光时间片间队列双拍提取
    frame_idx = 0;
    for (std::vector<int64_t>::const_iterator scan_iterator =
             scan_timestamps_.cbegin();
         scan_iterator < scan_timestamps_.cend() - 1; ++scan_iterator) {
      const auto& next_scan_iterator = scan_iterator + 1;
      const auto& scan_time = *scan_iterator;
      const auto& next_scan_time = *next_scan_iterator;
      auto frame = std::make_shared<FrameData>();
      // Find closest odometry
      auto before_vec = odom_queue_.popBefore(scan_time);
      auto after_vec = odom_queue_.getRange(scan_time, next_scan_time);
      for (const auto& odom : before_vec) {
        frames_[frame_idx]->between_odoms.emplace_back(odom);
      }
      for (const auto& odom : after_vec) {
        frames_[frame_idx]->between_odoms.emplace_back(odom);
      }
      // 打印双拍提取信息
      frame_idx++;
    }
    // 补偿最后一帧激光雷达的数据丢弃，队列中始终有一帧数据
    return !frames_.empty();
  }

  /**
   * @brief 检测一帧激光的反光柱
   */
  void processFrame(size_t frame_index) {
    if (frame_index >= frames_.size()) {
      RCLCPP_WARN(this->get_logger(), "帧索引超出范围: %zu/%zu", frame_index,
                  frames_.size());
      return;
    }
    // 获取base_link->laser
    if (!has_laser_to_base_) {
      std::string source_frame = "laser";
      std::string target_frame = "base_link";
      try {
        int64_t firstscan_timestamp = frames_[frame_index]->timestamp;
        // Get transform from base_link to laser frame
        if (!has_laser_to_base_) {
          laser_to_base_ = tf_buffer_->lookupTransform(
              target_frame, source_frame, rclcpp::Time(firstscan_timestamp),
              std::chrono::milliseconds(100));
          has_laser_to_base_ = true;
          RCLCPP_WARN_STREAM(this->get_logger(),
                      "Finish LaserToBase transform configure!" << transforms::ToRigid3d(laser_to_base_));
        }
      } catch (tf2::TransformException& ex) {
        RCLCPP_FATAL_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "无法获取从%s到%s的变换: %s",
                              source_frame.c_str(), target_frame.c_str(),
                              ex.what());
        has_laser_to_base_ = false;
        return;
      }
    }

    current_frame_index_ = frame_index;
    auto& frame = frames_[frame_index];

    RCLCPP_INFO(this->get_logger(), "处理帧 %zu/%zu, 时间: %.3f", frame_index,
                frames_.size() - 1, rclcpp::Time(frame->timestamp).seconds());
    if(frame->between_odoms.empty()) {
      return;
    }
    // 1. 使用扭曲补偿, 并获取扫描时刻插值轨迹
    auto scan_points = convertScanToTimedPoints(frame->scan);
    /// 矫正点云并计算laser在矫正拟合器中估计的全局位姿
    auto compensated_points = CublicDistortionCorrector::correctDistortion(
        scan_points, frame->between_odoms,
        transforms::ToRigid3d(laser_to_base_), frame->timestamp,
        frame->global_pose);
    if (!use_distort_corrector_) {
      frame->compensated_points = convertScanToPoints(frame->scan);
    } else {
      frame->compensated_points = compensated_points;
    }
    // 进行利用global_poses和odom计算当前laser时刻map->odom的映射，从而计算global_pose
    auto globalposes = globalpose_queue_.popBefore(frame->timestamp);
    auto odompose = frame->between_odoms[frame->between_odoms.size() / 2];
    transforms::Rigid3d global_laser_pose;
    
    if (!map_odom_initialized_) {
      // 第一次计算map_odom_
      if (globalposes.empty()) {
        // popBefore为空，说明第一帧时间早于所有globalpose，使用front()初始化
        if (!globalpose_queue_.empty()) {
          auto globalpose = globalpose_queue_.front();
          global_laser_pose = globalpose.data;
          map_odom_ = global_laser_pose * odompose.data.inverse();
          map_odom_initialized_ = true;
          LOG(INFO) << "首次初始化map_odom_ (使用front): " << map_odom_
                    << " 全局时刻:" << globalpose.timestamp
                    << " 激光时刻:" << frame->timestamp;
        } else {
          LOG(ERROR) << "globalpose_queue_为空，无法初始化map_odom_";
          return;
        }
      } else {
        // popBefore非空，使用back()初始化
        auto globalpose = globalposes.back();
        global_laser_pose = globalpose.data * transforms::ToRigid3d(laser_to_base_);
        map_odom_ = global_laser_pose * odompose.data.inverse();
        map_odom_initialized_ = true;
        LOG(INFO) << "首次初始化map_odom_ (使用popBefore.back): " << map_odom_
                  << " 全局时刻:" << globalpose.timestamp
                  << " 激光时刻:" << frame->timestamp;
      }
    } else {
      // map_odom_已初始化，检查是否需要更新
      if (!globalposes.empty()) {
        // popBefore非空，说明map_odom_在此帧发生了变化，需要更新
        auto globalpose = globalposes.back();
        global_laser_pose = globalpose.data;
        map_odom_ = global_laser_pose * odompose.data.inverse();
        LOG(WARNING) << frame->timestamp
                     << " 时刻map_odom_已更新: " << map_odom_
                     << " 全局位姿:" << globalpose.data
                     << " 全局时刻:" << globalpose.timestamp
                     << " 里程计位姿: " << odompose.data
                     << " 里程计时刻:" << odompose.timestamp;
      } else {
        // popBefore为空，说明map_odom_之间没有变化，不用更新
        global_laser_pose = map_odom_ * odompose.data;
        LOG(INFO) << frame->timestamp
                  << " 时刻map_odom_未变化，保持不变";
      }
    }
    
    // 验证计算结果
    auto odom_pose = frame->between_odoms.back().data;
    auto map_pose = map_odom_ * odom_pose;
    LOG(INFO) << "验证: map_odom_ * odom_pose = " << map_pose;
    frame->global_pose = map_pose * transforms::ToRigid3d(laser_to_base_);
    // auto globalLaserPose = map_odom_ * frame->global_pose;
    // frame->global_pose = globalLaserPose;
    LOG(INFO) << frame->timestamp << " 估计laser全局位姿" << frame->global_pose;
    // 2. Apply intensity filtering
    auto filtered_points =
        filterByIntensity(frame->compensated_points, raw_intensity_threshold_);
    RCLCPP_INFO(this->get_logger(), "原始点数: %zu, 强度过滤后: %zu",
                frame->compensated_points.size(), filtered_points.size());
    // 3. DBSCAN clustering
    frame->filtered_points = filtered_points;
    std::vector<std::vector<Point>> clusters =
        fixed_dbscan_.splitCluster(frame->filtered_points);
    if (clusters.empty()) {
      RCLCPP_WARN(this->get_logger(), "DBSCAN聚类后无簇");
      frame->reflectors.clear();
      return;
    }
    // 4. Detect reflectors
    // INFO: 修复圆拟合检测性问题，连续性插值检测;
    // 局部非凹性检测; 1.2m内有大噪声; 保存： 当前帧pcd点云;
    if (use_short_tracker_) {
      frame->reflectors = reflector_detector_.detectReflectorsWithShortTracking(
          clusters, frame->global_pose, frame->timestamp);
      RCLCPP_INFO(this->get_logger(), "短时跟踪检测到 %zu 个反光柱",
                  frame->reflectors.size());
    } else {
      frame->reflectors = reflector_detector_.detectReflectors(clusters);
      RCLCPP_INFO(this->get_logger(), "检测到 %zu 个反光柱",
                  frame->reflectors.size());
    }
  }

  /**
   * @brief 时间回调定时器进行发布可视化消息，调用visualization_helper
   */
  void publishTimerCallback() {
    if (current_frame_index_ >= frames_.size()) {
      return;
    }
    const auto& frame = frames_[current_frame_index_];
    visualization_helper_->pulishOriginLaserScan(frame->scan);
    // 指定发布frame是以laser为准，还是以矫正后map为准的global_points
    visualization_helper_->publishFilteredPointCloud(frame->compensated_points,
                                                     frame->global_pose);
    // 发布阈值滤波后的点云
    // 指定发布frame是以laser为准，还是以矫正后map为准的global_points
    visualization_helper_->publishClusteredPointCloud(frame->filtered_points,
                                                      frame->global_pose);
    // 新增可视化拟合圆, 默认是以laser为准，可原则是否以map为frame
    visualization_helper_->publishReflectorMarkers(frame->reflectors,
                                                   frame->global_pose);
    // 新增可视化跟踪后的反光柱, 默认是以laser为准，可原则是否以map为frame
    visualization_helper_->publishTrackedReflectorMarkers(
        reflector_detector_.getCurrentAllTrackedReflectors(),
        frame->global_pose);
    visualization_helper_->publishLaserPose(frame->global_pose);
  }

  /**
   * @brief Keyboard input thread
   */
  void keyboardInputThread() {
    RCLCPP_INFO(this->get_logger(), "键盘控制:");
    RCLCPP_INFO(this->get_logger(), "  'n' - 下一帧");
    RCLCPP_INFO(this->get_logger(), "  'p' - 上一帧");
    RCLCPP_INFO(this->get_logger(), "  ' ' (空格) - 切换自动模式");
    RCLCPP_INFO(this->get_logger(), "  'q' - 退出");

    while (!should_exit_) {
      char key = get_char_without_enter();
      RCLCPP_INFO(this->get_logger(), "Key: %c", key);
      switch (key) {
        case 'n':
        case 'N':
          // 只考虑{N-1}帧由于双拍缓存，最帧{0,1,...N-2}序列
          if (current_frame_index_ < frames_.size() - 2) {
            processFrame(current_frame_index_ + 1);
          } else {
            RCLCPP_WARN(this->get_logger(), "已是最后一帧");
          }
          break;

        case 'p':
        case 'P':
          if (current_frame_index_ > 0) {
            processFrame(current_frame_index_ - 1);
          } else {
            RCLCPP_WARN(this->get_logger(), "已是第一帧");
          }
          break;

        case ' ':
          auto_mode_ = !auto_mode_;
          RCLCPP_INFO(this->get_logger(), "自动模式: %s",
                      auto_mode_ ? "开启" : "关闭");
          if (auto_mode_) {
            // startAutoMode();
          }
          break;

        case 'q':
        case 'Q':
          RCLCPP_INFO(this->get_logger(), "安全退出程序");
          should_exit_ = true;
          rclcpp::shutdown();
          break;

        default:
          break;
      }
    }
  }

  /**
   * @brief Start automatic processing mode
   */
  void startAutoMode() {
    std::thread([this]() {
      while (auto_mode_ && !should_exit_) {
        if (current_frame_index_ < frames_.size() - 1) {
          processFrame(current_frame_index_ + 1);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
          RCLCPP_INFO(this->get_logger(), "自动处理完成");
          auto_mode_ = false;
          break;
        }
      }
    }).detach();
  }

  /*
   * @brief 将LaserScan转化成带有时间戳，强度和有向序列的二维点
   */
  static std::vector<TimePoint> convertScanToTimedPoints(
      const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    std::vector<TimePoint> points;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      if (scan_msg->ranges[i] < scan_msg->range_min ||
          scan_msg->ranges[i] > scan_msg->range_max ||
          !std::isfinite(scan_msg->ranges[i])) {
        continue;
      }

      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double range = scan_msg->ranges[i];

      TimePoint point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      int64_t point_stamp = rclcpp::Time(scan_msg->header.stamp).nanoseconds() +
                            int(i * scan_msg->time_increment * 1e9);
      point.timestamp = point_stamp;
      if (i < scan_msg->intensities.size()) {
        point.intensity = scan_msg->intensities[i];
      } else {
        point.intensity = 0.0;
      }

      points.push_back(point);
    }

    return points;
  }

  static std::vector<Point> convertScanToPoints(
      const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    std::vector<Point> points;
    int index = 0;
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      if (scan_msg->ranges[i] < scan_msg->range_min ||
          scan_msg->ranges[i] > scan_msg->range_max ||
          !std::isfinite(scan_msg->ranges[i])) {
        continue;
      }

      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double range = scan_msg->ranges[i];

      Point point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      point.origin_index = index++;
      if (i < scan_msg->intensities.size()) {
        point.intensity = scan_msg->intensities[i];
      } else {
        point.intensity = 0.0;
      }
      points.push_back(point);
    }
    return points;
  }

  /**
   * @brief 强度过滤
   */
  std::vector<Point> filterByIntensity(const std::vector<Point>& points,
                                       double threshold) {
    std::vector<Point> filtered;
    filtered.reserve(points.size());
    for (const auto& point : points) {
      if (point.intensity >= threshold) {
        filtered.push_back(point);
      }
    }
    return filtered;
  }

  /**
   * @brief 聚类中心的距离
   */
  Point computeCentroid(const std::vector<Point>& cluster) {
    Point centroid;
    if (cluster.empty()) {
      return centroid;
    }

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& point : cluster) {
      sum_x += point.x;
      sum_y += point.y;
    }

    centroid.x = sum_x / cluster.size();
    centroid.y = sum_y / cluster.size();
    return centroid;
  }

  void configureDistortionCorrector() {
    this->declare_parameter("distort_corrector.enable", true);
    this->use_distort_corrector_ =
        this->get_parameter("distort_corrector.enable").as_bool();
    this->declare_parameter("distort_corrector.method", "Spline");
    this->distortcorrect_method_ =
        this->get_parameter("distort_corrector.method").as_string();
    if (use_distort_corrector_) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "[✔] 配置点云矫正器: " << distortcorrect_method_);
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "[✘] 没有使用点云矫正器！！！！");
    }
  }

  void configureFixedDBSCAN() {
    FixedDBSCAN::Config dbscan_config;
    this->declare_parameter("fixed_dbscan.eps", 0.05);
    this->declare_parameter("fixed_dbscan.min_points", 5);
    this->declare_parameter("fixed_dbscan.contine_gap", 3);
    this->declare_parameter("fixed_dbscan.continue_points", 8);
    // 提取参数
    dbscan_config.eps = this->get_parameter("fixed_dbscan.eps").as_double();
    dbscan_config.min_points =
        this->get_parameter("fixed_dbscan.min_points").as_int();
    dbscan_config.gap_threshold =
        this->get_parameter("fixed_dbscan.contine_gap").as_int();
    dbscan_config.continue_points =
        this->get_parameter("fixed_dbscan.continue_points").as_int();
    this->fixed_dbscan_ = FixedDBSCAN(dbscan_config);
    // 打印配置信息
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[✔] 配置FixedDBSCAN.eps: " << dbscan_config.eps);
    RCLCPP_INFO_STREAM(this->get_logger(), "[✔] 配置FixedDBSCAN.min_points: "
                                               << dbscan_config.min_points);
    RCLCPP_INFO_STREAM(this->get_logger(), "[✔] 配置FixedDBSCAN.contine_gap: "
                                               << dbscan_config.gap_threshold);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[✔] 配置FixedDBSCAN.continue_points: "
                           << dbscan_config.continue_points);
  }

  void configurePCAClassification() {
    bool use_pca_classification = false;
    this->declare_parameter("pca_classification.enable", true);
    this->declare_parameter("pca_classification.min_points", 13);
    this->declare_parameter("pca_classification.max_elongation_post", 9.5);
    this->declare_parameter("pca_classification.min_elongation_board", 12.0);
    this->declare_parameter("pca_classification.max_linearity_post", 0.97);
    this->declare_parameter("pca_classification.min_linearity_board", 0.93);
    this->declare_parameter("pca_classification.near_distance", 1.3);
    this->declare_parameter("pca_classification.near_min_points", 20);
    this->declare_parameter("pca_classification.near_max_linearity_post", 0.89);
    // Configure
    use_pca_classification =
        this->get_parameter("pca_classification.enable").as_bool();
    ShapeClassificationParams pca_params;
    pca_params.min_points =
        this->get_parameter("pca_classification.min_points").as_int();
    pca_params.max_elongation_post =
        this->get_parameter("pca_classification.max_elongation_post")
            .as_double();
    pca_params.min_elongation_board =
        this->get_parameter("pca_classification.min_elongation_board")
            .as_double();
    pca_params.max_linearity_post =
        this->get_parameter("pca_classification.max_linearity_post")
            .as_double();
    pca_params.min_linearity_board =
        this->get_parameter("pca_classification.min_linearity_board")
            .as_double();
    pca_params.near_distance =
        this->get_parameter("pca_classification.near_distance").as_double();
    pca_params.near_min_points =
        this->get_parameter("pca_classification.near_min_points").as_int();
    pca_params.near_max_linearity_post =
        this->get_parameter("pca_classification.near_max_linearity_post")
            .as_double();
    // 配置
    reflector_detector_.setDetectMethod(classification_method_);
    if (use_pca_classification) {
      reflector_detector_.configPCAShapeClassifier(pca_params);
    }
  }

  void configureCicrleFit() {
    this->declare_parameter("circle_fit.max_fit_error", 0.03);
    this->declare_parameter("circle_fit.min_inlier_ratio", 0.5);
    this->declare_parameter("circle_fit.max_fit_error_near", 0.01);
    this->declare_parameter("circle_fit.max_fit_error_far", 0.02);
    this->declare_parameter("circle_fit.far_distance_threshold", 1.5);
    this->declare_parameter("circle_fit.min_radius", 0.02);
    this->declare_parameter("circle_fit.max_radius", 0.05);
    this->declare_parameter("circle_fit.max_concave_ratio", 0.2);
    this->declare_parameter("circle_fit.ransac_iterations", 100);
    this->declare_parameter("circle_fit.ransac_inlier_threshold", 0.0015);
    this->declare_parameter("circle_fit.ransac_min_points", 13);
    // Configure
    CircleFitParams circle_params;
    circle_params.max_fit_error =
        this->get_parameter("circle_fit.max_fit_error").as_double();
    circle_params.min_inlier_ratio =
        this->get_parameter("circle_fit.min_inlier_ratio").as_double();
    circle_params.max_fit_error_near =
        this->get_parameter("circle_fit.max_fit_error_near").as_double();
    circle_params.max_fit_error_far =
        this->get_parameter("circle_fit.max_fit_error_far").as_double();
    circle_params.far_distance_threshold =
        this->get_parameter("circle_fit.far_distance_threshold").as_double();
    circle_params.min_radius =
        this->get_parameter("circle_fit.min_radius").as_double();
    circle_params.max_radius =
        this->get_parameter("circle_fit.max_radius").as_double();
    circle_params.max_concave_ratio =
        this->get_parameter("circle_fit.max_concave_ratio").as_double();
    circle_params.ransac_iterations =
        this->get_parameter("circle_fit.ransac_iterations").as_int();
    circle_params.ransac_inlier_threshold =
        this->get_parameter("circle_fit.ransac_inlier_threshold").as_double();
    circle_params.ransac_min_points =
        this->get_parameter("circle_fit.ransac_min_points").as_int();
    // 配置
    reflector_detector_.configCircleFitter(circle_params);
  }

  void configureGlobalTracker() {
    this->declare_parameter("global_tracking.enable", true);
    this->declare_parameter("global_tracking.match_distance_threshold", 0.3);
    this->declare_parameter("global_tracking.match_distance_inactive", 0.5);
    this->declare_parameter("global_tracking.confirm_time_window", 1.0);
    this->declare_parameter("global_tracking.min_detections_in_window", 8);
    this->declare_parameter("global_tracking.inactive_timeout", 5.0);
    this->declare_parameter("global_tracking.max_inactive_time", 60.0);
    this->declare_parameter("global_tracking.position_filter_alpha", 0.3);
    this->declare_parameter("global_tracking.position_filter_beta", 0.2);
    this->declare_parameter("global_tracking.min_std_dev", 0.02);
    this->declare_parameter("global_tracking.max_std_dev", 0.5);
    this->declare_parameter("global_tracking.min_confidence_to_track", 0.3);
    this->declare_parameter("global_tracking.confidence_filter_alpha", 0.2);
    this->declare_parameter("global_tracking.diameter_filter_alpha", 0.3);

    // Configure global tracking
    bool use_global_tracker = false;
    use_global_tracker =
        this->get_parameter("global_tracking.enable").as_bool();
    GlobalReflectorTracker::Config tracking_config;
    tracking_config.match_distance_threshold =
        this->get_parameter("global_tracking.match_distance_threshold")
            .as_double();
    tracking_config.match_distance_inactive =
        this->get_parameter("global_tracking.match_distance_inactive")
            .as_double();
    tracking_config.confirm_time_window =
        this->get_parameter("global_tracking.confirm_time_window").as_double();
    tracking_config.min_detections_in_window =
        this->get_parameter("global_tracking.min_detections_in_window")
            .as_int();
    tracking_config.inactive_timeout =
        this->get_parameter("global_tracking.inactive_timeout").as_double();
    tracking_config.max_inactive_time =
        this->get_parameter("global_tracking.max_inactive_time").as_double();
    tracking_config.position_filter_alpha =
        this->get_parameter("global_tracking.position_filter_alpha")
            .as_double();
    tracking_config.position_filter_beta =
        this->get_parameter("global_tracking.position_filter_beta").as_double();
    tracking_config.min_std_dev =
        this->get_parameter("global_tracking.min_std_dev").as_double();
    tracking_config.max_std_dev =
        this->get_parameter("global_tracking.max_std_dev").as_double();
    tracking_config.min_confidence_to_track =
        this->get_parameter("global_tracking.min_confidence_to_track")
            .as_double();
    tracking_config.confidence_filter_alpha =
        this->get_parameter("global_tracking.confidence_filter_alpha")
            .as_double();
    tracking_config.diameter_filter_alpha =
        this->get_parameter("global_tracking.diameter_filter_alpha")
            .as_double();
    if (use_global_tracker) {
      // Set the config to Detector
      RCLCPP_INFO(
          this->get_logger(),
          "#===反光柱检测器==== 开启全局反光柱跟踪器短时跟踪功能!!! ###");
      reflector_detector_.configGlobalReflectorTracker(tracking_config);
    }
    use_short_tracker_ = use_global_tracker;
  }

  //---------------------- Parameters -------------------
  std::string bag_path_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string classification_method_;
  double raw_intensity_threshold_;  // 重要

  // 激光与里程计相关数据
  std::unordered_map<int64_t, sensor_msgs::msg::LaserScan::SharedPtr>
      scan_map_;                          // Map of scan messages by timestamp
  std::vector<int64_t> odom_timestamps_;  // Vector of frames
  TimeOrderQueue<transforms::Rigid3d>
      odom_queue_;  // 可用于在线里程计的队列管理
  TimeOrderQueue<transforms::Rigid3d> globalpose_queue_;
  std::vector<int64_t> scan_timestamps_;            // 离线扫描时间
  std::vector<std::shared_ptr<FrameData>> frames_;  // 实际可用帧
  // 用于估计反光柱的激光帧，离线播放时可用帧调整;
  size_t current_frame_index_;
  // 激光雷达相对于基座的位姿变换
  geometry_msgs::msg::TransformStamped laser_to_base_;
  bool has_laser_to_base_{false};

  // 点云矫正器
  bool use_distort_corrector_;
  std::string distortcorrect_method_;
  // 聚类器
  FixedDBSCAN fixed_dbscan_;
  double min_confidence_;
  // Detection modules
  bool use_short_tracker_;
  RefelctorDetector reflector_detector_;

  // 可视化 Publishers
  std::shared_ptr<VisualizationHelper> visualization_helper_;

  // Timer for continuous publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Timer for republishing clock during final optimization
  rclcpp::TimerBase::SharedPtr clock_republish_timer_;

  // TF buffer and clock publisher for offline node
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rosgraph_msgs::msg::Clock clock_msg_;
  std::unique_ptr<cartographer_ros::Node> cartographer_node_;

  // Threading
  std::atomic<bool> auto_mode_;
  std::atomic<bool> should_exit_;
  transforms::Rigid3d map_odom_;
  bool map_odom_initialized_{false};  // 标记map_odom_是否已初始化
};

}  // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto node = std::make_shared<cartographer_ros::ReflectorNoiseBagNode>();
  RCLCPP_INFO(node->get_logger(), "启动反光柱逐帧检测 (Bag处理版本)");
  node->run();

  rclcpp::shutdown();

  return 0;
}