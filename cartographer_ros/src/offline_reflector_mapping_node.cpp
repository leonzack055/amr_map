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

#include "cartographer_ros/node.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/playable_bag.h"
#include "cartographer_ros/urdf_reader.h"
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

void RunOfflineNode(const MapBuilderFactory& map_builder_factory,
                    rclcpp::Node::SharedPtr cartographer_offline_node) {
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  LOG(WARNING) << "FLAGS_configuration_directory "
               << FLAGS_configuration_directory;
  CHECK(!FLAGS_configuration_basenames.empty())
      << "-configuration_basenames is missing.";
  LOG(WARNING) << "FLAGS_configuration_basenames "
               << FLAGS_configuration_basenames;
  CHECK(!(FLAGS_bag_filenames.empty() && FLAGS_load_state_filename.empty()))
      << "-bag_filenames and -load_state_filename cannot both be unspecified.";
  std::regex regex(",");
  std::vector<std::string> bag_filenames;
  if (!FLAGS_bag_filenames.empty()) {
    std::regex regex(",");
    std::vector<std::string> if_bag_filenames(
        std::sregex_token_iterator(FLAGS_bag_filenames.begin(),
                                   FLAGS_bag_filenames.end(), regex, -1),
        std::sregex_token_iterator());
    bag_filenames = if_bag_filenames;
  }
  cartographer_ros::NodeOptions node_options;
  std::vector<std::string> configuration_basenames(
      std::sregex_token_iterator(FLAGS_configuration_basenames.begin(),
                                 FLAGS_configuration_basenames.end(), regex,
                                 -1),
      std::sregex_token_iterator());

  std::vector<TrajectoryOptions> bag_trajectory_options(1);
  std::tie(node_options, bag_trajectory_options.at(0)) =
      LoadOptions(FLAGS_configuration_directory, configuration_basenames.at(0));

  for (size_t bag_index = 1; bag_index < bag_filenames.size(); ++bag_index) {
    TrajectoryOptions current_trajectory_options;
    if (bag_index < configuration_basenames.size()) {
      std::tie(std::ignore, current_trajectory_options) = LoadOptions(
          FLAGS_configuration_directory, configuration_basenames.at(bag_index));
    } else {
      current_trajectory_options = bag_trajectory_options.at(0);
    }
    bag_trajectory_options.push_back(current_trajectory_options);
  }
  if (bag_filenames.size() > 0) {
    CHECK_EQ(bag_trajectory_options.size(), bag_filenames.size());
  }

  // Since we preload the transform buffer, we should never have to wait for a
  // transform. When we finish processing the bag, we will simply drop any
  // remaining sensor data that cannot be transformed due to missing transforms.
  node_options.lookup_transform_timeout_sec = 0.;

  auto map_builder = map_builder_factory(node_options.map_builder_options);

  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(cartographer_offline_node->get_clock(),
                                        tf2::durationFromSec(10),
                                        cartographer_offline_node);

  // 从urdf文件中读取 对应的static_transforms变化
  std::vector<geometry_msgs::msg::TransformStamped> urdf_transforms;

  if (!FLAGS_urdf_filenames.empty()) {
    std::vector<std::string> urdf_filenames(
        std::sregex_token_iterator(FLAGS_urdf_filenames.begin(),
                                   FLAGS_urdf_filenames.end(), regex, -1),
        std::sregex_token_iterator());
    for (const auto& urdf_filename : urdf_filenames) {
      const auto current_urdf_transforms =
          ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer);
      urdf_transforms.insert(urdf_transforms.end(),
                             current_urdf_transforms.begin(),
                             current_urdf_transforms.end());
    }
  }
  // 开启 dedicated thread 用于 tf
  // 变换，指为TF变换处理单独分配一个后台线程，与主线程分离运行
  tf_buffer->setUsingDedicatedThread(true);

  // 创建cartographer节点，进行建图工作
  Node node(node_options, std::move(map_builder), tf_buffer,
            cartographer_offline_node, FLAGS_collect_metrics);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher =
      cartographer_offline_node->create_publisher<tf2_msgs::msg::TFMessage>(
          kTfTopic, kLatestOnlyPublisherQueueSize);

  ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(
      cartographer_offline_node);

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher =
      cartographer_offline_node->create_publisher<rosgraph_msgs::msg::Clock>(
          kClockTopic, kLatestOnlyPublisherQueueSize);
  // 发布静态tf，从urdf文件中读取
  if (urdf_transforms.size() > 0) {
    static_tf_broadcaster.sendTransform(urdf_transforms);
  }

  rosgraph_msgs::msg::Clock clock;

  std::vector<
      std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
      bag_expected_sensor_ids;
  if (configuration_basenames.size() == 1) {
    const auto current_bag_expected_sensor_ids =
        node.ComputeDefaultSensorIdsForMultipleBags(
            {bag_trajectory_options.front()});
    bag_expected_sensor_ids = {bag_filenames.size(),
                               current_bag_expected_sensor_ids.front()};
  } else {
    bag_expected_sensor_ids =
        node.ComputeDefaultSensorIdsForMultipleBags(bag_trajectory_options);
  }
  CHECK_EQ(bag_expected_sensor_ids.size(), bag_filenames.size());

  std::map<std::pair<int /* bag_index */, std::string>,
           cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      bag_topic_to_sensor_id;
  PlayableBagMultiplexer playable_bag_multiplexer(cartographer_offline_node);
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
                   << bag_resolved_topic.second << " which is already used by "
                   << " sensor "
                   << bag_topic_to_sensor_id.at(bag_resolved_topic).id;
      }
      bag_topic_to_sensor_id[bag_resolved_topic] = expected_sensor_id;
    }

    auto serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
    playable_bag_multiplexer.AddPlayableBag(PlayableBag(
        bag_filename, current_bag_index, kDelay,
        // PlayableBag::FilteringEarlyMessageHandler is used to get an early
        // peek at the tf messages in the bag and insert them into 'tf_buffer'.
        // When a message is retrieved by GetNextMessage() further below,
        // we will have already inserted further 'kDelay' seconds worth of
        // transforms into 'tf_buffer' via this lambda.
        [&tf_publisher, tf_buffer, cartographer_offline_node, serializer](
            std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {
          // TODO: filter bag msg per type ? Planned rosbag2 evolution ?
          if (msg->topic_name == kTfTopic ||
              msg->topic_name == kTfStaticTopic) {
            if (FLAGS_use_bag_transforms) {
              tf2_msgs::msg::TFMessage tf_message;
              rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
              try {
                serializer.deserialize_message(&serialized_msg, &tf_message);
                for (auto& transform : tf_message.transforms) {
                  try {
                    // We need to keep 'tf_buffer' small because it becomes very
                    // inefficient otherwise. We make sure that tf_messages are
                    // published before any data messages, so that tf lookups
                    // always work.
                    tf_buffer->setTransform(transform, "unused_authority",
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
            // Tell 'PlayableBag' to filter the tf message since there is no
            // further use for it.
            return false;
          } else {
            return true;
          }
        }));
  }

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

  std::unordered_map<int, int> bag_index_to_trajectory_id;
  const rclcpp::Time begin_time =
      // If no bags were loaded, we cannot peek the time of first message.
      playable_bag_multiplexer.IsMessageAvailable()
          ? playable_bag_multiplexer.PeekMessageTime()
          : rclcpp::Time();

  auto laser_scan_serializer =
      rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
  auto multi_echo_laser_scan_serializer =
      rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
  auto pcl2_serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
  auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
  auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
  auto nav_sat_fix_serializer =
      rclcpp::Serialization<sensor_msgs::msg::NavSatFix>();
  auto landmark_list_serializer =
      rclcpp::Serialization<cartographer_ros_msgs::msg::LandmarkList>();

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

    int trajectory_id;
    // Lazily add trajectories only when the first message arrives in order
    // to avoid blocking the sensor queue.
    if (bag_index_to_trajectory_id.count(bag_index) == 0) {
      trajectory_id =
          node.AddOfflineTrajectory(bag_expected_sensor_ids.at(bag_index),
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

    const auto bag_topic = std::make_pair(
        bag_index, cartographer_offline_node->get_node_base_interface()
                       ->resolve_topic_or_service_name(msg.topic_name, false));
    auto it = bag_topic_to_sensor_id.find(bag_topic);

    if (it != bag_topic_to_sensor_id.end()) {
      const std::string& sensor_id = it->second.id;

      if (topic_type == "sensor_msgs/msg/LaserScan") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg =
            std::make_shared<sensor_msgs::msg::LaserScan>();
        laser_scan_serializer.deserialize_message(&serialized_msg,
                                                  laser_scan_msg.get());
        node.HandleLaserScanMessage(trajectory_id, sensor_id, laser_scan_msg);
        // TODO: landmark检测器，进和检测；
        // landmark检测器，进行landmark的参数的获取
      } else if (topic_type == "sensor_msgs/msg/MultiEchoLaserScan") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::MultiEchoLaserScan::SharedPtr
            multi_echo_laser_scan_msg =
                std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
        multi_echo_laser_scan_serializer.deserialize_message(
            &serialized_msg, multi_echo_laser_scan_msg.get());
        node.HandleMultiEchoLaserScanMessage(trajectory_id, sensor_id,
                                             multi_echo_laser_scan_msg);
      } else if (topic_type == "sensor_msgs/msg/PointCloud2") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::PointCloud2::SharedPtr pcl2_scan_msg =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl2_serializer.deserialize_message(&serialized_msg,
                                            pcl2_scan_msg.get());
        node.HandlePointCloud2Message(trajectory_id, sensor_id, pcl2_scan_msg);
      } else if (topic_type == "sensor_msgs/msg/Imu") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::Imu::SharedPtr imu_scan_msg =
            std::make_shared<sensor_msgs::msg::Imu>();
        imu_serializer.deserialize_message(&serialized_msg, imu_scan_msg.get());
        node.HandleImuMessage(trajectory_id, sensor_id, imu_scan_msg);
      } else if (topic_type == "nav_msgs/msg/Odometry") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        nav_msgs::msg::Odometry::SharedPtr odom_scan_msg =
            std::make_shared<nav_msgs::msg::Odometry>();
        odom_serializer.deserialize_message(&serialized_msg,
                                            odom_scan_msg.get());
        node.HandleOdometryMessage(trajectory_id, sensor_id, odom_scan_msg);
      } else if (topic_type == "sensor_msgs/msg/NavSatFix") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_msg =
            std::make_shared<sensor_msgs::msg::NavSatFix>();
        nav_sat_fix_serializer.deserialize_message(&serialized_msg,
                                                   nav_sat_fix_msg.get());
        node.HandleNavSatFixMessage(trajectory_id, sensor_id, nav_sat_fix_msg);
      } else if (topic_type == "cartographer_ros_msgs/msg/LandmarkList") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        cartographer_ros_msgs::msg::LandmarkList::SharedPtr landmark_list_msg =
            std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
        landmark_list_serializer.deserialize_message(&serialized_msg,
                                                     landmark_list_msg.get());
        node.HandleLandmarkMessage(trajectory_id, sensor_id, landmark_list_msg);
      }
    }
#ifdef PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME
    clock.clock = rclcpp::Time(msg.time_stamp);
#else
    clock.clock = rclcpp::Time(msg.recv_timestamp);
#endif
    clock_publisher->publish(clock);
    rclcpp::spin_some(cartographer_offline_node);

    if (is_last_message_in_bag) {
      node.FinishTrajectory(trajectory_id);
    }
  }

  // Ensure the clock is republished after the bag has been finished, during the
  // final optimization, serialization, and optional indefinite spinning at the
  // end.
  // TODO: need a spin for the timer to tick
  auto clock_republish_timer = cartographer_offline_node->create_wall_timer(
      std::chrono::milliseconds(int(kClockPublishFrequencySec)),
      [&clock_publisher, &clock]() { clock_publisher->publish(clock); });
  node.RunFinalOptimization();

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

  // Serialize unless we have neither a bagfile nor an explicit state filename.
  if (rclcpp::ok() &&
      !(bag_filenames.empty() && FLAGS_save_state_filename.empty())) {
    const std::string state_output_filename =
        FLAGS_save_state_filename.empty() ? bag_filenames.front() + ".pbstream"
                                          : FLAGS_save_state_filename;
    LOG(INFO) << "Writing state to '" << state_output_filename << "'...";
    node.SerializeState(state_output_filename,
                        true /* include_unfinished_submaps */);
  }
  if (FLAGS_keep_running) {
    LOG(INFO) << "Finished processing and waiting for shutdown.";
    rclcpp::spin(cartographer_offline_node);
  }
}
}  // namespace cartographer_ros

using namespace amr_reflector_noise_handling;
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


/**
 * @brief Frame data structure for storing scan and odometry information
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

/**
 * @brief Global pose tracking using odometry
 */
class PoseTracker {
 public:
  PoseTracker() : initialized_(false) {}

  /**
   * @brief Update global pose using odometry
   */
  void update(
      std::shared_ptr<FrameData>& laser_frame,
      const std::unordered_map<int64_t, nav_msgs::msg::Odometry::SharedPtr>&
          odom_queue,
      const transforms::Rigid3d& laser_to_base) {
    // TODO: 使用OdometryQueue进行LaserScan数据的CSplines拟合
    // 1. 获取laser帧前后两个里程数据
    std::vector<PosePoint> odom_poses(laser_frame->between_next_odoms.size());
    for (const auto odom_stamp : laser_frame->between_next_odoms) {
      PosePoint tmp_pose;
      tmp_pose.pose =
          transforms::ToRigid3d(odom_queue.at(odom_stamp)->pose.pose);
      tmp_pose.timestamp = odom_stamp * 1e-9;
      odom_poses.emplace_back(tmp_pose);
    }
    // 3. 对于laser帧前后两个里程数据进行插值
    PoseCubicSpline odom_spline(odom_poses);
    // 4. 计算当前扫描点的里程计位姿
    auto global_base_pose =
        odom_spline.interpolate(laser_frame->timestamp * 1e-9);
    // 激光雷达在里程计下的全局坐标位姿
    global_pose_ = global_base_pose.pose * laser_to_base;
    trajectory_.push_back(global_pose_);
    // 2.
    // 使用CSpline进行插值求取laser帧各个扫描点的全局位姿；构建filtered点云
    if (!initialized_) {
      // Initialize with first odometry
      initialized_ = true;
      return;
    }
  }

  /**
   * @brief Get current global pose
   */
  transforms::Rigid3d getGlobalPose() const { return global_pose_; }

  const std::vector<transforms::Rigid3d>& getTrajectory() const {
    return trajectory_;
  }
  /**
   * @brief Reset tracker
   */
  void reset() { initialized_ = false; }

 private:
  bool initialized_;
  transforms::Rigid3d global_pose_;
  std::vector<transforms::Rigid3d> trajectory_;
};

/**
 * @brief Interactive bag processing node
 */
class ReflectorNoiseBagNode : public rclcpp::Node {
 public:
  ReflectorNoiseBagNode()
      : Node("reflector_noise_bag_node"),
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
    // 创建反光柱可视化器
    visualization_helper_ =
        std::make_shared<VisualizationHelper>(shared_from_this());
    // Create timer for continuous publishing (10 Hz)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ReflectorNoiseBagNode::publishTimerCallback, this));

    if (bag_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Bag路径未设置，请使用--ros-args -p bag_path:=<path>");
      return;
    }

    // Open bag file
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions converter_options;
    reader.open(storage_options, converter_options);
    RCLCPP_INFO(this->get_logger(), "打开Bag文件: %s", bag_path_.c_str());
    for (auto& topic : reader.get_all_topics_and_types()) {
      RCLCPP_INFO(this->get_logger(), "Topic: %s, Type: %s", topic.name.c_str(),
                  topic.type.c_str());
    }

    // Pre-load all frames
    if (!loadAllFrames(reader)) {
      RCLCPP_ERROR(this->get_logger(), "加载帧失败");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "加载完成，共 %zu 帧", frames_.size());

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

 private:
  // 配置算法相关参数，从ros2的参数服务器中提取，只提取一次
  void configureDetectionModules() {
    // Configure PCA classifier
    configurePCAClassification();
    // Configure circle fittercircle_fitter_
    configureCicrleFit();
    // Configure GlobalTracker
    configureGlobalTracker();
  }

  /**
   * @brief Load all frames from bag
   * WARN: 由于里程计前后时间跳变, 这里选用录包时刻的系统时间戳为Laser时间
   */
  bool loadAllFrames(rosbag2_cpp::Reader& reader) {
    int64_t peek_time = 0;  // Peek time for next message

    auto laser_scan_serializer =
        rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
    // auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
    auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
    auto tf_serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
    int64_t last_odom_stamp = 0;
    int64_t last_scan_stamp = 0;

    // Read all messages
    RCLCPP_INFO(this->get_logger(), "开始读取rosbag包: %s ..... ",
                bag_path_.c_str());
    while (reader.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

      // Deserialize message
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (msg->time_stamp > peek_time) {
        peek_time = msg->time_stamp;
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "录制的rosbag包出现前后事件跳变: 消息 %s, %ld",
                     msg->topic_name.c_str(), msg->time_stamp);
      }
      // Process based on topic
      if (msg->topic_name == scan_topic_) {
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
        laser_scan_serializer.deserialize_message(&serialized_msg, scan.get());
        scan->header.stamp = rclcpp::Time(msg->time_stamp);
        if (scan_map_.find(rclcpp::Time(msg->time_stamp).nanoseconds()) ==
            scan_map_.end()) {
          scan_map_[rclcpp::Time(msg->time_stamp).nanoseconds()] = scan;
          scan_timestamps_.push_back(
              rclcpp::Time(msg->time_stamp).nanoseconds());
          assert(last_scan_stamp < rclcpp::Time(msg->time_stamp).nanoseconds());
          last_scan_stamp = rclcpp::Time(msg->time_stamp).nanoseconds();
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "读取scan消息: "
                                  << " bag timestamp: " << msg->time_stamp
                                  << "存在重复时间戳");
        }
      } else if (msg->topic_name == odom_topic_) {
        auto odom = std::make_shared<nav_msgs::msg::Odometry>();
        odom_serializer.deserialize_message(&serialized_msg, odom.get());
        odom->header.stamp = rclcpp::Time(msg->time_stamp);
        if (odom_map_.find(rclcpp::Time(odom->header.stamp).nanoseconds()) ==
            odom_map_.end()) {
          odom_map_[rclcpp::Time(odom->header.stamp).nanoseconds()] = odom;
          odom_timestamps_.push_back(
              rclcpp::Time(odom->header.stamp).nanoseconds());
          assert(last_odom_stamp <
                 rclcpp::Time(odom->header.stamp).nanoseconds());
          last_odom_stamp = rclcpp::Time(odom->header.stamp).nanoseconds();
          odom_queue_.push(TimeRigid3d(transforms::ToRigid3d(odom->pose.pose),
                                       last_odom_stamp));
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "读取odom消息: "
                                  << " bag timestamp: " << msg->time_stamp
                                  << "存在重复时间戳");
        }
      } else if (msg->topic_name == "/tf_static") {
        // Read tf_static for laser_scan to base_link transform
        auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
        try {
          tf_serializer.deserialize_message(&serialized_msg, tf_msg.get());
          for (auto& transform : tf_msg->transforms) {
            if (transform.header.frame_id == "base_link" &&
                transform.child_frame_id == "laser") {
              laser_to_base_ = transform;
              RCLCPP_INFO(this->get_logger(),
                          "找到laser到base_link的变换: (%.3f, %.3f)",
                          transform.transform.translation.x,
                          transform.transform.translation.y);
            }
          }
        } catch (const rclcpp::exceptions::RCLError& rcl_error) {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "解析TF_STATIC发生错误" << rcl_error.what());
        }
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "完成读取rosbag包:  ..... " << bag_path_);
    // 展示整体队列和信息内容:
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "激光雷达队列信息: " << scan_map_.size()
                                            << "帧, 起始范围: ["
                                            << scan_timestamps_.front() << " , "
                                            << scan_timestamps_.back() << "]");
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "里程计队列信息: " << odom_map_.size()
                                          << "帧, 起始范围: ["
                                          << scan_timestamps_.front() << " , "
                                          << scan_timestamps_.back() << "]");
    // 为激光数据进行里程计计算
    // 1. C-Splines拟合算法
    // TODO: 此处应该直接使用
    // scan的时间戳来进行{duration}时间范围内查找，可用里程计；
    // 这里使用3次样条插值进行拟合
    // 估计出laserscan当前时刻下以odom为关联轴的位姿拟合结果，并附带关联odom的起始数据，以及C-BSpline的拟合函数；
    // ---odom3--odom4--odom5--|--laser0-- | --odom6--odom7--odom9-- |
    // --laser1-- | --odom10--odom11--odom12-- |
    // --laser2 -- ... laser0: [odom3, odom9]
    // 进行数据关联，并利用此范围内数据进行拟合;
    // 当接收到laser1时，所以laser0为数据处理起始位置 laser1： [odom4, odom12]
    // 进行数据关联，并利用此范围内数据进行拟合;
    // 当接收到laser2时，永远以2帧为1拍进行数据关联
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
      // RCLCPP_INFO(this->get_logger(),
      //             "[-]激光雷达第 %d帧, 使用的里程计数据: %ld帧, 双拍提取数据:
      //             "
      //             "%ld帧, before_vec: %ld, after_vec: %ld",
      //             frame_idx, frames_[frame_idx]->between_next_odoms.size(),
      //             frames_[frame_idx]->between_odoms.size(),
      //             before_vec.size(), after_vec.size());
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

    current_frame_index_ = frame_index;
    auto& frame = frames_[frame_index];

    RCLCPP_INFO(this->get_logger(), "处理帧 %zu/%zu, 时间: %.3f", frame_index,
                frames_.size() - 1, rclcpp::Time(frame->timestamp).seconds());
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
   * @brief Filter points by intensity
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
   * @brief Compute centroid of cluster
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
      scan_map_;  // Map of scan messages by timestamp
  std::unordered_map<int64_t, nav_msgs::msg::Odometry::SharedPtr> odom_map_;
  std::vector<int64_t> odom_timestamps_;  // Vector of frames
  TimeOrderQueue<transforms::Rigid3d>
      odom_queue_;  // 可用于在线里程计的队列管理
  std::vector<int64_t> scan_timestamps_;            // 离线扫描时间
  std::vector<std::shared_ptr<FrameData>> frames_;  // 实际可用帧
  // 用于估计反光柱的激光帧，离线播放时可用帧调整;
  size_t current_frame_index_;
  // 激光雷达相对于基座的位姿变换
  geometry_msgs::msg::TransformStamped laser_to_base_;

  // 点云矫正器
  bool use_distort_corrector_;
  std::string distortcorrect_method_;
  // 聚类器
  FixedDBSCAN fixed_dbscan_;
  double min_confidence_;
  // Pose tracking
  PoseTracker pose_tracker_;
  // Detection modules
  bool use_short_tracker_;
  RefelctorDetector reflector_detector_;

  // 可视化 Publishers
  std::shared_ptr<VisualizationHelper> visualization_helper_;

  // Timer for continuous publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Threading
  std::atomic<bool> auto_mode_;
  std::atomic<bool> should_exit_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReflectorNoiseBagNode>();

  RCLCPP_INFO(node->get_logger(), "启动反光柱逐帧检测 (Bag处理版本)");

  node->run();

  rclcpp::shutdown();

  return 0;
}