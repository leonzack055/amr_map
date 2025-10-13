/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer_ros/lifecycle_offline_node.h"
#include "rclcpp_lifecycle/transition.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"  // 核心头文件
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "cartographer_ros_msgs/msg/bagfile_progress.hpp"
#include "byd_mapbuilder_msgs/msg/map_build_process.hpp"

#include <errno.h>
#include <string>
#ifndef WIN32
#include <sys/resource.h>
#endif
#include <time.h>

#include <chrono>
#include <future>
#include <atomic>

#include "cartographer_ros/node.h"
#include "cartographer_ros/playable_bag.h"
#include "cartographer_ros/urdf_reader.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/msg_conversion.h"
#include "gflags/gflags.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "landmark_localization/common/msg_conversion.hpp"
#include "landmark_localization/reflective_post_detector.hpp"
#include "landmark_localization/landmark_assigner.hpp"
#ifdef USE_URDF_H_FILES
#include "urdf/model.h"
#else
#include "urdf/model.hpp"
#endif
#include "rclcpp/exceptions.hpp"
#include <boost/filesystem.hpp>
#include <regex>
#include <string>

DEFINE_bool(
  collect_metrics, false,
  "Activates the collection of runtime metrics. If activated, the "
  "metrics can be accessed via a ROS service.");
DEFINE_bool(
  use_bag_transforms, true,
  "Whether to read, use and republish transforms from bags.");
DEFINE_bool(
  keep_running, true,
  "Keep running the offline node after all messages from the bag "
  "have been processed.");
DEFINE_double(
  skip_seconds, 0,
  "Optional amount of seconds to skip from the beginning "
  "(i.e. when the earliest bag starts.). ");

namespace cartographer_ros
{

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "/tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;
// 发布tf消息比其他消息提前1秒。假设tf的频率更高，这应该确保tf可以始终插值。
const rclcpp::Duration kDelay(1.0, 0);

LifecycleOfflineCartoNode::LifecycleOfflineCartoNode(
  const std::string & node_name, bool intra_process_comms,
  rclcpp::Executor::SharedPtr executor, const MapBuilderFactory & map_builder_factory)
: rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  executor_ = executor;
  resolution_ = 0.05f;
  map_builder_factory_ = map_builder_factory;
  // 构造函数中自定义创建
  map_build_srv_ = this->create_service<byd_mapbuilder_msgs::srv::MapBuild>(
    "build_map_service",
    std::bind(
      &LifecycleOfflineCartoNode::map_build_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  map_build_status_ = MapBuildStatus::STATUS_IDEL;
  enable_mapping_ = true;
  // 创建process监听
  process_pub_ = this->create_publisher<byd_mapbuilder_msgs::msg::MapBuildProcess>(
    "/mapping_process", 10);
  bag_process_sub_ = this->create_subscription<cartographer_ros_msgs::msg::BagfileProgress>(
    "/bagfile_progress", 10,
    std::bind(&LifecycleOfflineCartoNode::bagProcessDataCallback, this, std::placeholders::_1));

  // 扩展：获取包的共享资源路径（如share目录）
  std::string package_name = "cartographer_ros";
  std::string use_urdf_file = "/urdf/byd_amr.urdf";
  std::string package_prefix = ament_index_cpp::get_package_prefix(package_name);
  std::string package_shared = ament_index_cpp::get_package_share_directory(package_name);
  cartographer_shared_dir_ = package_shared;
  cartographer_install_dir_ = package_prefix;
  urdf_path_ = cartographer_shared_dir_ + use_urdf_file;
  default_configuration_basename_ = "offline_bdy_amr2.lua";
  RCLCPP_INFO(
    this->get_logger(), "%s 包的安装路径: %s share路径； %s", package_name.c_str(), package_prefix.c_str(),
    package_shared.c_str());
  // 指定默认权重
  landmark_translation_weight_ = 1e5;
  landmark_rotation_weight_ = 1e1;
}

void LifecycleOfflineCartoNode::bagProcessDataCallback(
  const cartographer_ros_msgs::msg::BagfileProgress::SharedPtr msg)
{
  float total_seconds = msg->total_seconds;
  float processed_seconds = msg->processed_seconds;
  float processed_percentage = processed_seconds / total_seconds;
  auto bag_msg = std::make_unique<byd_mapbuilder_msgs::msg::MapBuildProcess>();
  bag_msg->progress = processed_percentage;
  // Header 使用系统时间，使用clock时间。
  auto now = std::chrono::system_clock::now();
  auto now_epoch = now.time_since_epoch();
  // 转换为秒和纳秒
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(now_epoch).count();
  auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now_epoch).count() % 1000000000;
  // 构造ROS Time消息
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(sec);
  ros_time.nanosec = static_cast<uint32_t>(nsec);
  std_msgs::msg::Header header;
  header.stamp = ros_time;  // 使用系统真实时间
  header.frame_id = "base_link";    // 坐标系（根据实际场景设置）
  bag_msg->header = header;
  process_pub_->publish(std::move(bag_msg));
}

// 服务回调函数
void LifecycleOfflineCartoNode::map_build_callback(
  const std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Request> request,
  std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Response> response)
{
  switch (request->req.cmd_id) {
    case byd_mapbuilder_msgs::msg::MapBuildRequest::CMD_IDEL:
      /* 查询处理，获取当前建图状态；
       1. 如果当前状态为unconfigured, 并且默认，小车没有配置，说明是第一次建图，需要配置 STATUS_NONE
       2. 如果是activate状态，则表示在建图中, STATUS_BUILDING
       3. 如果是inactivate状态说明建图完成，或者任务取消 , STATUS_COMPLETE or STATUS_CANCLE
      */
      {
        if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          if (map_build_status_ == MapBuildStatus::STATUS_IDEL) {
            response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_NONE;
            response->msg = "当前建图节点没有建图任务，处于空闲状态";
          } else {
            LOG(WARNING) << "map_build_status_: " << map_build_status_ << "当前建图节点内部状态";
            response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
            response->msg = "当前节点处于未知状态，请检查相关代码";
          }
        } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_BUILDING;
          response->msg = "当前建图节点正在进行建图工作";
          response->status.progress = 0.5;
        } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_BUILDING;
          response->msg = "当前建图节点正在进行地图保存和资源清理中.....";
        }
      }
      break;
    case byd_mapbuilder_msgs::msg::MapBuildRequest::CMD_START:
      /* 开始建图
       1. 检查是否传入bagfile和filename， 如果不存在返回错误
       2. 检查是否处于unconfigured状态，如果不是，返回错误，并告知当前机器人的状态
       */
      {
        if (request->bagfile.empty() || request->filename.empty()) {
          response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
          response->msg = "请求参数错误，bagfile和filename不能为空";
        } else {
          std::string bagfile = request->bagfile;
          std::string output_pbstream_path = request->filename;
          bag_file_path_ = cartographer_install_dir_ + "/" + bagfile;
          output_pbstream_path_ = cartographer_install_dir_ + "/" + output_pbstream_path;
          map_filestem_ = output_pbstream_path;
          const std::string kPbstreamSuffix = ".pbstream";
          if (output_pbstream_path.length() >= kPbstreamSuffix.length() &&
            output_pbstream_path.substr(output_pbstream_path.length() - kPbstreamSuffix.length()) ==
            kPbstreamSuffix)
          {
            map_filestem_ = output_pbstream_path.substr(
              0,
              output_pbstream_path.length() - kPbstreamSuffix.length());
          }
          RCLCPP_INFO(get_logger(), "rosbag包名称：%s", bag_file_path_.c_str());
          RCLCPP_INFO(get_logger(), "保存地图pbstream名称：%s", output_pbstream_path_.c_str());
          RCLCPP_INFO(get_logger(), "保存地图Smap名称：%s", map_filestem_.c_str());
          if (!boost::filesystem::exists(bag_file_path_)) {
            LOG(ERROR) << "指定的rosbag包路径不存在: " << bag_file_path_;
            response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
            response->msg = "建图失败！ 指定的rosbag包路径不存在!!";
            break;
          }
          if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
            map_build_status_ == MapBuildStatus::STATUS_IDEL)
          {
            RCLCPP_INFO(get_logger(), "当前节点状态为unconfigured，准备切换到inactivate状态");
            trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            // 创建future变量，超时等待，异步等待
            std::atomic<bool> stop_flag(false);
            std::future<void> activate_future = std::async(
              std::launch::async, [&] {
                while (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                  if (get_current_state().id() ==
                  lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING)
                  {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    continue;
                  }
                  RCLCPP_INFO(get_logger(), "当前节点状态为configuring，准备切换到activate状态");
                  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                  if (stop_flag.load(std::memory_order_relaxed)) {
                    RCLCPP_INFO(get_logger(), "终止ACTIVIATE状态切换");
                    return;
                  }
                }
              });
            // 等待任务执行，设置超时时间为500ms
            auto status = activate_future.wait_for(std::chrono::seconds(5));
            if (status == std::future_status::timeout) {
              // 超时：设置停止标志，让任务主动退出
              RCLCPP_INFO(get_logger(), "等待超时5s，尝试终止任务");
              stop_flag.store(true, std::memory_order_relaxed);
              // 等待任务真正退出（可选，确保资源释放）
              activate_future.wait(); // 此时任务应已检测到标志并退出
              response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
              response->msg = "建图请求成功失败，请检查代码......";
            } else if (status == std::future_status::ready) {
              // 任务在超时前已完成
              RCLCPP_INFO(get_logger(), "任务在超时前完成, ACTIVATE状态激活，进行建图");
              response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_BUILDING;
              response->msg = "建图请求成功，正在进行建图工作......";
            }
          } else {
            response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
            response->msg = "上一次建图过程未完成，无法开始新的建图任务！！";
          }
        }
      }
      break;
    case byd_mapbuilder_msgs::msg::MapBuildRequest::CMD_CANCEL:
      /* 取消建图
       * 1. 如果在unconfigured状态，返回错误，当前无建图任务
       * 2. 如果在active状态，当前建图任务正在进行中， 将 enable_mapping_ = false，进入deactivate
       * 3. 如果在inactive状态，返回错误，建图任务已经取消或者完成，正在进行资源清理
       */
      {
        if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
          response->msg = "当前无建图任务，请开始建图!!!";
        } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          if (enable_mapping_) {enable_mapping_ = false;}
          response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_CANCEL;
          response->msg = "取消建图成功，当前建图任务正在取消.....";
        } else {
          response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
          response->msg = "当前建图任务已经取消或者完成，正在进行资源重置，无法取消！！";
        }
      }
      break;
    default:
      response->status.status = byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
      response->msg = "当前请求指令未知，无法进行有效处理，请检查指令是否正确";
      break;
  }
  RCLCPP_WARN(get_logger(), "## 服务请求，完成任务处理！！！");
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineCartoNode::on_configure(const rclcpp_lifecycle::State & state)
{
  // 创建cartographer节点
  auto ros_node_option = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", "/bcr_bot/scan:=scan",
      "-r", "/bcr_bot/imu:=imu",
      "-r", "/bcr_bot/odom:=odom"});
  ros_node_ = std::make_shared<rclcpp::Node>("cartographer_offline_node", ros_node_option);
  RCLCPP_INFO(get_logger(), "cartographer_ros节点创建完成!");
  // 配置STAUS_NONE
  map_build_status_ = MapBuildStatus::STATUS_IDEL;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 创建配置文件
bool LifecycleOfflineCartoNode::create_options()
{
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineCartoNode::on_activate(const rclcpp_lifecycle::State & state)
{
  carto_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // 1. 创建线程，用于处理rosbag中的数据
  // 配置STAUS_BUILDING
  map_build_status_ = MapBuildStatus::STATUS_BUILDING;
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      carto_executor_->add_node(ros_node_);
      cartographer_ros::NodeOptions node_options;
      // 从服务中获取rosbag包的路径和pbstream的输出路径,如果不存在无法建图！
      std::vector<std::string> bag_filenames;
      if (bag_file_path_.empty() || !boost::filesystem::exists(bag_file_path_) ||
      output_pbstream_path_.empty() || default_configuration_basename_.empty())
      {
        map_build_status_ = MapBuildStatus::STATUS_ERROR;
        return;
      }
      LOG(WARNING) << "bag_file_path_ : " << bag_file_path_;
      LOG(WARNING) << "output_pbstream_path_ : " << output_pbstream_path_;
      LOG(WARNING) << "建图默认配置文件 : " << default_configuration_basename_;
      bag_filenames.push_back(bag_file_path_);
      // TODO： 如果存储多个rosbag则可能存在指定的多个配置文件， 这里暂时只考虑一个configure文件
      // std::regex regex(",");
      // std::vector<std::string> configuration_basenames(
      //   std::sregex_token_iterator(
      //     FLAGS_configuration_basenames.begin(), FLAGS_configuration_basenames.end(), regex, -1),
      //   std::sregex_token_iterator()
      // );
      std::vector<std::string> configuration_basenames;
      configuration_basenames.push_back(default_configuration_basename_);
      // 不同轨迹rosbag包可能存在不同的传感器配置和
      std::vector<TrajectoryOptions> bag_trajectory_options(1);
      // 从第一configure文件中加载与ros node相关的配置，以及map_builder配置主要是PoseGraphOptions的配置
      // 所有轨迹的PoseGraphOptions配置必须相同，
      std::string configuration_directory = cartographer_shared_dir_ + "/configuration_files";
      std::tie(node_options, bag_trajectory_options.at(0)) =
      LoadOptions(configuration_directory, configuration_basenames.at(0));
      // 提取不同轨迹的TrajectoryBuilder配置，构建local_trajectory使用的传感器,TrackingFrame都可以不同
      for (size_t bag_index = 1; bag_index < bag_filenames.size(); ++bag_index) {
        TrajectoryOptions current_trajectory_options;
        if (bag_index < configuration_basenames.size()) {
          std::tie(std::ignore, current_trajectory_options) = LoadOptions(
            configuration_directory, configuration_basenames.at(bag_index));
        } else {
          current_trajectory_options = bag_trajectory_options.at(0);
        }
        bag_trajectory_options.push_back(current_trajectory_options);
      }
      if (bag_filenames.size() > 0) {
        CHECK_EQ(bag_trajectory_options.size(), bag_filenames.size());
      }

      // 由于我们预加载了变换缓冲区，因此我们永远不应该等待变换。
      // 当我们完成处理包时，我们将简单地丢弃任何由于缺少变换而无法转换的传感器数据。
      node_options.lookup_transform_timeout_sec = 0.;

      // 进行cartographer节点构建并进行逻辑处理, 创建cartographer的核心map_builder
      auto map_builder = map_builder_factory_(node_options.map_builder_options);

      std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        ros_node_->get_clock(),
        tf2::durationFromSec(10),
        ros_node_);
      // 从urdf文件中读取 对应的static_transforms变化
      std::regex regex(",");
      std::vector<geometry_msgs::msg::TransformStamped> urdf_transforms;
      if (!FLAGS_use_bag_transforms && !urdf_path_.empty()) {
        std::vector<std::string> urdf_filenames;
        urdf_filenames.push_back(urdf_path_);
        for (const auto & urdf_filename : urdf_filenames) {
          LOG(INFO) << "加载URDF文件: " << urdf_filename;
          const auto current_urdf_transforms =
          ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer);
          urdf_transforms.insert(
            urdf_transforms.end(),
            current_urdf_transforms.begin(),
            current_urdf_transforms.end());
        }
      } else if(!FLAGS_use_bag_transforms){
        map_build_status_ = MapBuildStatus::STATUS_ERROR;
        LOG(WARNING) << "没有指定urdf文件，无法发布静态tf消息";
        return;
      }
      // 发布静态tf消息，从urdf文件中读取， 开始建图流程了
      // 开启 dedicated thread 用于 tf 变换，指为TF变换处理单独分配一个后台线程，与主线程分离运行
      map_build_status_ = MapBuildStatus::STATUS_BUILDING;
      tf_buffer->setUsingDedicatedThread(true);
      // -----------------------------------------------------------------------
      const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();
      // 创建cartographer节点，进行建图工作
      Node node(node_options, std::move(map_builder), tf_buffer, ros_node_, false);
      // 发布tf消息，包括静态tf和动态tf，并保持tf的队列为1,始终使用最新的tf消息
      rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher =
      ros_node_->create_publisher<tf2_msgs::msg::TFMessage>(
        kTfTopic, kLatestOnlyPublisherQueueSize);
      ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(ros_node_);
      // 发布始终，所以use_sim_time为true，使用默认创建节点的时钟，而不是系统时钟，当然这两个时钟相同。
      rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher =
      ros_node_->create_publisher<rosgraph_msgs::msg::Clock>(
        kClockTopic, kLatestOnlyPublisherQueueSize);
      // 发布静态tf，从urdf文件中读取
      if (urdf_transforms.size() > 0) {
        static_tf_broadcaster.sendTransform(urdf_transforms);
      }

      rosgraph_msgs::msg::Clock clock;

      // 从不同的local_trajectory_opitons中提取cartograher中创建的sensor_id
      // 只有当bag数量>1时，bag_expected_sensor_ids才会出现以`bag_Num`为前缀创建sensor_id
      // 如果只有一个bag包，那么还是会以 {laserscan, odom, imu} 的名称创建sensor_id
      // 例如: [{bag_1_laserscan1, bag_1_odom1}; {bag_2_laserscan2, bag_2_odom2}];
      // 当只有一个configuration配置文件，也就时一个辆小车时，以上不会出现，它会以默认的前缀组织sensor_id
      // [{laserscan1, odom, imu}, {laserscan1, odom, imu} ...] 这样的方式出现；
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
      PlayableBagMultiplexer playable_bag_multiplexer(ros_node_);
      for (size_t current_bag_index = 0; current_bag_index < bag_filenames.size();
      ++current_bag_index)
      {
        const std::string & bag_filename = bag_filenames.at(current_bag_index);
        if (!rclcpp::ok()) {
          return;
        }
        // 将bag对应的cartographer总的topic_id进行对应，既将不同bag的{sensor_id}放到
        // 一一列举出来， (<bag_index, sensor_id_string>, sensor_id);
        // 因为cartographer_node->AddOfflineTrajectory()需要传入sensor_id_set
        // 所以不同的包创建不同的Trajectory需要明确sensor_id的集合;
        for (const auto & expected_sensor_id :
        bag_expected_sensor_ids.at(current_bag_index))
        {
          LOG(INFO) << "expected_sensor_id.id " << expected_sensor_id.id;
          const auto bag_resolved_topic = std::make_pair(
            static_cast<int>(current_bag_index),
            "/" + expected_sensor_id.id);
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
        playable_bag_multiplexer.AddPlayableBag(
          PlayableBag(
            bag_filename, current_bag_index, kDelay,
            [&tf_publisher, tf_buffer, this,
            serializer](std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {
              // TODO: filter bag msg per type ? Planned rosbag2 evolution ?
              if (msg->topic_name == kTfTopic || msg->topic_name == kTfStaticTopic) {
                if (FLAGS_use_bag_transforms) {
                  tf2_msgs::msg::TFMessage tf_message;
                  rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                  try {
                    serializer.deserialize_message(&serialized_msg, &tf_message);
                    for (auto & transform : tf_message.transforms) {
                      try {
                        tf_buffer->setTransform(
                          transform, "unused_authority",
                          msg->topic_name == kTfStaticTopic);
                      } catch (const tf2::TransformException & ex) {
                        LOG(WARNING) << ex.what();
                      }
                    }
                    tf_publisher->publish(tf_message);
                  } catch (const rclcpp::exceptions::RCLError & rcl_error) {
                    return true;
                  }
                }
                // 'PlayableBag' 进行过滤消息
                return false;
              } else {
                return true;
              }
            }));
      }

      // rosbag中包含的topic会以原数据形式放入到playable_bag_multiplexer.topics()中
      // 通过对cartographer_offline_node的ros_node节点解析remapping得到映射的topic名
      //  /bcr_bot/scan --> scan；
      // 多个bag包也可能有相同的topic；
      std::set<std::string> bag_topics;
      std::stringstream bag_topics_string;
      for (const auto & topic : playable_bag_multiplexer.topics()) {
        std::string resolved_topic = ros_node_->get_node_base_interface()->
        resolve_topic_or_service_name(topic, false);
        bag_topics.insert(resolved_topic);
        bag_topics_string << resolved_topic << ",";
      }
      bool print_topics = false;
      // TODO： 多个bag包且有多个配置时，必须指定包与sensor_id映射关系，否则会出现警告；这里因为包内topic可能有重复，
      // 所以需要指定映射指定后， 指定的映射关系bag_topics 中基本不会包含 {bag_1_laserscan1}
      // 这样的sensor_id;
      for (const auto & entry : bag_topic_to_sensor_id) {
        const std::string & resolved_topic = entry.first.second;
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
      playable_bag_multiplexer.IsMessageAvailable() ?
      playable_bag_multiplexer.PeekMessageTime() :
      rclcpp::Time();

      auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
      auto multi_echo_laser_scan_serializer =
      rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
      auto pcl2_serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
      auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
      auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
      auto nav_sat_fix_serializer = rclcpp::Serialization<sensor_msgs::msg::NavSatFix>();
      auto landmark_list_serializer = rclcpp::Serialization<cartographer_ros_msgs::msg::LandmarkList>();
      // 反光柱Laserscan处理方法，当前只支持单个传感器的Laserscan
      // 1. 生成反光柱检测器，使用实车场景中的默认参数; intesity=1600
      auto landmarks_detector = std::make_shared<landmark_localization::ReflectivePostDetector>();
      // 2. 创建反光柱id分配器；
      auto landmarks_assigner = std::make_shared<landmark_localization::LandmarkAssigner>();
      landmarks_assigner->search_range(10.0).match_threshold(0.2);
      // TODO：加入判断进行建图取消响应操作 配置 STATUS_CANCEL
      while (playable_bag_multiplexer.IsMessageAvailable() && enable_mapping_) {
        if (!::rclcpp::ok()) {
          LOG(FATAL) << "current rclcpp shutdown.";
          return;
        }

        const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
        const rosbag2_storage::SerializedBagMessage & msg = std::get<0>(next_msg_tuple);
        const int bag_index = std::get<1>(next_msg_tuple);
        const std::string topic_type = std::get<2>(next_msg_tuple);
        const bool is_last_message_in_bag = std::get<3>(next_msg_tuple);

#ifdef PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME
        if (msg.time_stamp <
        (begin_time.nanoseconds() + rclcpp::Duration(FLAGS_skip_seconds, 0).nanoseconds()))
        {
          continue;
        }
#else
        if (msg.recv_timestamp <
        (begin_time.nanoseconds() + rclcpp::Duration(FLAGS_skip_seconds, 0).nanoseconds()))
        {
          continue;
        }
#endif

        int trajectory_id;
        // Lazily add trajectories only when the first message arrives in order
        // to avoid blocking the sensor queue.
        if (bag_index_to_trajectory_id.count(bag_index) == 0) {
          trajectory_id =
          node.AddOfflineTrajectory(
            bag_expected_sensor_ids.at(bag_index),
            bag_trajectory_options.at(bag_index));
          CHECK(
            bag_index_to_trajectory_id
            .emplace(
              std::piecewise_construct,
              std::forward_as_tuple(bag_index),
              std::forward_as_tuple(trajectory_id))
            .second);
          LOG(INFO) << "Assigned trajectory " << trajectory_id << " to bag "
                    << bag_filenames.at(bag_index);
        } else {
          trajectory_id = bag_index_to_trajectory_id.at(bag_index);
        }
        // 对于多个bag包，只有作了bag_1_scan_1 -> scan这样的remapping，才能找到对应的sensor_id
        // 当bag_topic_to_sensor_id
        const auto bag_topic = std::make_pair(
          bag_index,
          ros_node_->get_node_base_interface()->
          resolve_topic_or_service_name(msg.topic_name, false));
        auto it = bag_topic_to_sensor_id.find(bag_topic);
        // 找到对应传感器类型进行处理建图
        if (it != bag_topic_to_sensor_id.end()) {
          const std::string & sensor_id = it->second.id;
          if (topic_type == "sensor_msgs/msg/LaserScan") {
            rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
            sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg =
            std::make_shared<sensor_msgs::msg::LaserScan>();
            laser_scan_serializer.deserialize_message(&serialized_msg, laser_scan_msg.get());
            // TODO: 进行landmarkers的反光柱消息提取，进行处理，分配到landmark处理过程中。
            // 分配器还没有多Laser映射机制，目前只能拿单雷达进ID维护
            if (bag_trajectory_options.at(bag_index).use_landmarks &&
            sensor_id == "scan" && !landmarks_assigner->isBase2LaserTransOK())
            {
              geometry_msgs::msg::TransformStamped transform;
              try {
                tf_buffer->canTransform(
                  "base_link", laser_scan_msg->header.frame_id,
                  tf2::TimePointZero);
                transform = tf_buffer->lookupTransform(
                  "base_link",
                  laser_scan_msg->header.frame_id,
                  tf2::TimePointZero);
              } catch (const tf2::TransformException & ex) {
                LOG(FATAL) << "TF2 查找 base_link 2 " << laser_scan_msg->header.frame_id
                           << "error: " << ex.what();
              }
              landmarks_assigner->setBase2LaserTrans(transforms::ToRigid3d(transform));
              LOG(
                WARNING) << "反光柱检测器分配器设置base_link To " << laser_scan_msg->header.frame_id << " 变换";
            }
            // 2. 激光雷达扫描处理
            node.HandleLaserScanMessage(trajectory_id, sensor_id, laser_scan_msg);
            // 1. NOTICE: 获取当前laserscan后，匹配的tracked_pose；可能有多个包，但仍然是单车单雷达可通过获取当前对应
            // 轨迹的全局tracked_pose，来更新assigner中的位姿，维持数据，要确保assigner中的位姿是最新的，
            // 也就是说多个包的情况也是按时间顺序发布的。
            if(!bag_trajectory_options.at(bag_index).use_landmarks || sensor_id != "scan") {
              continue;
            }
            for (const auto & entry : node.map_builder_bridge_->GetLocalTrajectoryData()) {
              const auto & trajectory_data = entry.second;
              const cartographer::transform::Rigid3d tracking_to_local_3d = trajectory_data.local_slam_data->local_pose;
              ::geometry_msgs::msg::PoseStamped pose_msg;
              pose_msg.header.stamp = ToRos(
                trajectory_data.local_slam_data->time);
              pose_msg.header.frame_id = "map";
              const cartographer::transform::Rigid3d tracking_to_map = trajectory_data.local_to_map * tracking_to_local_3d;
              pose_msg.pose = cartographer_ros::ToGeometryMsgPose(tracking_to_map);
              landmarks_assigner->update_tracked_pose(
                transforms::ToRigid3d(pose_msg.pose),
                pose_msg.header.stamp.nanosec);
            }
            // 2. 对当前scan中的landmarkers进行匹配，更新tracked_pose的landmarkers；
            landmark_localization::LaserScan scan;
            scan.header = laser_scan_msg->header.frame_id;
            scan.ranges = laser_scan_msg->ranges;
            scan.intensities = laser_scan_msg->intensities;
            scan.angle_min = laser_scan_msg->angle_min;
            scan.angle_max = laser_scan_msg->angle_max;
            scan.angle_increment = laser_scan_msg->angle_increment;
            scan.scan_time = laser_scan_msg->scan_time;
            scan.range_min = laser_scan_msg->range_min;
            scan.range_max = laser_scan_msg->range_max;
            auto detected_posts = landmarks_detector->detect_circles(scan);
            if (!landmarks_assigner->isLandmarkDetectorOK(laser_scan_msg->header.stamp.nanosec)) {
              LOG(WARNING) << "反光柱分配器未初始化完成!! 请检查TF和tracked_pose!";
              continue;
            }
            if (detected_posts.empty()) {
              LOG(INFO) << "未检测到有效反光柱";
              continue;
            }
            auto reflector_posts = landmarks_assigner->assignLandmarkToReflectorBar(detected_posts);
            cartographer_ros_msgs::msg::LandmarkList::SharedPtr landmark_list_msg =
            std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
            landmark_list_msg->header = laser_scan_msg->header;
            std::vector<cartographer_ros_msgs::msg::LandmarkEntry> poses_array;
            cartographer_ros_msgs::msg::LandmarkEntry poseSimple;
            for (auto & landmark : reflector_posts) {
              poseSimple.tracking_from_landmark_transform = transforms::ToGeometryMsgPose(
                landmark.g_detection_.pose.pose);
              poseSimple.translation_weight = landmark.g_detection_.translationW *
              landmark_translation_weight_;
              poseSimple.rotation_weight = landmark_rotation_weight_;
              poseSimple.id = landmark.id_str_;
              poses_array.push_back(poseSimple);
            }
            landmark_list_msg->landmarks = poses_array;
            node.HandleLandmarkMessage(
              trajectory_id, cartographer_ros::kLandmarkTopic,
              landmark_list_msg);

            // 3. 更新tracked_pose的landmarkers；这里以now()使用仿真时间来驱别于laser_scan的时间戳
            rclcpp::Time landmarks_ros_timestamp = ros_node_->now();
            auto landmark_makers_msg = node.map_builder_bridge_->GetLandmarkPosesList(ros_node_->now());
            std::map<int, transforms::Rigid3d> optimized_landmarks;
            int64_t optimized_timestamp;
            for (const auto & marker_msg : landmark_makers_msg.markers) {
              // 处理反光柱位姿
              if (marker_msg.ns == "Landmarks" && marker_msg.header.frame_id == "map") {
                int id = marker_msg.id;
                optimized_landmarks[id] = transforms::ToRigid3d(marker_msg.pose);
              }
            }
            optimized_timestamp = landmarks_ros_timestamp.nanoseconds();
            if (!optimized_landmarks.empty()) {
              LOG(INFO) << "反光柱分配器更新优化后的反光柱位姿: 更新" << optimized_landmarks.size() << "个反光柱位姿";
            }
            landmarks_assigner->update_landmarks(optimized_landmarks, optimized_timestamp);
          } else if (topic_type == "sensor_msgs/msg/MultiEchoLaserScan") {
            rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
            sensor_msgs::msg::MultiEchoLaserScan::SharedPtr multi_echo_laser_scan_msg =
            std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
            multi_echo_laser_scan_serializer.deserialize_message(
              &serialized_msg,
              multi_echo_laser_scan_msg.get());
            node.HandleMultiEchoLaserScanMessage(
              trajectory_id, sensor_id,
              multi_echo_laser_scan_msg);
          } else if (topic_type == "sensor_msgs/msg/PointCloud2") {
            rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
            sensor_msgs::msg::PointCloud2::SharedPtr pcl2_scan_msg =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl2_serializer.deserialize_message(&serialized_msg, pcl2_scan_msg.get());
            node.HandlePointCloud2Message(
              trajectory_id, sensor_id,
              pcl2_scan_msg);
          } else if (topic_type == "sensor_msgs/msg/Imu") {
            rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
            sensor_msgs::msg::Imu::SharedPtr imu_scan_msg =
            std::make_shared<sensor_msgs::msg::Imu>();
            imu_serializer.deserialize_message(&serialized_msg, imu_scan_msg.get());
            node.HandleImuMessage(
              trajectory_id, sensor_id,
              imu_scan_msg);
          } else if (topic_type == "nav_msgs/msg/Odometry") {
            rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
            nav_msgs::msg::Odometry::SharedPtr odom_scan_msg =
            std::make_shared<nav_msgs::msg::Odometry>();
            odom_serializer.deserialize_message(&serialized_msg, odom_scan_msg.get());
            node.HandleOdometryMessage(
              trajectory_id, sensor_id,
              odom_scan_msg);
          } else if (topic_type == "sensor_msgs/msg/NavSatFix") {
            rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
            sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_msg =
            std::make_shared<sensor_msgs::msg::NavSatFix>();
            nav_sat_fix_serializer.deserialize_message(&serialized_msg, nav_sat_fix_msg.get());
            node.HandleNavSatFixMessage(
              trajectory_id, sensor_id,
              nav_sat_fix_msg);
          } else if (topic_type == "cartographer_ros_msgs/msg/LandmarkList") {
            rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
            cartographer_ros_msgs::msg::LandmarkList::SharedPtr landmark_list_msg =
            std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
            landmark_list_serializer.deserialize_message(&serialized_msg, landmark_list_msg.get());
            node.HandleLandmarkMessage(
              trajectory_id, sensor_id,
              landmark_list_msg);
          }
        }
        // PlayableBagMultiplexer 时按顺序进行排序的，所以这里使用其对应的msg时间作为clock时间发布
        // 不然从rosbag中提取tf_buffer的tflook会报错。
#ifdef PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME
        clock.clock = rclcpp::Time(msg.time_stamp);
#else
        clock.clock = rclcpp::Time(msg.recv_timestamp);
#endif
        clock_publisher->publish(clock);
        carto_executor_->spin_some();
        if (is_last_message_in_bag) {
          node.FinishTrajectory(trajectory_id);
        }
      }
      //--------------------- 完成rosbag发布 -------------------------
      // TODO: need a spin for the timer to tick
      // 确保的FinalOptimization时，还有/clock消息发布，默认30hz； 但这个时间是最后一个msg时间，
      // 所以这里需要手动发布一次，确保最后一个msg时间的clock时间也发布出去。
      auto clock_republish_timer = ros_node_->create_wall_timer(
        std::chrono::milliseconds(int(kClockPublishFrequencySec)),
        [&clock_publisher, &clock]() {
          clock_publisher->publish(clock);
        });
      node.RunFinalOptimization();
      // 统计建图时间！！
      const std::chrono::time_point<std::chrono::steady_clock> end_time =
      std::chrono::steady_clock::now();
      const double wall_clock_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(
        end_time -
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
      !(bag_filenames.empty() && output_pbstream_path_.empty()) && enable_mapping_)
      {
        const std::string state_output_filename =
        output_pbstream_path_.empty() ? bag_filenames.front() + ".pbstream" : output_pbstream_path_;
        LOG(INFO) << "....正在保存pbstream地图文件: '" << state_output_filename << "'...";
        node.SerializeState(
          state_output_filename,
          true /* include_unfinished_submaps */);
        // TODO: 保存SMAP
        while (!boost::filesystem::exists(output_pbstream_path_)) {
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        LOG(INFO) << "完成保存地图文件: '" << state_output_filename << "'...";
        // 读取pbsteam转换成smap
        cartographer::io::ProtoStreamReader reader(state_output_filename);
        cartographer::io::ProtoStreamDeserializer deserializer(&reader);
        LOG(INFO) << "加载pbstream地图.......";
        std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;
        cartographer::mapping::ValueConversionTables conversion_tables;
        cartographer::io::DeserializeAndFillSubmapSlices(
          &deserializer, &submap_slices, &conversion_tables);
        CHECK(reader.eof());
        LOG(INFO) << "生成地图切片submap slices.";
        auto result =
        ::cartographer::io::PaintSubmapSlices(submap_slices, resolution_);
        // 生成pgm和yaml,以及smap
        std::string map_filestem = cartographer_install_dir_ + "/" + map_filestem_;
        cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

        cartographer::io::Image image(std::move(result.surface));

        const Eigen::Vector2d origin(
          -result.origin.x() * resolution_,
          (result.origin.y() - image.height()) * resolution_);

        WritePgm(image, resolution_, &pgm_writer, origin, state_output_filename);

        cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
        WriteYaml(resolution_, origin, pgm_writer.GetFilename(), &yaml_writer);
        LOG(INFO) << "生成SMap地图.....";
        LOG(INFO) << "完成carographer offline建图流程!!.";
      }

      // 3. 节点与执行器解绑
      node.submap_list_publisher_.reset();
      node.trajectory_node_list_publisher_.reset();
      node.tracked_pose_publisher_.reset();
      node.scan_matched_point_cloud_publisher_.reset();
      node.constraint_list_publisher_.reset();
      node.landmark_poses_list_publisher_.reset();
      node.submap_list_timer_.reset();
      node.local_trajectory_data_timer_.reset();
      node.trajectory_node_list_timer_.reset();
      node.landmark_pose_list_timer_.reset();
      node.constrain_list_timer_.reset();
      node.maybe_warn_about_topic_mismatch_timer_.reset();
      clock_republish_timer.reset();
      landmarks_assigner.reset();
      landmarks_detector.reset();
      LOG(INFO) << "完成节点资源清理建图流程!!.";
      // 配置STAUS_COMPLETE 完成建图
      if (!enable_mapping_) {
        LOG(WARNING) << "未完成carographer offline建图流程, 过程中被取消!!.";
        map_build_status_ = MapBuildStatus::STATUS_CANCEL;
      } else {
        map_build_status_ = MapBuildStatus::STATUS_COMPLETE;
      }
      LOG(INFO) << "carto_exector清理rosnode_";
      carto_executor_->remove_node(ros_node_);
      LOG(INFO) << "进行INACTIVATE 状态转换!!!!";
      std::thread(
        [&]() {
          // 等待当前节点进入 INACTIVE 状态（确保 deactivate 完成）
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          // 3. 发送 cleanup 转换请求（INACTIVE → Unconfigured）
          auto client =
          this->create_client<lifecycle_msgs::srv::ChangeState>(
            "/lifecycle_cartographer_node/change_state");
          while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "等待 /change_state 服务可用...");
          }

          auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
          request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;    // cleanup 转换ID

          auto result = client->async_send_request(request).get();
          if (result->success) {
            RCLCPP_INFO(get_logger(), "deactivate 转换触发成功 (ACTIVE → INACTIVE )");
          } else {
            RCLCPP_ERROR(get_logger(), "deactivate 转换触发失败");
          }
        }).detach();  // 异步执行，不阻塞当前回调
    });
  // 2. 启动线程
  LOG(INFO) << "cartographer_ros节点启动完成!";
  process_pub_->on_activate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineCartoNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // 1. 回收线程
  LOG(INFO) << "DEACTIVATING 从 ACTIVATE -> INACTIVAE 状态转换";
  thread_->join();
  LOG(INFO) << "thread_join() 完成，cartographer安全退出";
  carto_executor_->cancel();
  LOG(INFO) << "carto_exector取消spin()";
  // 2. 重置enable_mapping_
  if (!enable_mapping_) {
    enable_mapping_ = true;
  }
  if (map_build_status_ == MapBuildStatus::STATUS_BUILDING) {
    LOG(WARNING) << "建图节点状态存在问题请检查代码，此时建图已经完成仍然为building状态";
  } else if (map_build_status_ == MapBuildStatus::STATUS_CANCEL) {
    LOG(WARNING) << "建图任务已经取消，进行资源重置";
  } else if (map_build_status_ == MapBuildStatus::STATUS_COMPLETE) {
    LOG(WARNING) << "建图任务已经完成并保存pbstream地图资源，进行资源重置";
  } else if (map_build_status_ == MapBuildStatus::STATUS_ERROR) {
    LOG(WARNING) << "建图过程中出现错误，请检查on_activate状态代码";
  }
  // 3. 跳转UNCONFIGURED状态
  LOG(INFO) << "完成建图过程，停止cartographer节点，并切换到unconfigure状态.....";
  // trigger_transition(rclcpp_lifecycle::Transition(2, "TRANSITION_CLEANINGUP"));

  std::thread(
    [&]() {
      // 等待当前节点进入 INACTIVE 状态（确保 deactivate 完成）
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      // 3. 发送 cleanup 转换请求（INACTIVE → Unconfigured）
      auto client =
      this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/lifecycle_cartographer_node/change_state");
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "等待 /change_state 服务可用...");
      }

      auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;        // cleanup 转换ID

      auto result = client->async_send_request(request).get();
      if (result->success) {
        RCLCPP_INFO(get_logger(), "cleanup 转换触发成功（INACTIVE → Unconfigured）");
      } else {
        RCLCPP_ERROR(get_logger(), "cleanup 转换触发失败");
      }
    }).detach();      // 异步执行，不阻塞当前回调

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineCartoNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  thread_.reset();
  ros_node_.reset();
  LOG(INFO) << "cartographer_ros节点清理完成! 进入unconfigured状态";
  map_build_status_ = MapBuildStatus::STATUS_IDEL;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineCartoNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  LOG(WARNING) << "不允许生命周期节点shutdown不然无法恢复";
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


}  // namespace cartographer_ros
