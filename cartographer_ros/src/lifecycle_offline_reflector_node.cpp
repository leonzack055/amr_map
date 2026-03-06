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

#include "cartographer_ros/lifecycle_offline_reflector_node.h"

#include <errno.h>

#include <string>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "byd_mapbuilder_msgs/msg/map_build_process.hpp"
#include "cartographer_ros_msgs/msg/bagfile_progress.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/transition.hpp"
#ifndef WIN32
#include <sys/resource.h>
#endif
#include <time.h>

#include <atomic>
#include <chrono>
#include <future>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/playable_bag.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#ifdef USE_URDF_H_FILES
#include "urdf/model.h"
#else
#include "urdf/model.hpp"
#endif
#include <boost/filesystem.hpp>
#include <regex>
#include <string>

#include "rclcpp/exceptions.hpp"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish transforms from bags.");
DEFINE_double(skip_seconds, 0,
              "Optional amount of seconds to skip from the beginning "
              "(i.e. when the earliest bag starts.). ");
DEFINE_bool(use_default_bagDir, true,
            "Optional where the ros bags for offline mapping is installed "
            "(abs_DIR: /home/admin/map_dir .). ");

namespace cartographer_ros {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "/tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;
const rclcpp::Duration kDelay(1.0, 0);

using namespace amr_reflector_noise_handling;

// Convert cartographer transform to local transform type
transforms::Rigid3d convert_carto_transform(
    const cartographer::transform::Rigid3d& transform) {
  return transforms::Rigid3d(transform.translation(), transform.rotation());
}

LifecycleOfflineReflectorNode::LifecycleOfflineReflectorNode(
    const std::string& node_name, bool intra_process_comms,
    rclcpp::Executor::SharedPtr executor,
    const MapBuilderFactory& map_builder_factory)
    : rclcpp_lifecycle::LifecycleNode(
          node_name,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {
  // TODO: 外部指resolution
  executor_ = executor;
  // resolution_ = 0.05f;
  map_builder_factory_ = map_builder_factory;

  // Create map build service
  map_build_srv_ = this->create_service<byd_mapbuilder_msgs::srv::MapBuild>(
      "build_map_service",
      std::bind(&LifecycleOfflineReflectorNode::map_build_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  map_build_status_ = MapBuildStatus::STATUS_IDEL;
  enable_mapping_ = true;

  // Create process publisher
  process_pub_ =
      this->create_publisher<byd_mapbuilder_msgs::msg::MapBuildProcess>(
          "/mapping_process", 10);
  bag_process_sub_ =
      this->create_subscription<cartographer_ros_msgs::msg::BagfileProgress>(
          "/bagfile_progress", 10,
          std::bind(&LifecycleOfflineReflectorNode::bagProcessDataCallback,
                    this, std::placeholders::_1));

  rosbag_dir_ = "";
  cartoConfig_dir_ = "";
  this->declare_parameter("rosbag_dir", "/home/admin/map_dir/bag_dir");
  this->get_parameter<std::string>("rosbag_dir", this->rosbag_dir_);
  this->declare_parameter("cartoConfig_dir",
                          "/home/admin/map_dir/carto_config");
  this->get_parameter<std::string>("cartoConfig_dir", this->cartoConfig_dir_);
  this->declare_parameter("output_dir", "/home/admin/map_dir/output_dir");
  this->get_parameter<std::string>("output_dir",
                                   this->cartographer_output_dir_);

  // Get package paths
  std::string package_name = "cartographer_ros";
  std::string use_urdf_file = "/urdf/byd_amr.urdf";
  std::string package_prefix =
      ament_index_cpp::get_package_prefix(package_name);
  std::string package_shared =
      ament_index_cpp::get_package_share_directory(package_name);
  cartographer_shared_dir_ = package_shared;
  cartographer_install_dir_ = package_prefix;
  RCLCPP_INFO(this->get_logger(), "%s 包的安装索引路径: %s\nshare配置路径； %s",
              package_name.c_str(), package_prefix.c_str(),
              package_shared.c_str());
  if (FLAGS_use_default_bagDir) {
    cartographer_install_dir_ = rosbag_dir_;
    cartographer_shared_dir_ = cartoConfig_dir_;
    cartographer_output_dir_ = cartographer_output_dir_;
    LOG(INFO) << "使用用户指定目录:";
    LOG(INFO) << "\t[✔] rosbag包目录:" << cartographer_install_dir_;
    LOG(INFO) << "\t[✔] carto建图参数目录:" << cartographer_shared_dir_;
    LOG(INFO) << "\t[✔] 地图输出目录:" << cartographer_output_dir_;
  } else {
    cartographer_output_dir_ = cartographer_install_dir_;
  }
  urdf_path_ = cartographer_shared_dir_ + use_urdf_file;
  RCLCPP_INFO(this->get_logger(), "%s 包的安装路径: %s share路径； %s",
              package_name.c_str(), package_prefix.c_str(),
              package_shared.c_str());

  // Initialize landmark weights
  landmark_translation_weight_ = 1e5;
  landmark_rotation_weight_ = 1e1;
}

void LifecycleOfflineReflectorNode::bagProcessDataCallback(
    const cartographer_ros_msgs::msg::BagfileProgress::SharedPtr msg) {
  float total_seconds = msg->total_seconds;
  float processed_seconds = msg->processed_seconds;
  float processed_percentage = processed_seconds / total_seconds;
  auto bag_msg = std::make_unique<byd_mapbuilder_msgs::msg::MapBuildProcess>();
  bag_msg->progress = processed_percentage;

  auto now = std::chrono::system_clock::now();
  auto now_epoch = now.time_since_epoch();
  auto sec =
      std::chrono::duration_cast<std::chrono::seconds>(now_epoch).count();
  auto nsec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now_epoch).count() %
      1000000000;

  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(sec);
  ros_time.nanosec = static_cast<uint32_t>(nsec);
  std_msgs::msg::Header header;
  header.stamp = ros_time;
  header.frame_id = "base_link";
  bag_msg->header = header;
  process_pub_->publish(std::move(bag_msg));
}

void LifecycleOfflineReflectorNode::map_build_callback(
    const std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Request> request,
    std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Response> response) {
  switch (request->req.cmd_id) {
    case byd_mapbuilder_msgs::msg::MapBuildRequest::CMD_IDEL: {
      if (get_current_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        if (map_build_status_ == MapBuildStatus::STATUS_IDEL) {
          response->status.status =
              byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_NONE;
          response->msg = "当前建图节点没有建图任务，处于空闲状态";
        } else {
          LOG(WARNING) << "map_build_status_: " << map_build_status_
                       << "当前建图节点内部状态";
          response->status.status =
              byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
          response->msg = "当前节点处于未知状态，请检查相关代码";
        }
      } else if (get_current_state().id() ==
                 lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        response->status.status =
            byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_BUILDING;
        response->msg = "当前建图节点正在进行建图工作";
        response->status.progress = 0.5;
      } else if (get_current_state().id() ==
                 lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        response->status.status =
            byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_BUILDING;
        response->msg = "当前建图节点正在进行地图保存和资源清理中.....";
      }
    } break;
    case byd_mapbuilder_msgs::msg::MapBuildRequest::CMD_START: {
      if (request->bagfile.empty() || request->filename.empty()) {
        response->status.status =
            byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
        response->msg = "请求参数错误，bagfile和filename不能为空";
      } else {
        std::string bagfile = request->bagfile;
        std::string output_pbstream_path = request->filename;
        std::string config_carto_lua = request->configfile;
        std::string reflector_param_file = request->reflector_param_file;
        reflector_param_file_ = reflector_param_file;
        // TODO: 新加参数文件参数，用于指定lua文件
        // TODO: 修改保存地图路径
        default_configuration_basename_ = config_carto_lua;
        bag_file_path_ = cartographer_install_dir_ + "/" + bagfile;
        output_pbstream_path_ =
            cartographer_output_dir_ + "/" + output_pbstream_path;
        map_filestem_ = output_pbstream_path;
        const std::string kPbstreamSuffix = ".pbstream";
        if (output_pbstream_path.length() >= kPbstreamSuffix.length() &&
            output_pbstream_path.substr(output_pbstream_path.length() -
                                        kPbstreamSuffix.length()) ==
                kPbstreamSuffix) {
          map_filestem_ = output_pbstream_path.substr(
              0, output_pbstream_path.length() - kPbstreamSuffix.length());
        }
        RCLCPP_INFO(get_logger(), "rosbag包名称：%s", bag_file_path_.c_str());
        RCLCPP_INFO(get_logger(), "保存地图pbstream名称：%s",
                    output_pbstream_path_.c_str());
        RCLCPP_INFO(get_logger(), "保存地图Smap名称：%s",
                    map_filestem_.c_str());
        if (!boost::filesystem::exists(bag_file_path_)) {
          LOG(ERROR) << "指定的rosbag包路径不存在: " << bag_file_path_;
          response->status.status =
              byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
          response->msg = "建图失败！ 指定的rosbag包路径不存在!!";
          break;
        }
        if (get_current_state().id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
            map_build_status_ == MapBuildStatus::STATUS_IDEL) {
          RCLCPP_INFO(get_logger(),
                      "当前节点状态为unconfigured，准备切换到configuring状态");
          auto current_state = trigger_transition(
              lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
          std::atomic<bool> stop_flag(false);
          std::future<void> configure_future =
              std::async(std::launch::async, [&] {
                while (get_current_state().id() !=
                       lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                  if (stop_flag.load(std::memory_order_relaxed)) {
                    RCLCPP_INFO(get_logger(), "终止ACTIVIATE状态切换");
                    return;
                  }
                  if (get_current_state().id() ==
                      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                    rclcpp::sleep_for(std::chrono::milliseconds(1000));
                    RCLCPP_INFO(
                        get_logger(),
                        "当前节点状态为configuring,准备切换到inactive状态");
                    continue;
                  }
                  LOG(WARNING)
                      << "当前节点的运行状态为: " << get_current_state().label()
                      << int(get_current_state().id())
                      << " 没有完成inactivate的状态转换";
                }
              });
          auto status = configure_future.wait_for(std::chrono::seconds(10));
          if (status == std::future_status::timeout) {
            RCLCPP_INFO(get_logger(), "configuring 等待超时10s，尝试终止任务");
            stop_flag.store(true, std::memory_order_relaxed);
            configure_future.wait();
            response->status.status =
                byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
            response->msg =
                "10s配置任务请求超时，建图请求成功失败，请检查代码......";
            return;
          } else if (status == std::future_status::ready) {
            RCLCPP_INFO(get_logger(),
                        "configure配置成功, "
                        "进入ACTIVATE状态激活，进行建图ACTIVATEd激活");
          }
          stop_flag = false;
          trigger_transition(
              lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
          std::future<void> activate_future =
              std::async(std::launch::async, [&] {
                while (get_current_state().id() !=
                       lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                  if (stop_flag.load(std::memory_order_relaxed)) {
                    RCLCPP_INFO(get_logger(), "终止ACTIVIATE状态切换");
                    return;
                  }
                  if (get_current_state().id() ==
                      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                    rclcpp::sleep_for(std::chrono::milliseconds(1000));
                    RCLCPP_INFO(
                        get_logger(),
                        "当前节点状态为inactivate状态,准备切换到active状态");
                    continue;
                  }
                  LOG(WARNING)
                      << "当前节点的运行状态为: " << get_current_state().label()
                      << int(get_current_state().id())
                      << " 没有完成activate的状态转换";
                }
              });
          status = activate_future.wait_for(std::chrono::seconds(10));
          if (status == std::future_status::timeout) {
            RCLCPP_INFO(get_logger(), "等待超时10s，尝试终止任务");
            stop_flag.store(true, std::memory_order_relaxed);
            activate_future.wait();
            response->status.status =
                byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
            response->msg =
                "10s任务请求超时，建图请求成功失败，请检查代码......";
          } else if (status == std::future_status::ready) {
            RCLCPP_INFO(get_logger(),
                        "任务在超时前完成, ACTIVATE状态激活，进行建图");
            response->status.status =
                byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_BUILDING;
            response->msg = "建图请求成功，正在进行建图工作......";
          }
        } else {
          response->status.status =
              byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
          response->msg = "上一次建图过程未完成，无法开始新的建图任务！！";
        }
      }
    } break;
    case byd_mapbuilder_msgs::msg::MapBuildRequest::CMD_CANCEL: {
      if (get_current_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        response->status.status =
            byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
        response->msg = "当前无建图任务，请开始建图!!!";
      } else if (get_current_state().id() ==
                 lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        if (enable_mapping_) {
          enable_mapping_ = false;
        }
        response->status.status =
            byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_CANCEL;
        response->msg = "取消建图成功，当前建图任务正在取消.....";
      } else {
        response->status.status =
            byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
        response->msg =
            "当前建图任务已经取消或者完成，正在进行资源重置，无法取消！！";
      }
    } break;
    default:
      response->status.status =
          byd_mapbuilder_msgs::msg::MapBuildStatus::STATUS_FAILED;
      response->msg = "当前请求指令未知，无法进行有效处理，请检查指令是否正确";
      break;
  }
  RCLCPP_WARN(get_logger(), "## 服务请求，完成任务处理！！！");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineReflectorNode::on_configure(
    const rclcpp_lifecycle::State& state) {
  // TODO: 加入params_file_path利用其进行cartographer相应参数绑定
  std::string params_file_path =
      "/home/admin/map_dir/reflector_config/" + reflector_param_file_;
  auto ros_node_option = rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r", "/bcr_bot/scan:=scan", "-r", "/scan:=scan", "-r",
       "/odom_combined:=odom", "-p", "params_file:=" + params_file_path});
  ros_node_ = std::make_shared<rclcpp::Node>(
      "cartographer_offline_reflector_node", ros_node_option);
  RCLCPP_INFO(get_logger(), "cartographer_ros反光柱建图节点创建完成!");
  // 反光柱tf信息
  reflector_scan_frame_ = "laser";
  reflector_base_frame_ = "base_link";
  // Configure detection modules
  configureDetectionModules();

  map_build_status_ = MapBuildStatus::STATUS_IDEL;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

bool LifecycleOfflineReflectorNode::create_options() { return true; }

// ==================== Reflector Detection Configuration ====================

void LifecycleOfflineReflectorNode::configureDetectionModules() {
  // Initialize reflector detection parameters
  current_frame_index_ = 0;
  use_distort_corrector_ = false;
  distortcorrect_method_ = "Spline";
  use_short_tracker_ = false;
  // Initialize ROS parameters
  ros_node_->declare_parameter("scan_topic", "/scan");
  ros_node_->declare_parameter("odom_topic", "/odom_combined");
  ros_node_->declare_parameter("classification_method", "pca");
  ros_node_->declare_parameter("raw_intensity_threshold", 1000.0);
  ros_node_->declare_parameter("min_confidence", 0.5);
  // Get parameters
  scan_topic_ = ros_node_->get_parameter("scan_topic").as_string();
  odom_topic_ = ros_node_->get_parameter("odom_topic").as_string();
  classification_method_ =
      ros_node_->get_parameter("classification_method").as_string();
  raw_intensity_threshold_ =
      ros_node_->get_parameter("raw_intensity_threshold").as_double();
  min_confidence_ = ros_node_->get_parameter("min_confidence").as_double();
  // 配置点云矫正器
  configureDistortionCorrector();
  // 配置聚类器
  configureFixedDBSCAN();
  // Configure detection modules
  configurePCAClassification();
  configureCircleFit();
  configureGlobalTracker();

  RCLCPP_INFO(this->get_logger(), "反光柱检测模块配置完成");
}

void LifecycleOfflineReflectorNode::configureDistortionCorrector() {
  ros_node_->declare_parameter("distort_corrector.enable", false);
  this->use_distort_corrector_ =
      ros_node_->get_parameter("distort_corrector.enable").as_bool();
  ros_node_->declare_parameter("distort_corrector.method", "Spline");
  this->distortcorrect_method_ =
      ros_node_->get_parameter("distort_corrector.method").as_string();

  if (use_distort_corrector_) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[✔] 配置点云矫正器: " << distortcorrect_method_);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "[✘] 没有使用点云矫正器！！！！");
  }
}

void LifecycleOfflineReflectorNode::configureFixedDBSCAN() {
  FixedDBSCAN::Config dbscan_config;
  ros_node_->declare_parameter("fixed_dbscan.eps", 0.05);
  ros_node_->declare_parameter("fixed_dbscan.min_points", 5);
  ros_node_->declare_parameter("fixed_dbscan.contine_gap", 3);
  ros_node_->declare_parameter("fixed_dbscan.continue_points", 8);

  dbscan_config.eps = ros_node_->get_parameter("fixed_dbscan.eps").as_double();
  dbscan_config.min_points =
      ros_node_->get_parameter("fixed_dbscan.min_points").as_int();
  dbscan_config.gap_threshold =
      ros_node_->get_parameter("fixed_dbscan.contine_gap").as_int();
  dbscan_config.continue_points =
      ros_node_->get_parameter("fixed_dbscan.continue_points").as_int();
  this->fixed_dbscan_ = FixedDBSCAN(dbscan_config);
  // 打印配置信息
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "[✔] 配置FixedDBSCAN.eps: " << dbscan_config.eps);
  RCLCPP_INFO_STREAM(this->get_logger(), "[✔] 配置FixedDBSCAN.min_points: "
                                             << dbscan_config.min_points);
  RCLCPP_INFO_STREAM(this->get_logger(), "[✔] 配置FixedDBSCAN.contine_gap: "
                                             << dbscan_config.gap_threshold);
  RCLCPP_INFO_STREAM(this->get_logger(), "[✔] 配置FixedDBSCAN.continue_points: "
                                             << dbscan_config.continue_points);
}

void LifecycleOfflineReflectorNode::configurePCAClassification() {
  ros_node_->declare_parameter("pca_classification.enable", true);
  ros_node_->declare_parameter("pca_classification.min_points", 13);
  ros_node_->declare_parameter("pca_classification.max_elongation_post", 9.5);
  ros_node_->declare_parameter("pca_classification.min_elongation_board", 12.0);
  ros_node_->declare_parameter("pca_classification.max_linearity_post", 0.97);
  ros_node_->declare_parameter("pca_classification.min_linearity_board", 0.93);
  ros_node_->declare_parameter("pca_classification.near_distance", 1.3);
  ros_node_->declare_parameter("pca_classification.near_min_points", 20);
  ros_node_->declare_parameter("pca_classification.near_max_linearity_post",
                               0.89);

  bool use_pca_classification =
      ros_node_->get_parameter("pca_classification.enable").as_bool();

  ShapeClassificationParams pca_params;
  pca_params.min_points =
      ros_node_->get_parameter("pca_classification.min_points").as_int();
  pca_params.max_elongation_post =
      ros_node_->get_parameter("pca_classification.max_elongation_post")
          .as_double();
  pca_params.min_elongation_board =
      ros_node_->get_parameter("pca_classification.min_elongation_board")
          .as_double();
  pca_params.max_linearity_post =
      ros_node_->get_parameter("pca_classification.max_linearity_post")
          .as_double();
  pca_params.min_linearity_board =
      ros_node_->get_parameter("pca_classification.min_linearity_board")
          .as_double();
  pca_params.near_distance =
      ros_node_->get_parameter("pca_classification.near_distance").as_double();
  pca_params.near_min_points =
      ros_node_->get_parameter("pca_classification.near_min_points").as_int();
  pca_params.near_max_linearity_post =
      ros_node_->get_parameter("pca_classification.near_max_linearity_post")
          .as_double();
  // 配置
  reflector_detector_.setDetectMethod(classification_method_);
  if (use_pca_classification) {
    reflector_detector_.configPCAShapeClassifier(pca_params);
  }
}

void LifecycleOfflineReflectorNode::configureCircleFit() {
  ros_node_->declare_parameter("circle_fit.max_fit_error", 0.03);
  ros_node_->declare_parameter("circle_fit.min_inlier_ratio", 0.5);
  ros_node_->declare_parameter("circle_fit.max_fit_error_near", 0.01);
  ros_node_->declare_parameter("circle_fit.max_fit_error_far", 0.02);
  ros_node_->declare_parameter("circle_fit.far_distance_threshold", 1.5);
  ros_node_->declare_parameter("circle_fit.min_radius", 0.02);
  ros_node_->declare_parameter("circle_fit.max_radius", 0.05);
  ros_node_->declare_parameter("circle_fit.max_concave_ratio", 0.2);
  ros_node_->declare_parameter("circle_fit.ransac_iterations", 100);
  ros_node_->declare_parameter("circle_fit.ransac_inlier_threshold", 0.0015);
  ros_node_->declare_parameter("circle_fit.ransac_min_points", 13);

  CircleFitParams circle_params;
  circle_params.max_fit_error =
      ros_node_->get_parameter("circle_fit.max_fit_error").as_double();
  circle_params.min_inlier_ratio =
      ros_node_->get_parameter("circle_fit.min_inlier_ratio").as_double();
  circle_params.max_fit_error_near =
      ros_node_->get_parameter("circle_fit.max_fit_error_near").as_double();
  circle_params.max_fit_error_far =
      ros_node_->get_parameter("circle_fit.max_fit_error_far").as_double();
  circle_params.far_distance_threshold =
      ros_node_->get_parameter("circle_fit.far_distance_threshold").as_double();
  circle_params.min_radius =
      ros_node_->get_parameter("circle_fit.min_radius").as_double();
  circle_params.max_radius =
      ros_node_->get_parameter("circle_fit.max_radius").as_double();
  circle_params.max_concave_ratio =
      ros_node_->get_parameter("circle_fit.max_concave_ratio").as_double();
  circle_params.ransac_iterations =
      ros_node_->get_parameter("circle_fit.ransac_iterations").as_int();
  circle_params.ransac_inlier_threshold =
      ros_node_->get_parameter("circle_fit.ransac_inlier_threshold")
          .as_double();
  circle_params.ransac_min_points =
      ros_node_->get_parameter("circle_fit.ransac_min_points").as_int();

  reflector_detector_.configCircleFitter(circle_params);
}

void LifecycleOfflineReflectorNode::configureGlobalTracker() {
  ros_node_->declare_parameter("global_tracking.enable", true);
  ros_node_->declare_parameter("global_tracking.match_distance_threshold", 0.3);
  ros_node_->declare_parameter("global_tracking.match_distance_inactive", 0.5);
  ros_node_->declare_parameter("global_tracking.confirm_time_window", 1.0);
  ros_node_->declare_parameter("global_tracking.min_detections_in_window", 8);
  ros_node_->declare_parameter("global_tracking.inactive_timeout", 5.0);
  ros_node_->declare_parameter("global_tracking.max_inactive_time", 60.0);
  ros_node_->declare_parameter("global_tracking.position_filter_alpha", 0.3);
  ros_node_->declare_parameter("global_tracking.position_filter_beta", 0.2);
  ros_node_->declare_parameter("global_tracking.min_std_dev", 0.02);
  ros_node_->declare_parameter("global_tracking.max_std_dev", 0.5);
  ros_node_->declare_parameter("global_tracking.min_confidence_to_track", 0.3);
  ros_node_->declare_parameter("global_tracking.confidence_filter_alpha", 0.2);
  ros_node_->declare_parameter("global_tracking.diameter_filter_alpha", 0.3);

  bool use_global_tracker =
      ros_node_->get_parameter("global_tracking.enable").as_bool();

  GlobalReflectorTracker::Config tracking_config;
  tracking_config.match_distance_threshold =
      ros_node_->get_parameter("global_tracking.match_distance_threshold")
          .as_double();
  tracking_config.match_distance_inactive =
      ros_node_->get_parameter("global_tracking.match_distance_inactive")
          .as_double();
  tracking_config.confirm_time_window =
      ros_node_->get_parameter("global_tracking.confirm_time_window")
          .as_double();
  tracking_config.min_detections_in_window =
      ros_node_->get_parameter("global_tracking.min_detections_in_window")
          .as_int();
  tracking_config.inactive_timeout =
      ros_node_->get_parameter("global_tracking.inactive_timeout").as_double();
  tracking_config.max_inactive_time =
      ros_node_->get_parameter("global_tracking.max_inactive_time").as_double();
  tracking_config.position_filter_alpha =
      ros_node_->get_parameter("global_tracking.position_filter_alpha")
          .as_double();
  tracking_config.position_filter_beta =
      ros_node_->get_parameter("global_tracking.position_filter_beta")
          .as_double();
  tracking_config.min_std_dev =
      ros_node_->get_parameter("global_tracking.min_std_dev").as_double();
  tracking_config.max_std_dev =
      ros_node_->get_parameter("global_tracking.max_std_dev").as_double();
  tracking_config.min_confidence_to_track =
      ros_node_->get_parameter("global_tracking.min_confidence_to_track")
          .as_double();
  tracking_config.confidence_filter_alpha =
      ros_node_->get_parameter("global_tracking.confidence_filter_alpha")
          .as_double();
  tracking_config.diameter_filter_alpha =
      ros_node_->get_parameter("global_tracking.diameter_filter_alpha")
          .as_double();

  if (use_global_tracker) {
    RCLCPP_INFO(this->get_logger(),
                "#===反光柱检测器==== 开启全局反光柱跟踪器短时跟踪功能!!! ###");
    reflector_detector_.reset();
    reflector_detector_.configGlobalReflectorTracker(tracking_config);
  }
  use_short_tracker_ = use_global_tracker;
}

// ==================== LaserScan Processing ====================

void LifecycleOfflineReflectorNode::setLaserScan(
    int64_t timestamp, sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  if (scan_map_.find(timestamp) == scan_map_.end()) {
    scan_map_[timestamp] = scan_msg;
    scan_timestamps_.push_back(timestamp);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "scan消息: 存在重复时间戳");
  }
}

std::vector<TimePoint> LifecycleOfflineReflectorNode::convertScanToTimedPoints(
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

std::vector<Point> LifecycleOfflineReflectorNode::convertScanToPoints(
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

std::vector<Point> LifecycleOfflineReflectorNode::filterByIntensity(
    const std::vector<Point>& points, double threshold) {
  std::vector<Point> filtered;
  filtered.reserve(points.size());
  for (const auto& point : points) {
    if (point.intensity >= threshold) {
      filtered.push_back(point);
    }
  }
  return filtered;
}

// ==================== Landmark Conversion ====================

cartographer_ros_msgs::msg::LandmarkList::SharedPtr
LifecycleOfflineReflectorNode::convertTrackedReflectorsToLandmarkList(
    const std::vector<TrackedReflector>& tracked_reflectors, int64_t timestamp,
    const std::string& frame_id) {
  auto landmark_list =
      std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
  landmark_list->header.stamp = rclcpp::Time(timestamp);
  landmark_list->header.frame_id = frame_id;

  for (const auto& tracked : tracked_reflectors) {
    // Only add confirmed reflectors as landmarks
    if (tracked.state == TrackedReflector::TENTATIVE) {
      continue;
    }

    cartographer_ros_msgs::msg::LandmarkEntry entry;

    // Use global_id as landmark ID (must be string)
    entry.id = std::to_string(tracked.global_id);

    // Set position from filtered_position
    entry.tracking_from_landmark_transform.position.x =
        tracked.filtered_position.x;
    entry.tracking_from_landmark_transform.position.y =
        tracked.filtered_position.y;
    entry.tracking_from_landmark_transform.position.z = 0.0;  // 2D

    // Identity quaternion for rotation (point landmark)
    entry.tracking_from_landmark_transform.orientation.w = 1.0;
    entry.tracking_from_landmark_transform.orientation.x = 0.0;
    entry.tracking_from_landmark_transform.orientation.y = 0.0;
    entry.tracking_from_landmark_transform.orientation.z = 0.0;

    // Weights based on position uncertainty
    // Higher weight = lower uncertainty
    double weight = 1.0 / (tracked.position_std_dev + 1e-6);
    weight = std::min(weight, 100.0);  // Clamp to reasonable range

    entry.translation_weight = weight * landmark_translation_weight_;
    entry.rotation_weight = 0.0;  // Point landmarks don't constrain rotation

    landmark_list->landmarks.push_back(entry);
  }

  return landmark_list;
}

// ==================== Frame Processing ====================

bool LifecycleOfflineReflectorNode::loadAllFrames() {
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
    auto frame = std::make_shared<ReflectorFrameData>();
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
  LOG(INFO) << "激光雷达数据为:" << frames_.size() << "帧";
  // 补偿最后一帧激光雷达的odom数据虽然它可能只有一半
  if (odom_peek < odom_timestamps_.cend()) {
    for (std::vector<int64_t>::const_iterator scan_iterator =
             scan_timestamps_.cend() - 1;
         scan_iterator < scan_timestamps_.cend(); ++scan_iterator) {
      const auto& scan_time = *scan_iterator;
      auto frame = std::make_shared<ReflectorFrameData>();
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
    auto frame = std::make_shared<ReflectorFrameData>();
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

void LifecycleOfflineReflectorNode::processFrame(
    size_t frame_index, Node& node, int trajectory_id,
    const std::vector<TrajectoryOptions>& bag_trajectory_options) {
  if (frame_index >= frames_.size()) {
    RCLCPP_WARN(this->get_logger(), "帧索引超出范围: %zu/%zu", frame_index,
                frames_.size());
    return;
  }

  // Get base_link to laser transform
  if (!has_laser_to_base_) {
    // TODO: 从rosbag中获取laser的frame_id； 从node_options中获取track_frameId;
    try {
      int64_t firstscan_timestamp = frames_[frame_index]->timestamp;
      if (!has_laser_to_base_) {
        laser_to_base_ = tf_buffer_->lookupTransform(
            reflector_base_frame_, reflector_scan_frame_,
            rclcpp::Time(firstscan_timestamp), std::chrono::milliseconds(100));
        has_laser_to_base_ = true;
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Finish LaserToBase transform configure!"
                               << transforms::ToRigid3d(laser_to_base_));
      }
    } catch (tf2::TransformException& ex) {
      RCLCPP_FATAL_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "无法获取从%s到%s的变换: %s",
                            reflector_scan_frame_.c_str(),
                            reflector_base_frame_.c_str(), ex.what());
      has_laser_to_base_ = false;
      return;
    }
  }

  current_frame_index_ = frame_index;
  auto& frame = frames_[frame_index];

  RCLCPP_INFO(this->get_logger(), "处理帧 %zu/%zu, 时间: %.3f", frame_index,
              frames_.size() - 1, rclcpp::Time(frame->timestamp).seconds());

  if (frame->between_odoms.empty()) {
    return;
  }

  // 1. Distortion correction and global pose estimation
  auto scan_points = convertScanToTimedPoints(frame->scan);
  auto compensated_points = CublicDistortionCorrector::correctDistortion(
      scan_points, frame->between_odoms, transforms::ToRigid3d(laser_to_base_),
      frame->timestamp, frame->global_pose);

  if (!use_distort_corrector_) {
    frame->compensated_points = convertScanToPoints(frame->scan);
  } else {
    frame->compensated_points = compensated_points;
  }

  // Calculate global pose using globalpose_queue_ and odom
  auto globalposes = globalpose_queue_.popBefore(frame->timestamp);
  auto odompose = frame->between_odoms[frame->between_odoms.size() / 2];
  transforms::Rigid3d global_laser_pose;

  if (!map_odom_initialized_) {
    if (globalposes.empty()) {
      if (!globalpose_queue_.empty()) {
        auto globalpose = globalpose_queue_.front();
        global_laser_pose = globalpose.data;
        map_odom_ = global_laser_pose * odompose.data.inverse();
        map_odom_initialized_ = true;
        LOG(INFO) << "首次初始化map_odom_ (使用front): " << map_odom_;
      } else {
        LOG(ERROR) << "globalpose_queue_为空，无法初始化map_odom_";
        return;
      }
    } else {
      auto globalpose = globalposes.back();
      global_laser_pose =
          globalpose.data * transforms::ToRigid3d(laser_to_base_);
      map_odom_ = global_laser_pose * odompose.data.inverse();
      map_odom_initialized_ = true;
      LOG(INFO) << "首次初始化map_odom_ (使用popBefore.back): " << map_odom_;
    }
  } else {
    if (!globalposes.empty()) {
      auto globalpose = globalposes.back();
      global_laser_pose = globalpose.data;
      map_odom_ = global_laser_pose * odompose.data.inverse();
    } else {
      global_laser_pose = map_odom_ * odompose.data;
    }
  }

  auto odom_pose = frame->between_odoms.back().data;
  auto map_pose = map_odom_ * odom_pose;
  frame->global_pose = map_pose * transforms::ToRigid3d(laser_to_base_);

  // 2. Intensity filtering
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

void LifecycleOfflineReflectorNode::writeReflectorsToPbstream(
    cartographer_ros::Node& node) {
  // In processFrame() after reflector detection:
  auto tracked_reflectors =
      reflector_detector_.getCurrentAllTrackedReflectors();
  auto landmark_list = convertTrackedReflectorsToLandmarkList(
      tracked_reflectors, ros_node_->now().nanoseconds(),
      "map"  // 只能是全局坐标系
  );

  auto ros_mapbuilder_bridge = node.map_builder_bridge_;
  auto trajectories = ros_mapbuilder_bridge->GetTrajectoryStates();
  CHECK(trajectories.size() == 1);
  CHECK(trajectories.begin()->second ==
        ::cartographer::mapping::PoseGraphInterface::TrajectoryState::ACTIVE);
  LOG(WARNING) << "[✔] 全部跟踪到的反光柱, 加入Carotgrapher轨迹图结构.......";
  ros_mapbuilder_bridge->SetGlobalLandmarkList(landmark_list);
  LOG(WARNING)
      << "[✔] 完成反光柱加入轨迹并优化，结束轨迹任务，进行文件保存.......";
  // 保存地图名称
  std::vector<std::string> bag_filenames;
  const std::string state_output_filename = output_pbstream_path_.empty()
                                                ? "byd_amr.pbstream"
                                                : output_pbstream_path_;
  LOG(INFO) << "....正在保存pbstream地图文件: '" << state_output_filename
            << "'...";
  node.SerializeState(state_output_filename,
                      true /* include_unfinished_submaps */);
  // TODO: 保存SMAP
  while (!boost::filesystem::exists(state_output_filename)) {
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
  LOG(INFO) << "[✔] 生成地图切片submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution_);
  // 生成pgm和yaml,以及smap
  std::string map_filestem = cartographer_output_dir_ + "/" + map_filestem_;
  cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  cartographer::io::Image image(std::move(result.surface));

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution_,
      (result.origin.y() - image.height()) * resolution_);

  WritePgm(image, resolution_, &pgm_writer, origin, state_output_filename);

  cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution_, origin, pgm_writer.GetFilename(), &yaml_writer);
  LOG(INFO) << "[✔] 生成SMap地图.....";
  LOG(INFO) << "[✔] 完成carographer 离线反光柱建图流程!!.";
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineReflectorNode::on_activate(
    const rclcpp_lifecycle::State& state) {
  carto_executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  map_build_status_ = MapBuildStatus::STATUS_BUILDING;

  thread_ = std::make_unique<std::thread>([&]() {
    carto_executor_->add_node(ros_node_);
    cartographer_ros::NodeOptions node_options;
    // 从服务中获取rosbag包的路径和pbstream的输出路径,如果不存在无法建图！
    std::vector<std::string> bag_filenames;
    if (bag_file_path_.empty() || !boost::filesystem::exists(bag_file_path_) ||
        output_pbstream_path_.empty() ||
        default_configuration_basename_.empty()) {
      map_build_status_ = MapBuildStatus::STATUS_ERROR;
      return;
    }
    LOG(WARNING) << "bag_file_path_ : " << bag_file_path_;
    LOG(WARNING) << "output_pbstream_path_ : " << output_pbstream_path_;
    LOG(WARNING) << "建图默认配置文件 : " << default_configuration_basename_;
    bag_filenames.push_back(bag_file_path_);
    // TODO： 如果存储多个rosbag则可能存在指定的多个配置文件，
    // 这里暂时只考虑一个configure文件 std::regex regex(",");
    // std::vector<std::string> configuration_basenames(
    //   std::sregex_token_iterator(
    //     FLAGS_configuration_basenames.begin(),
    //     FLAGS_configuration_basenames.end(), regex, -1),
    //   std::sregex_token_iterator()
    // );
    std::vector<std::string> configuration_basenames;
    configuration_basenames.push_back(default_configuration_basename_);
    // 不同轨迹rosbag包可能存在不同的传感器配置和
    std::vector<TrajectoryOptions> bag_trajectory_options(1);
    // 从第一configure文件中加载与ros
    // node相关的配置，以及map_builder配置主要是PoseGraphOptions的配置
    // 所有轨迹的PoseGraphOptions配置必须相同，
    std::string configuration_directory =
        cartographer_shared_dir_ + "/configuration_files";
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
    // TODO: 单独解这个
    resolution_ =
        bag_trajectory_options.at(0)
            .trajectory_builder_options.trajectory_builder_2d_options()
            .submaps_options()
            .grid_options_2d()
            .resolution();

    // 由于我们预加载了变换缓冲区，因此我们永远不应该等待变换。
    // 当我们完成处理包时，我们将简单地丢弃任何由于缺少变换而无法转换的传感器数据。
    node_options.lookup_transform_timeout_sec = 0.;

    // 进行cartographer节点构建并进行逻辑处理, 创建cartographer的核心map_builder
    auto map_builder = map_builder_factory_(node_options.map_builder_options);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        ros_node_->get_clock(), tf2::durationFromSec(10), ros_node_);
    // 从urdf文件中读取 对应的static_transforms变化
    std::regex regex(",");
    std::vector<geometry_msgs::msg::TransformStamped> urdf_transforms;
    if (!FLAGS_use_bag_transforms && !urdf_path_.empty()) {
      std::vector<std::string> urdf_filenames;
      urdf_filenames.push_back(urdf_path_);
      for (const auto& urdf_filename : urdf_filenames) {
        LOG(INFO) << "加载URDF文件: " << urdf_filename;
        const auto current_urdf_transforms =
            ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer_);
        urdf_transforms.insert(urdf_transforms.end(),
                               current_urdf_transforms.begin(),
                               current_urdf_transforms.end());
      }
    } else if (!FLAGS_use_bag_transforms) {
      map_build_status_ = MapBuildStatus::STATUS_ERROR;
      LOG(WARNING) << "没有指定urdf文件，无法发布静态tf消息";
      return;
    }
    // 发布静态tf消息，从urdf文件中读取， 开始建图流程了
    // 开启 dedicated thread 用于 tf
    // 变换，指为TF变换处理单独分配一个后台线程，与主线程分离运行
    map_build_status_ = MapBuildStatus::STATUS_BUILDING;
    tf_buffer_->setUsingDedicatedThread(true);
    // -----------------------------------------------------------------------
    const std::chrono::time_point<std::chrono::steady_clock> start_time =
        std::chrono::steady_clock::now();
    // 创建cartographer节点，进行建图工作
    Node node(node_options, std::move(map_builder), tf_buffer_, ros_node_,
              false);
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
    // 如果只有一个bag包，那么还是会以 {laserscan, odom, imu}
    // 的名称创建sensor_id 例如: [{bag_1_laserscan1, bag_1_odom1};
    // {bag_2_laserscan2, bag_2_odom2}];
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
         ++current_bag_index) {
      const std::string& bag_filename = bag_filenames.at(current_bag_index);
      if (!rclcpp::ok()) {
        return;
      }
      // 将bag对应的cartographer总的topic_id进行对应，既将不同bag的{sensor_id}放到
      // 一一列举出来， (<bag_index, sensor_id_string>, sensor_id);
      // 因为cartographer_node->AddOfflineTrajectory()需要传入sensor_id_set
      // 所以不同的包创建不同的Trajectory需要明确sensor_id的集合;
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

      auto serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
      playable_bag_multiplexer.AddPlayableBag(PlayableBag(
          bag_filename, current_bag_index, kDelay,
          [&tf_publisher, this, serializer](
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
    for (const auto& topic : playable_bag_multiplexer.topics()) {
      std::string resolved_topic =
          ros_node_->get_node_base_interface()->resolve_topic_or_service_name(
              topic, false);
      bag_topics.insert(resolved_topic);
      bag_topics_string << resolved_topic << ",";
    }
    bool print_topics = false;
    // TODO：
    // 多个bag包且有多个配置时，必须指定包与sensor_id映射关系，否则会出现警告；这里因为包内topic可能有重复，
    // 所以需要指定映射指定后， 指定的映射关系bag_topics 中基本不会包含
    // {bag_1_laserscan1} 这样的sensor_id;
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
    auto pcl2_serializer =
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
    auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
    auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
    auto nav_sat_fix_serializer =
        rclcpp::Serialization<sensor_msgs::msg::NavSatFix>();
    auto landmark_list_serializer =
        rclcpp::Serialization<cartographer_ros_msgs::msg::LandmarkList>();
    // 反光柱Laserscan处理方法，当前只支持单个传感器的Laserscan
    // TODO：加入判断进行建图取消响应操作 配置 STATUS_CANCEL
    while (playable_bag_multiplexer.IsMessageAvailable() && enable_mapping_) {
      if (!::rclcpp::ok()) {
        LOG(FATAL) << "current rclcpp shutdown.";
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

      // 对于多个bag包，只有作了bag_1_scan_1 ->
      // scan这样的remapping，才能找到对应的sensor_id
      // 当bag_topic_to_sensor_id
      const auto bag_topic = std::make_pair(
          bag_index,
          ros_node_->get_node_base_interface()->resolve_topic_or_service_name(
              msg.topic_name, false));
      auto it = bag_topic_to_sensor_id.find(bag_topic);
      // 找到对应传感器类型进行处理建图
      if (it != bag_topic_to_sensor_id.end()) {
        const std::string& sensor_id = it->second.id;
        if (topic_type == "sensor_msgs/msg/LaserScan") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg =
              std::make_shared<sensor_msgs::msg::LaserScan>();
          laser_scan_serializer.deserialize_message(&serialized_msg,
                                                    laser_scan_msg.get());
          // 2. 激光雷达扫描处理
          node.HandleLaserScanMessage(trajectory_id, sensor_id, laser_scan_msg);
          // 1. NOTICE:
          // 获取当前laserscan后，匹配的tracked_pose；可能有多个包，但仍然是单车单雷达可通过获取当前对应
          // 轨迹的全局tracked_pose，来更新assigner中的位姿，维持数据，要确保assigner中的位姿是最新的，
          // 也就是说多个包的情况也是按时间顺序发布的。
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
          node.HandleMultiEchoLaserScanMessage(trajectory_id, sensor_id,
                                               multi_echo_laser_scan_msg);
        } else if (topic_type == "sensor_msgs/msg/PointCloud2") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::PointCloud2::SharedPtr pcl2_scan_msg =
              std::make_shared<sensor_msgs::msg::PointCloud2>();
          pcl2_serializer.deserialize_message(&serialized_msg,
                                              pcl2_scan_msg.get());
          node.HandlePointCloud2Message(trajectory_id, sensor_id,
                                        pcl2_scan_msg);
        } else if (topic_type == "sensor_msgs/msg/Imu") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          sensor_msgs::msg::Imu::SharedPtr imu_scan_msg =
              std::make_shared<sensor_msgs::msg::Imu>();
          imu_serializer.deserialize_message(&serialized_msg,
                                             imu_scan_msg.get());
          node.HandleImuMessage(trajectory_id, sensor_id, imu_scan_msg);
        } else if (topic_type == "nav_msgs/msg/Odometry") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          nav_msgs::msg::Odometry::SharedPtr odom_scan_msg =
              std::make_shared<nav_msgs::msg::Odometry>();
          odom_serializer.deserialize_message(&serialized_msg,
                                              odom_scan_msg.get());
          node.HandleOdometryMessage(trajectory_id, sensor_id, odom_scan_msg);
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
          node.HandleNavSatFixMessage(trajectory_id, sensor_id,
                                      nav_sat_fix_msg);
        } else if (topic_type == "cartographer_ros_msgs/msg/LandmarkList") {
          rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
          cartographer_ros_msgs::msg::LandmarkList::SharedPtr
              landmark_list_msg =
                  std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
          landmark_list_serializer.deserialize_message(&serialized_msg,
                                                       landmark_list_msg.get());
          node.HandleLandmarkMessage(trajectory_id, sensor_id,
                                     landmark_list_msg);
        }
      }
      // PlayableBagMultiplexer
      // 时按顺序进行排序的，所以这里使用其对应的msg时间作为clock时间发布
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
    // 确保的FinalOptimization时，还有/clock消息发布，默认30hz；
    // 但这个时间是最后一个msg时间，
    // 所以这里需要手动发布一次，确保最后一个msg时间的clock时间也发布出去。
    auto clock_republish_timer = ros_node_->create_wall_timer(
        std::chrono::milliseconds(int(kClockPublishFrequencySec)),
        [&clock_publisher, &clock]() { clock_publisher->publish(clock); });
    node.RunFinalOptimization();
    // 统计建图时间！！
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
    // Serialize unless we have neither a bagfile nor an explicit state
    // filename.
    if (rclcpp::ok() &&
        !(bag_filenames.empty() && output_pbstream_path_.empty()) &&
        enable_mapping_) {
      LOG(INFO) << "完成Cartographer离线建图..... 准备进反光柱检测回溯......";
      // 提取优化后轨迹
      auto ros_mapbuilder_bridge = node.map_builder_bridge_;
      cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr
          poses_requeset = std::make_shared<
              cartographer_ros_msgs::srv::TrajectoryQuery::Request>();
      poses_requeset->trajectory_id = 0;
      cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr
          poses_res_ptr = std::make_shared<
              cartographer_ros_msgs::srv::TrajectoryQuery::Response>();
      ros_mapbuilder_bridge->HandleTrajectoryQuery(poses_requeset,
                                                   poses_res_ptr);

      int odom_indx = 0;
      for (const auto& geo_msg : poses_res_ptr->trajectory) {
        int64_t timestamp = rclcpp::Time(geo_msg.header.stamp).nanoseconds();
        // Add odometry to the time-ordered queue
        globalpose_queue_.push(
            TimeRigid3d(transforms::ToRigid3d(geo_msg.pose), timestamp));
      }

      if (!loadAllFrames()) {
        LOG(FATAL) << "mark==  回溯激光里程失败";
        exit(-1);
      }
      RCLCPP_INFO(this->get_logger(), "加载完成，共 %zu 帧", frames_.size());
      if (frames_.size() < 2) {
        LOG(FATAL) << "mark==  统计激光里程帧数小于2";
      }
      while (true) {
        if (current_frame_index_ < frames_.size() - 2) {
          processFrame(current_frame_index_, node, 0, bag_trajectory_options);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          current_frame_index_++;
        } else {
          RCLCPP_INFO(this->get_logger(), "自动处理完成");
          auto tracked_reflectors =
              reflector_detector_.getCurrentAllTrackedReflectors();
          auto landmark_list = convertTrackedReflectorsToLandmarkList(
              tracked_reflectors, ros_node_->now().nanoseconds(),
              "map"  // 只能是全局坐标系
          );
          auto trajectories = ros_mapbuilder_bridge->GetTrajectoryStates();
          CHECK(trajectories.size() == 1);
          LOG(WARNING)
              << "[✔] 全部跟踪到的反光柱, 加入Carotgrapher轨迹图结构.......";
          LOG(WARNING) << "[✔] 全部跟踪到的反光柱, 反光柱个数:"
                       << tracked_reflectors.size() << " 带有ID的反光柱个数:"
                       << landmark_list->landmarks.size();
          ros_mapbuilder_bridge->SetGlobalLandmarkList(landmark_list);
          LOG(WARNING)
              << "[✔] "
                 "完成反光柱加入轨迹并优化，结束轨迹任务，进行文件保存.......";
          break;
        }
      }

      const std::string state_output_filename =
          output_pbstream_path_.empty() ? bag_filenames.front() + ".pbstream"
                                        : output_pbstream_path_;
      LOG(INFO) << "....正在保存pbstream地图文件: '" << state_output_filename
                << "'...";
      node.SerializeState(state_output_filename,
                          true /* include_unfinished_submaps */);
      // TODO: 保存SMAP
      while (!boost::filesystem::exists(output_pbstream_path_)) {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }

      // 读取pbsteam转换成smap
      cartographer::io::ProtoStreamReader reader(state_output_filename);
      cartographer::io::ProtoStreamDeserializer deserializer(&reader);
      LOG(INFO) << "二次加载pbstream地图.......";
      std::map<::cartographer::mapping::SubmapId,
               ::cartographer::io::SubmapSlice>
          submap_slices;
      cartographer::mapping::ValueConversionTables conversion_tables;
      cartographer::io::DeserializeAndFillSubmapSlices(
          &deserializer, &submap_slices, &conversion_tables);
      CHECK(reader.eof());
      LOG(INFO) << "生成地图切片submap slices.";
      auto result =
          ::cartographer::io::PaintSubmapSlices(submap_slices, resolution_);
      // 生成pgm和yaml,以及smap
      std::string map_filestem = cartographer_output_dir_ + "/" + map_filestem_;
      cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

      cartographer::io::Image image(std::move(result.surface));

      const Eigen::Vector2d origin(
          -result.origin.x() * resolution_,
          (result.origin.y() - image.height()) * resolution_);

      WritePgm(image, resolution_, &pgm_writer, origin, state_output_filename);

      cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
      WriteYaml(resolution_, origin, map_filestem_ + ".pgm", &yaml_writer);

      LOG(INFO) << "完成保存地图文件: '" << state_output_filename << "'...";
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
    std::thread([&]() {
      // 等待当前节点进入 INACTIVE 状态（确保 deactivate 完成）
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      // 3. 发送 cleanup 转换请求（INACTIVE → Unconfigured）
      auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
          "/lifecycle_cartographer_reflector_node/change_state");
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "等待 /change_state 服务可用...");
      }

      auto request =
          std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      request->transition.id =
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;  // cleanup
                                                                   // 转换ID

      auto result = client->async_send_request(request).get();
      if (result->success) {
        RCLCPP_INFO(get_logger(),
                    "deactivate 转换触发成功 (ACTIVE → INACTIVE )");
      } else {
        RCLCPP_ERROR(get_logger(), "deactivate 转换触发失败");
      }
    }).detach();  // 异步执行，不阻塞当前回调
  });

  LOG(INFO) << "[✔] cartographer_ros反光柱建图节点启动完成!";
  process_pub_->on_activate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineReflectorNode::on_deactivate(
    const rclcpp_lifecycle::State& state) {
  LOG(INFO) << "DEACTIVATING 从 ACTIVATE -> INACTIVAE 状态转换";
  thread_->join();
  LOG(INFO) << "thread_join() 完成，cartographer安全退出";
  carto_executor_->cancel();
  LOG(INFO) << "carto_exector取消spin()";

  if (!enable_mapping_) {
    enable_mapping_ = true;
  }

  if (map_build_status_ == MapBuildStatus::STATUS_BUILDING) {
    LOG(WARNING)
        << "建图节点状态存在问题请检查代码，此时建图已经完成仍然为building状态";
  } else if (map_build_status_ == MapBuildStatus::STATUS_CANCEL) {
    LOG(WARNING) << "建图任务已经取消，进行资源重置";
  } else if (map_build_status_ == MapBuildStatus::STATUS_COMPLETE) {
    LOG(WARNING) << "建图任务已经完成并保存pbstream地图资源，进行资源重置";
  } else if (map_build_status_ == MapBuildStatus::STATUS_ERROR) {
    LOG(WARNING) << "建图过程中出现错误，请检查on_activate状态代码";
  }

  LOG(INFO)
      << "[✔] 完成建图过程，停止cartographer节点，并切换到unconfigure状态.....";

  std::thread([&]() {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/lifecycle_cartographer_reflector_node/change_state");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "等待 /change_state 服务可用...");
    }

    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id =
        lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

    auto result = client->async_send_request(request).get();
    if (result->success) {
      RCLCPP_INFO(get_logger(),
                  "cleanup 转换触发成功（INACTIVE → Unconfigured）");
    } else {
      RCLCPP_ERROR(get_logger(), "cleanup 转换触发失败");
    }
  }).detach();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineReflectorNode::on_cleanup(const rclcpp_lifecycle::State&) {
  thread_.reset();
  ros_node_.reset();
  // cartographer_ros清理
  output_pbstream_path_ = "";
  default_configuration_basename_ = "";

  // Clear reflector detection data
  reflector_param_file_ = "";
  frames_.clear();
  scan_map_.clear();
  scan_timestamps_.clear();
  odom_timestamps_.clear();
  odom_queue_ = TimeOrderQueue<transforms::Rigid3d>();
  globalpose_queue_ = TimeOrderQueue<transforms::Rigid3d>();
  has_laser_to_base_ = false;
  map_odom_initialized_ = false;

  LOG(INFO)
      << "[✔] cartographer_ros反光柱建图节点清理完成! 进入unconfigured状态";
  map_build_status_ = MapBuildStatus::STATUS_IDEL;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleOfflineReflectorNode::on_shutdown(const rclcpp_lifecycle::State&) {
  LOG(WARNING) << "不允许生命周期节点shutdown不然无法恢复";
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::FAILURE;
}

}  // namespace cartographer_ros