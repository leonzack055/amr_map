#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LIFECYCLE_OFFLINE_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LIFECYCLE_OFFLINE_NODE_H

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros_msgs/msg/bagfile_progress.hpp"

#include "byd_mapbuilder_msgs/msg/map_build_status.hpp"
#include "byd_mapbuilder_msgs/msg/map_build_process.hpp"
#include "byd_mapbuilder_msgs/srv/map_build.hpp"

using namespace std::chrono_literals;

namespace cartographer_ros
{

using MapBuilderFactory =
  std::function<std::unique_ptr<::cartographer::mapping::MapBuilderInterface>(
      const ::cartographer::mapping::proto::MapBuilderOptions &)>;

class LifecycleOfflineCartoNode : public rclcpp_lifecycle::LifecycleNode
{
  enum MapBuildStatus
  {
    STATUS_IDEL = 0,
    STATUS_BUILDING,
    STATUS_COMPLETE,
    STATUS_CANCEL,
    STATUS_ERROR,
  };

public:
  explicit LifecycleOfflineCartoNode(
    const std::string & node_name, bool intra_process_comms = false,
    rclcpp::Executor::SharedPtr executor = nullptr,
    const MapBuilderFactory & map_builder_factory = nullptr
  );
  // void publishProcessData(); // 发布处理数据包的进程
  
  /// @brief 配置回调函数
  /**
   * on_configure 回调当lifecycle节点进入 "configuring" 状态时调用。
   * 依赖于当前调用是否成功，如果成功则进入"inactive" 状态， 创建配置文件，cartographer_node节点和rosbagMultiplayer节点
   * 并进行默认参数的配置，输出pbstream文件名，以及指定的rosbag包；
   * 否则不成功，进入"unconfigured" 状态， 清除cartographer_node节点和rosbagMultiplayer节点
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  /// @brief 激活回调函数
  /**
   * on_activate 回调当lifecycle节点从"unconfigured" 进入 "activating" 状态时调用。
   * 依赖于当前调用是否成功，如果成功则进入"active" 状态， 创建一个异步调用thread线程，来触发rosbagMultiplayer进行工作。
   * 否则不成功，保持"inactive" 状态， 清除thread线程。
   * TRANSITION_CALLBACK_SUCCESS transitions to "active"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  // 建图过程完成，取消建图，或者建图失败，都需要进入deactivate状态； 此时建图节点资源依然存在
  // 重新建图请求，需要跳转到TRANSITION_CLEANUP状态, 然后再去配置ACTIVATE状态才能完成建图；
  // 此处先回收线程，然后根据建图是否被取消，来重置enbale_mapping_建图默认启动标志位，然后跳转
  // 到UNCONFIGURE状态
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;


  /// @brief 清理回调函数
  /**
   * on_cleanup 回调 当lifecycle节点从"inactive" 进入 "cleaningup" 状态时调用。
   * 依赖于当前调用是否成功，如果成功则进入"unconfigured" 状态， 清除cartographer_node节点和rosbagMultiplayer节点
   * 否则不成功，保持"inactive" 状态。
   * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  /// @brief 关闭回调函数
  /**
   * on_shutdown 回调函数 进入"shuttingdown" 状态.
   * 依赖返回结果，如果成功进入"finalized"，后面任何调用都无法进行相关操作，无法完成状态切换。
   * 如果失败，则保持当前状态。
   * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
   * TRANSITION_CALLBACK_FAILURE transitions to current state
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

private:
  // 创建配置文件
  bool create_options();

  // We hold an instance of a lifecycle publisher. This lifecycle publisher
  // can be activated or deactivated regarding on which state the lifecycle node
  // is in.
  // By default, a lifecycle publisher is inactive by creation and has to be
  // activated to publish messages into the ROS world.
  rclcpp_lifecycle::LifecyclePublisher<byd_mapbuilder_msgs::msg::MapBuildProcess>::SharedPtr process_pub_;
  rclcpp::Subscription<cartographer_ros_msgs::msg::BagfileProgress>::SharedPtr bag_process_sub_;
  void bagProcessDataCallback(const cartographer_ros_msgs::msg::BagfileProgress::SharedPtr msg); // 监听RosBagProcess消息

//   rclcpp_lifecycle::LifecycleNode<d_msgs::msg::String>::SharedPtr sub_process_data_;

  rclcpp::Node::SharedPtr ros_node_;
  // 必须将ros_node_装载到executor_中才能监听对应节点启动的消息
  rclcpp::Executor::SharedPtr executor_;

  // 通过服务获取rosbag包的路径
  std::string bag_file_path_;
  std::string output_pbstream_path_;
  std::string urdf_path_;
  std::string cartographer_install_dir_;
  std::string cartographer_shared_dir_;
  std::string default_configuration_basename_;
  MapBuilderFactory map_builder_factory_;
  // 建图逻辑线程
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr carto_executor_;
  // 建图节点的工作状态
  MapBuildStatus map_build_status_;
  bool enable_mapping_;
  // 服务端
  rclcpp::Service<byd_mapbuilder_msgs::srv::MapBuild>::SharedPtr map_build_srv_;
  // 回调函数
  void map_build_callback(
    const std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Request> request,
    std::shared_ptr<byd_mapbuilder_msgs::srv::MapBuild::Response> response);
  float resolution_;
  std::string map_filestem_;
};


}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LIFECYCLE_OFFLINE_NODE_H
