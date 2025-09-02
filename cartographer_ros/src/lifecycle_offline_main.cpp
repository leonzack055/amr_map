/*
 * Copyright 2025 LEON
 */

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/lifecycle_offline_node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  const cartographer_ros::MapBuilderFactory map_builder_factory = [](
      const ::cartographer::mapping::proto::MapBuilderOptions&
          map_builder_options) {
    return ::cartographer::mapping::CreateMapBuilder(map_builder_options);
  };

  rclcpp::executors::SingleThreadedExecutor::SharedPtr exe = rclcpp::executors::SingleThreadedExecutor::make_shared();

  std::shared_ptr<cartographer_ros::LifecycleOfflineCartoNode> cartographer_offline_node = 
  std::make_shared<cartographer_ros::LifecycleOfflineCartoNode>(
      "lifecycle_cartographer_node", false, exe, map_builder_factory);

  // cartographer_offline_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  exe->add_node(cartographer_offline_node->get_node_base_interface());
  exe->spin();

  rclcpp::shutdown();
}
