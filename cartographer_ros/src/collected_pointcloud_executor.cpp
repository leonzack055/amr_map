/*
 * Copyright 2024 The Cartographer Authors
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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer_ros/collected_pointcloud.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"

using namespace cv;

DEFINE_string(pbstream_path, "", "Path to pbstream file");
DEFINE_string(output_dir, "./line_features_output",
              "Output directory for results");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_bool(verbose, false, "Enable verbose logging");

using namespace cartographer::mapping;

void Run(const std::string& pbstream_filename, const double resolution) {
  // load options
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{FLAGS_configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));
  proto::TrajectoryBuilderOptions trajectory_builder_options =
      ::cartographer::mapping::CreateTrajectoryBuilderOptions(
          lua_parameter_dictionary.GetDictionary("trajectory_builder").get());
  proto::MapBuilderOptions map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary.GetDictionary("map_builder").get());
  // load map
  auto map_builder =
    cartographer::mapping::CreateMapBuilder(map_builder_options);
  map_builder->LoadStateFromFile(FLAGS_pbstream_path, false);

  // get PoseGraph
  auto posegraph_ptr = map_builder->pose_graph();

  // 1. create pointcloud executor
  auto pointcloud_executor = std::make_shared<cartographer_ros::CollectedPointCloudFrom2d>();
  bool ret = pointcloud_executor->collectPointCloud(posegraph_ptr);
  // 2. filter with 0.02
  ret = pointcloud_executor->filterPointCloud(0.02);
  ret = pointcloud_executor->savePointCloud(FLAGS_output_dir + "/pbstream.pcd");
  if(ret) {
    LOG(INFO) << "完成点云提取， 保存到" << FLAGS_output_dir + "/pbstream.pcd";
  }
}

int main(int argc, char** argv) {
  // 初始化ROS2
  rclcpp::init(argc, argv);
  // 初始化Google Flags和Logging
  google::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // 创建节点
  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto node = rclcpp::Node::make_shared("pointcloud_executor_simple");

  RCLCPP_INFO(node->get_logger(), "Pointcloud Executor Simple Version");
  RCLCPP_INFO(node->get_logger(), "pbstream_path: %s",
              FLAGS_pbstream_path.c_str());
  RCLCPP_INFO(node->get_logger(), "output_dir: %s", FLAGS_output_dir.c_str());
  RCLCPP_INFO(node->get_logger(), "verbose: %s",
              FLAGS_verbose ? "true" : "false");

  if (FLAGS_pbstream_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "pbstream_path is required");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Line feature extraction completed successfully!");
  Run(FLAGS_pbstream_path.c_str(), 0.05);
  // 关闭ROS2
  rclcpp::shutdown();

  return 0;
}