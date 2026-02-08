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

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/cv_lsd_detector.h"

#include <rclcpp/rclcpp.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#include "cartographer_ros/line_feature.h"
#include "cartographer_ros/line_feature_utils.h"
#include "cartographer_ros/line_feature_extractor.h"

DEFINE_string(pbstream_path, "", "Path to pbstream file");
DEFINE_string(output_dir, "./line_features_output", "Output directory for results");
DEFINE_string(preset, "balanced", "Preset configuration (fast/balanced/accurate/indoor/outdoor)");
DEFINE_bool(verbose, false, "Enable verbose logging");

void Run(const std::string& pbstream_filename, const double resolution)
{
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;
  ::cartographer::io::DeserializeAndFillSubmapSlices(&deserializer, &submap_slices, &conversion_tables);
  CHECK(reader.eof());

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::Image image(std::move(result.surface));
  // 创建cv::Mat对象
  cv::Mat gray_Mat(image.height(), image.width(), CV_8UC1);
  for (int y = 0; y < image.height(); ++y)
  {
    for (int x = 0; x < image.width(); ++x)
    {
      const char color = image.GetPixel(x, y)[0];
      gray_Mat.at<uchar>(y, x) = static_cast<uchar>(color);  // 将char类型转换为uchar类型
    }
  }
  cv::imshow("testMap2", gray_Mat);
  cv::waitKey(0);
  cv::Mat originMat;
  cv::cvtColor(gray_Mat, originMat, cv::COLOR_GRAY2BGR);
  //   cv::imwrite("testMap2.png", originMat);

  // 计算地图的原点，这里假设地图的原点在左上角； 左上角的世界坐标，
  // 这里是以 x 向上； y 向左 为世界坐标； z 向屏幕，满足右手定则
  /*
        x ^
          |
          |
  y <-----|
  */
  const Eigen::Vector2d origin(-result.origin.x() * resolution, (result.origin.y() - image.height()) * resolution);
  // 将image转换成cv::Mat 然后提取直线
  // 直线特征提取
  std::vector<cartographer_ros::LineFeature> line_features;
  // 创建直线特征提取器配置
  cartographer_ros::LineExtractionConfig config;
  config.enable_line_extraction = true;
  config.enable_adaptive_params = true;
  config.enable_debug_output = true;

  // 预处理配置
  config.preprocessing.use_otsu_threshold = true;
  config.preprocessing.manual_threshold = 86.0;  // 0.0 表示自动计算阈值
  config.preprocessing.enable_canny = true;
  config.preprocessing.canny_low_threshold = 50.0;
  config.preprocessing.canny_high_threshold = 150.0;
  config.preprocessing.enable_debug_output = true;
  config.preprocessing.enable_morphology = false;

  // 霍夫变换配置
  config.hough_transform.enabled = false;
  config.hough_transform.threshold = 48;
  config.hough_transform.min_line_length_meters = 0.5;
  config.hough_transform.confidence_threshold = 0.7;

  // LSD配置
  config.lsd.enabled = true;
  config.lsd.density_th = 0.7;
  config.lsd.min_line_length_meters = 0.3;
  config.lsd.confidence_threshold = 0.6;

  // 输出配置
  config.output.output_json = true;
  config.output.output_csv = true;
  config.output.output_visualization = true;
  config.output.output_directory = "./line_features/";
  config.output.base_filename = pbstream_filename.substr(0, pbstream_filename.find_last_of('.')) + "_lines";

  // 创建提取器并提取直线特征
  cartographer_ros::LineFeatureExtractor extractor(config);
  line_features = extractor.ExtractLineFeatures(gray_Mat, resolution, origin);
  LOG(INFO) << "直线特征提取完成，共提取 " << line_features.size() << " 条直线";

  // LSD 图像预处理
  cv::Mat bin_Mat;
  double otsu_threshold = config.preprocessing.manual_threshold;
  LOG(INFO) << "Otsu阈值: " << otsu_threshold;
  threshold(gray_Mat, bin_Mat, otsu_threshold, 255, THRESH_BINARY);
  cartographer_ros::CvLsdDetector lsd_detector(config.lsd);
  std::vector<cartographer_ros::LineFeature> lsd_lines;
  lsd_lines = lsd_detector.DetectLines(bin_Mat, resolution);

  // 将直线画在image上面，根据不同的类别设定不同的颜色
  // 将直线画在image上面，根据不同的类别设定不同的颜色
  for (const auto& line : line_features)
  {
    // 根据线段类型设置颜色
    cv::Scalar line_color;
    int line_thickness = 2;

    switch (line.type)
    {
      case cartographer_ros::LineType::WALL:
        line_color = cv::Scalar(0, 0, 255);  // 红色 - 墙壁
        line_thickness = 3;
        break;
      case cartographer_ros::LineType::CORRIDOR:
        line_color = cv::Scalar(0, 255, 0);  // 绿色 - 走廊
        line_thickness = 2;
        break;
      case cartographer_ros::LineType::OBSTACLE:
        line_color = cv::Scalar(0, 255, 255);  // 黄色 - 障碍物边界
        line_thickness = 2;
        break;
      case cartographer_ros::LineType::STRUCTURE:
        line_color = cv::Scalar(255, 0, 0);  // 蓝色 - 结构线
        line_thickness = 0;
        break;
      default:
        line_color = cv::Scalar(128, 128, 128);  // 灰色 - 未知类型
        line_thickness = 1;
        break;
    }

    // 将世界坐标转换为图像坐标
    // 图像坐标 x = (世界坐标 x - origin.x) / resolution
    // 图像坐标 y = image.height() - (世界坐标 y - origin.y) / resolution
    int x1 = static_cast<int>((line.start_point.x()));
    int y1 = static_cast<int>((line.start_point.y()));
    int x2 = static_cast<int>((line.end_point.x()));
    int y2 = static_cast<int>((line.end_point.y()));

    // 绘制直线
    cv::line(originMat, cv::Point(x1, y1), cv::Point(x2, y2), line_color, line_thickness);

    // 可选：绘制端点
    cv::circle(originMat, cv::Point(x1, y1), 3, line_color, -1);
    cv::circle(originMat, cv::Point(x2, y2), 3, line_color, -1);

    // 可选：在直线中心标注线段ID或类型
    int cx = (x1 + x2) / 2;
    int cy = (y1 + y2) / 2;
    std::string label = std::to_string(static_cast<int>(line.type));
    cv::putText(originMat, label, cv::Point(cx + 5, cy - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, line_color, 1);
  }

  for (const auto& line : lsd_lines)
  {
    // 根据线段类型设置颜色
    cv::Scalar line_color;
    int line_thickness = 2;

    switch (line.type)
    {
      case cartographer_ros::LineType::WALL:
        line_color = cv::Scalar(0, 0, 255);  // 红色 - 墙壁
        line_thickness = 3;
        break;
      case cartographer_ros::LineType::CORRIDOR:
        line_color = cv::Scalar(0, 255, 0);  // 绿色 - 走廊
        line_thickness = 2;
        break;
      case cartographer_ros::LineType::OBSTACLE:
        line_color = cv::Scalar(0, 255, 255);  // 黄色 - 障碍物边界
        line_thickness = 2;
        break;
      case cartographer_ros::LineType::STRUCTURE:
        line_color = cv::Scalar(255, 0, 0);  // 蓝色 - 结构线
        line_thickness = 2;
        break;
      default:
        line_color = cv::Scalar(128, 128, 128);  // 灰色 - 未知类型
        line_thickness = 1;
        break;
    }

    // 将世界坐标转换为图像坐标
    // 图像坐标 x = (世界坐标 x - origin.x) / resolution
    // 图像坐标 y = image.height() - (世界坐标 y - origin.y) / resolution
    int x1 = static_cast<int>(line.start_point.x());
    int y1 = static_cast<int>(line.start_point.y());
    int x2 = static_cast<int>(line.end_point.x());
    int y2 = static_cast<int>(line.end_point.y());

    // 绘制直线
    cv::line(originMat, cv::Point(x1, y1), cv::Point(x2, y2), line_color, line_thickness);

    // 可选：绘制端点
    cv::circle(originMat, cv::Point(x1, y1), 3, line_color, -1);
    cv::circle(originMat, cv::Point(x2, y2), 3, line_color, -1);

    // 可选：在直线中心标注线段ID或类型
    int cx = (x1 + x2) / 2;
    int cy = (y1 + y2) / 2;
    std::string label = std::to_string(static_cast<int>(line.type));
    cv::putText(originMat, label, cv::Point(cx + 5, cy - 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, line_color, 1);
  }

  // 保存带有直线标注的图像
  std::string output_path = FLAGS_output_dir + "/map_with_lines.png";
  imwrite(output_path, originMat);
  LOG(INFO) << "已保存带直线标注的地图图像: " << output_path;

  // 可选：显示图像
  imshow("Map with Line Features", originMat);
  waitKey(0);
}

int main(int argc, char** argv)
{
  // 初始化ROS2
  rclcpp::init(argc, argv);
  // 初始化Google Flags和Logging
  google::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // 创建节点
  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto node = rclcpp::Node::make_shared("line_feature_extractor_simple");

  RCLCPP_INFO(node->get_logger(), "Line Feature Extractor Simple Version");
  RCLCPP_INFO(node->get_logger(), "pbstream_path: %s", FLAGS_pbstream_path.c_str());
  RCLCPP_INFO(node->get_logger(), "output_dir: %s", FLAGS_output_dir.c_str());
  RCLCPP_INFO(node->get_logger(), "preset: %s", FLAGS_preset.c_str());
  RCLCPP_INFO(node->get_logger(), "verbose: %s", FLAGS_verbose ? "true" : "false");

  if (FLAGS_pbstream_path.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "pbstream_path is required");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Line feature extraction completed successfully!");
  Run(FLAGS_pbstream_path.c_str(), 0.05);
  // 关闭ROS2
  rclcpp::shutdown();

  return 0;
}