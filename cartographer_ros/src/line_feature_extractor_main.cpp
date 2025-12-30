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

#include "cartographer_ros/line_feature_extractor.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include <rclcpp/rclcpp.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iostream>
#include <chrono>

DEFINE_string(pbstream_path, "", "Path to the pbstream file");
DEFINE_string(config_file, "line_extraction_config.lua", "Path to the configuration file");
DEFINE_string(output_dir, "./line_features_output", "Output directory for results");
DEFINE_string(preset, "balanced", "Preset configuration (fast/balanced/accurate/indoor/outdoor)");
DEFINE_bool(enable_visualization, true, "Enable visualization output");
DEFINE_bool(verbose, false, "Enable verbose logging");

namespace cartographer_ros {

class LineFeatureExtractorNode : public rclcpp::Node 
{
public:
    LineFeatureExtractorNode(): Node("line_feature_extractor") {}
    bool extract_features() {
        // try {
        //     // 验证输入参数
        //     if (FLAGS_pbstream_path.empty()) {
        //         RCLCPP_ERROR(get_logger(), "pbstream_path is required");
        //         return false;
        //     }

        //     if (!std::filesystem::exists(FLAGS_pbstream_path)) {
        //         RCLCPP_ERROR(get_logger(), "pbstream file does not exist: %s", FLAGS_pbstream_path.c_str());
        //         return false;
        //     }

        //     // 创建输出目录
        //     std::filesystem::create_directories(FLAGS_output_dir);

        //     RCLCPP_INFO(get_logger(), "Starting line feature extraction");
        //     RCLCPP_INFO(get_logger(), "Input: %s", FLAGS_pbstream_path.c_str());
        //     RCLCPP_INFO(get_logger(), "Config: %s", FLAGS_config_file.c_str());
        //     RCLCPP_INFO(get_logger(), "Output: %s", FLAGS_output_dir.c_str());

        //     // 加载pbstream文件
        //     cartographer::io::ProtoStreamReader reader(FLAGS_pbstream_path);
        //     cartographer::io::ProtoStreamDeserializer deserializer(&reader);
            
        //     // 获取地图数据
        //     auto pose_graph = deserializer.pose_graph();
            
        //     // 从pbstream创建图像（这里需要根据实际的pbstream结构来实现）
        //     // 暂时创建一个示例图像用于演示
        //     cv::Mat example_image = cv::Mat::zeros(1000, 1000, CV_8UC1);
            
        //     // 创建一些示例直线用于演示
        //     cv::line(example_image, cv::Point(100, 100), cv::Point(900, 100), cv::Scalar(255), 3);
        //     cv::line(example_image, cv::Point(100, 100), cv::Point(100, 900), cv::Scalar(255), 3);
        //     cv::line(example_image, cv::Point(500, 200), cv::Point(800, 600), cv::Scalar(255), 2);
            
        //     // 转换为Cartographer图像格式
        //     auto cartographer_image = convert_cv_to_cartographer_image(example_image);
            
        //     // 创建直线特征提取器
        //     LineFeatureExtractor extractor;
            
        //     // 加载配置
        //     if (!FLAGS_config_file.empty()) {
        //         extractor.load_config(FLAGS_config_file);
        //     }
            
        //     // 应用预设配置
        //     if (!FLAGS_preset.empty()) {
        //         extractor.apply_preset(FLAGS_preset);
        //     }
            
        //     // 设置日志级别
        //     if (FLAGS_verbose) {
        //         extractor.set_log_level(LineFeatureExtractor::LogLevel::DEBUG);
        //     } else {
        //         extractor.set_log_level(LineFeatureExtractor::LogLevel::INFO);
        //     }
            
        //     // 提取直线特征
        //     auto start_time = std::chrono::high_resolution_clock::now();
            
        //     double resolution = 0.05; // 默认分辨率
        //     Eigen::Vector2d origin(0.0, 0.0); // 默认原点
            
        //     auto line_features = extractor.extract_lines(*cartographer_image, resolution, origin);
            
        //     auto end_time = std::chrono::high_resolution_clock::now();
        //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
        //     RCLCPP_INFO(get_logger(), "Extracted %zu line features in %ld ms", 
        //                line_features.size(), duration.count());
            
        //     // 保存结果
        //     save_results(line_features, resolution, origin);
            
        //     // 生成可视化
        //     if (FLAGS_enable_visualization) {
        //         generate_visualization(example_image, line_features);
        //     }
            
        //     return true;
            
        // } catch (const std::exception& e) {
        //     RCLCPP_ERROR(get_logger(), "Error during line feature extraction: %s", e.what());
        //     return false;
        // }
        return false;
    }

private:
    std::unique_ptr<::cartographer::io::Image> convert_cv_to_cartographer_image(const cv::Mat& cv_image) {
        // 创建Cartographer图像
        int width = cv_image.cols;
        int height = cv_image.rows;
        
        auto surface = cairo_image_surface_create(CAIRO_FORMAT_A8, width, height);
        auto data = cairo_image_surface_get_data(surface);
        
        // 复制图像数据
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                data[y * width + x] = cv_image.at<uchar>(y, x);
            }
        }
        
        cairo_surface_mark_dirty(surface);
        
        return std::make_unique<::cartographer::io::Image>(
            ::cartographer::io::MakeUniqueCairoSurfacePtr(surface));
    }
    
    // void save_results(const std::vector<LineFeature>& line_features, 
    //                 double resolution, const Eigen::Vector2d& origin) {
    //     // 保存JSON格式
    //     std::string json_file = FLAGS_output_dir + "/line_features.json";
    //     extractor.save_to_json(line_features, json_file, resolution, origin);
        
    //     // 保存CSV格式
    //     std::string csv_file = FLAGS_output_dir + "/line_features.csv";
    //     extractor.save_to_csv(line_features, csv_file);
        
    //     RCLCPP_INFO(get_logger(), "Results saved to %s", FLAGS_output_dir.c_str());
    // }
    
    void generate_visualization(const cv::Mat& original_image, 
                             const std::vector<LineFeature>& line_features) {
        // 创建可视化图像
        cv::Mat visualization = original_image.clone();
        cvtColor(visualization, visualization, cv::COLOR_GRAY2BGR);
        
        // 绘制直线特征
        for (const auto& feature : line_features) {
            cv::Point start(feature.start_point.x() / 0.05, 
                          feature.start_point.y() / 0.05);
            cv::Point end(feature.end_point.x() / 0.05, 
                        feature.end_point.y() / 0.05);
            
            // 根据置信度设置颜色
            cv::Scalar color;
            if (feature.confidence > 0.8) {
                color = cv::Scalar(0, 0, 255); // 红色 - 高置信度
            } else if (feature.confidence > 0.6) {
                color = cv::Scalar(0, 165, 255); // 橙色 - 中等置信度
            } else {
                color = cv::Scalar(0, 255, 255); // 黄色 - 低置信度
            }
            
            cv::line(visualization, start, end, color, 2);
            
            // 绘制端点
            cv::circle(visualization, start, 3, color, -1);
            cv::circle(visualization, end, 3, color, -1);
        }
        
        // 保存可视化结果
        std::string viz_file = FLAGS_output_dir + "/line_features_visualization.png";
        cv::imwrite(viz_file, visualization);
        
        RCLCPP_INFO(get_logger(), "Visualization saved to %s", viz_file.c_str());
    }
    std::unique_ptr<LineFeatureExtractor> extractor_ptr_;
    
};

} // namespace cartographer_ros

int main(int argc, char** argv) {
    // 初始化Google Flags和Logging
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<cartographer_ros::LineFeatureExtractorNode>();
    
    // 执行特征提取
    bool success = node->extract_features();
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return success ? 0 : 1;
}