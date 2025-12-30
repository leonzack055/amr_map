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

#ifndef CARTOGRAPHER_ROS_LINE_FEATURE_H_
#define CARTOGRAPHER_ROS_LINE_FEATURE_H_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "Eigen/Core"
#include "cartographer/io/image.h"

namespace cartographer_ros {

// 直线类型枚举
enum class LineType {
  UNKNOWN = 0,
  WALL = 1,           // 墙壁
  CORRIDOR = 2,       // 走廊
  OBSTACLE = 3,       // 障碍物边界
  STRUCTURE = 4        // 结构线
};

// 直线特征数据结构
struct LineFeature {
  Eigen::Vector2d start_point;    // 起点（世界坐标）
  Eigen::Vector2d end_point;      // 终点（世界坐标）
  double length;                  // 长度（米）
  double angle;                   // 角度（弧度）
  double confidence;              // 置信度 [0,1]
  int support_points;            // 支持点数
  std::string detection_method;  // 检测方法
  LineType type;                 // 线段类型
  
  // 兼容性字段（与霍夫变换实现保持一致）
  double start_x;               // 起点X坐标
  double start_y;               // 起点Y坐标
  double end_x;                 // 终点X坐标
  double end_y;                 // 终点Y坐标
  double center_x;              // 中心点X坐标
  double center_y;              // 中心点Y坐标
  double a, b, c;             // 直线方程参数
  
  LineFeature()
    : start_point(Eigen::Vector2d::Zero()),
      end_point(Eigen::Vector2d::Zero()),
      length(0.0),
      angle(0.0),
      confidence(0.0),
      support_points(0),
      detection_method("unknown"),
      type(LineType::UNKNOWN),
      start_x(0.0), start_y(0.0), end_x(0.0), end_y(0.0),
      center_x(0.0), center_y(0.0), a(0.0), b(0.0), c(0.0) {}
      
  // 便捷构造函数
  LineFeature(const Eigen::Vector2d& start, const Eigen::Vector2d& end)
    : start_point(start), end_point(end) {
    length = (end - start).norm();
    angle = std::atan2(end.y() - start.y(), end.x() - start.x());
    confidence = 0.0;
    support_points = 0;
    detection_method = "unknown";
    type = LineType::UNKNOWN;
    
    // 设置兼容性字段
    start_x = start.x();
    start_y = start.y();
    end_x = end.x();
    end_y = end.y();
    center_x = (start_x + end_x) / 2.0;
    center_y = (start_y + end_y) / 2.0;
    
    // 计算直线方程参数
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double line_length = std::sqrt(dx * dx + dy * dy);
    if (line_length > 1e-6) {
      a = -dy / line_length;
      b = dx / line_length;
      c = -(a * center_x + b * center_y);
    }
  }
  
  // 同步方法
  void SyncFields() {
    start_x = start_point.x();
    start_y = start_point.y();
    end_x = end_point.x();
    end_y = end_point.y();
    center_x = (start_x + end_x) / 2.0;
    center_y = (start_y + end_y) / 2.0;
    
    // 重新计算直线方程参数
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double line_length = std::sqrt(dx * dx + dy * dy);
    if (line_length > 1e-6) {
      // 单位法向量(-dy,dx)还有一种(dy,-dx)
      a = -dy / line_length;
      b = dx / line_length;
      c = -(a * center_x + b * center_y);
    }
  }
};

// 预处理配置
struct PreprocessingConfig {
  // 调试选项
  bool enable_debug_output = false;
  
  // 二值化
  bool use_otsu_threshold = true;
  double manual_threshold = 50.0;
  int adaptive_block_size = 15;
  double adaptive_c = 2.0;
  
  // 降噪
  bool enable_median_filter = true;
  bool enable_gaussian_filter = false;
  int filter_kernel_size = 3;
  double gaussian_sigma = 1.0;
  
  // 形态学操作
  bool enable_morphology = true;
  bool enable_opening = true;
  bool enable_closing = true;
  int morphology_kernel_size = 3;
  std::string morphology_kernel_shape = "rectangle";
  
  // 边缘检测
  bool enable_canny = true;
  double canny_low_threshold = 50.0;
  double canny_high_threshold = 150.0;
  int canny_kernel_size = 3;
  bool use_l2_gradient = true;
};

// 霍夫变换配置
struct HoughTransformConfig {
  bool enabled = true;
  
  // 基本参数
  double rho_resolution = 1.0;           // 距离分辨率（像素）
  double theta_resolution = M_PI / 180.0; // 角度分辨率（弧度）
  int threshold = 50;                    // 投票阈值
  
  // 线段参数
  double min_line_length_pixels = 30.0;   // 最小线长（像素）
  double max_line_gap_pixels = 10.0;      // 最大间隙（像素）
  double min_line_length_meters = 0.5;    // 最小线长（米）
  double max_line_gap_meters = 0.2;       // 最大间隙（米）
  
  // 过滤参数
  double confidence_threshold = 0.7;
  int min_support_points = 10;
  
  // 融合参数
  double max_angle_diff = 5.0 * M_PI / 180.0;  // 最大角度差（弧度）
  double max_distance_diff = 0.1;               // 最大距离差（米）
  double min_overlap_ratio = 0.3;                // 最小重叠比例
};

// LSD配置
struct LSDConfig {
  bool enabled = true;
  
  // 基本参数
  double scale = 0.8;
  double sigma_scale = 0.6;
  double quant = 2.0;
  double ang_th = 22.5;
  double log_eps = 0.0;
  double density_th = 0.7;
  int n_bins = 1024;
  
  // 过滤参数
  double min_line_length_meters = 0.3;
  double confidence_threshold = 0.6;
  int min_support_points = 5;
};

// 后处理配置
struct PostProcessingConfig {
  // 去重配置
  bool enable_deduplication = true;
  double duplicate_angle_threshold = 3.0 * M_PI / 180.0;
  double duplicate_distance_threshold = 0.05;
  double duplicate_overlap_threshold = 0.8;
  
  // 连接配置
  bool enable_connection = true;
  double connection_angle_threshold = 5.0 * M_PI / 180.0;
  double connection_gap_threshold = 0.15;
  
  // 分类配置
  bool enable_classification = true;
  double wall_length_threshold = 1.0;
  double corridor_width_threshold = 2.0;
  double obstacle_length_threshold = 0.5;
};

// 输出配置
struct OutputConfig {
  bool output_json = true;
  bool output_csv = true;
  bool output_yaml = true;
  bool output_visualization = true;
  bool output_protobuf = true;
  bool output_statistics = true;
  
  std::string output_directory = "./line_features/";
  std::string base_filename = "map_lines";
  bool separate_by_type = true;
  bool include_debug_images = false;
};

// 性能配置
struct PerformanceConfig {
  bool enable_parallel_processing = true;
  int num_threads = 0;  // 0表示使用所有可用线程
  int max_image_width = 2048;
  int max_image_height = 2048;
  bool enable_block_processing = true;
  int block_size = 512;
  bool enable_memory_optimization = true;
};

// 主配置结构
struct LineExtractionConfig {
  // 基本开关
  bool enable_line_extraction = true;
  bool enable_adaptive_params = true;
  bool enable_debug_output = true;
  
  // 子配置
  PreprocessingConfig preprocessing;
  HoughTransformConfig hough_transform;
  LSDConfig lsd;
  PostProcessingConfig post_processing;
  OutputConfig output;
  PerformanceConfig performance;
};

// 工具函数
namespace line_feature_utils {

// 角度归一化到[-π, π]
double NormalizeAngle(double angle);

// 计算两点间距离
double CalculateDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

// 计算点到直线的距离
double PointToLineDistance(const Eigen::Vector2d& point, 
                         const Eigen::Vector2d& line_start,
                         const Eigen::Vector2d& line_end);

// 计算两条直线的重叠度
double CalculateLineOverlap(const LineFeature& line1, const LineFeature& line2);

// 判断两条直线是否共线
bool AreLinesCollinear(const LineFeature& line1, const LineFeature& line2, 
                      double angle_threshold, double distance_threshold);

// 合并两条直线
LineFeature MergeLines(const LineFeature& line1, const LineFeature& line2);

// 将线段类型转换为字符串
std::string LineTypeToString(LineType type);

// 将字符串转换为线段类型
LineType StringToLineType(const std::string& type_str);

}  // namespace line_feature_utils

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LINE_FEATURE_H_