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

#ifndef CARTOGRAPHER_ROS_HOUGH_TRANSFORM_H_
#define CARTOGRAPHER_ROS_HOUGH_TRANSFORM_H_

#include <vector>
#include <memory>
#include "cartographer_ros/line_feature.h"
#include <opencv2/opencv.hpp>
#include <random>

namespace cartographer_ros {

// HoughTransformConfig已在line_feature.h中定义，这里不再重复定义

// 霍夫变换检测器类
class HoughTransform {
 public:
  explicit HoughTransform(const HoughTransformConfig& config);
  ~HoughTransform() = default;
  
  // 主要检测接口
  std::vector<LineFeature> DetectLines(
      const cv::Mat& edge_image,
      double resolution = 0.05,
      const Eigen::Vector2d& origin = Eigen::Vector2d::Zero());
  
  // 设置配置
  void SetConfig(const HoughTransformConfig& config);
  HoughTransformConfig GetConfig() const;
  
  // 获取检测统计信息
  struct DetectionStats {
    int input_pixels;
    int edge_pixels;
    int accumulator_votes;
    int detected_lines;
    int validated_lines;
    double processing_time_ms;
  };
  
  DetectionStats GetLastStats() const;
  
 private:
 std::mt19937 rng_; // 随机数生成器
  HoughTransformConfig config_;
  mutable DetectionStats last_stats_;
  
  // 图像预处理
  cv::Mat PreprocessImage(const cv::Mat& edge_image);
  
  // 自适应阈值计算
  int CalculateAdaptiveThreshold(const cv::Mat& edge_image);
  
  // 执行霍夫变换
  std::vector<cv::Vec4i> RunHoughTransform(const cv::Mat& edge_image);
  
  // 转换为LineFeature格式
  std::vector<LineFeature> ConvertToLineFeatures(
      const std::vector<cv::Vec4i>& lines,
      double resolution,
      const Eigen::Vector2d& origin);
  
  // 线段验证
  std::vector<LineFeature> ValidateLines(
      const std::vector<LineFeature>& lines,
      const cv::Mat& edge_image);
  
  // 计算线段置信度
  double CalculateConfidence(const LineFeature& line,
                         const cv::Mat& edge_image);
  
  // 边缘支持度计算
  int CalculateEdgeSupport(const LineFeature& line,
                        const cv::Mat& edge_image);
  
  // 角度滤波
  std::vector<LineFeature> FilterByAngle(
      const std::vector<LineFeature>& lines);
  
  // 长度滤波
  std::vector<LineFeature> FilterByLength(
      const std::vector<LineFeature>& lines);
  
  // 去重处理
  std::vector<LineFeature> RemoveOverlappingLines(
      const std::vector<LineFeature>& lines);
  
  // 检查线段重叠
  bool AreLinesOverlapping(const LineFeature& line1,
                         const LineFeature& line2,
                         double angle_threshold = 5.0,
                         double distance_threshold = 2.0);
  
  // 合并重叠线段
  LineFeature MergeLines(const LineFeature& line1,
                      const LineFeature& line2);
  
  // 计算线段参数
  void CalculateLineParameters(LineFeature& line);
  
  // 性能监控
  class PerformanceTimer {
   public:
    void Start();
    void Stop();
    double GetElapsedMilliseconds() const;
    
   private:
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point end_time_;
    bool running_;
  };
  
  mutable PerformanceTimer timer_;
};

// 工具函数
namespace hough_utils {

// 计算线段角度（度）
double CalculateLineAngle(const LineFeature& line);

// 计算线段长度
double CalculateLineLength(const LineFeature& line);

// 角度归一化到[0, 180)
double NormalizeAngle(double angle_degrees);

// 角度差值计算
double AngleDifference(double angle1, double angle2);

// 点到线段距离
double PointToLineDistance(const Eigen::Vector2d& point,
                        const LineFeature& line);

// 线段中点
Eigen::Vector2d LineMidpoint(const LineFeature& line);

// 线段方向向量
Eigen::Vector2d LineDirection(const LineFeature& line);

// 垂直距离
double PerpendicularDistance(const Eigen::Vector2d& point,
                         const Eigen::Vector2d& line_start,
                         const Eigen::Vector2d& line_direction);

}  // namespace hough_utils

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_HOUGH_TRANSFORM_H_