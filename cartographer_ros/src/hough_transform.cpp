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

#include "cartographer_ros/hough_transform.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <random>
#include <glog/logging.h>

namespace cartographer_ros
{

HoughTransform::HoughTransform(const HoughTransformConfig& config) : config_(config)
{
  // 初始化随机数生成器
  rng_.seed(std::random_device()());
}

std::vector<LineFeature> HoughTransform::DetectLines(const cv::Mat& edge_image, double resolution,
                                                     const Eigen::Vector2d& origin)
{
  LOG(INFO) << "开始霍夫变换直线检测...";

  // 重置统计信息
  last_stats_ = DetectionStats{};
  auto start_time = std::chrono::high_resolution_clock::now();

  // 1. 图像预处理
  cv::Mat processed_image = PreprocessImage(edge_image);
  last_stats_.input_pixels = edge_image.rows * edge_image.cols;

  // 2. 执行霍夫变换
  auto cv_lines = RunHoughTransform(processed_image);
  last_stats_.detected_lines = cv_lines.size();

  // 3. 转换为LineFeature格式
  auto lines = ConvertToLineFeatures(cv_lines, resolution, origin);

  // 4. 线段验证
  auto validated_lines = ValidateLines(lines, processed_image);
  last_stats_.validated_lines = validated_lines.size();

  auto end_time = std::chrono::high_resolution_clock::now();
  last_stats_.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  LOG(INFO) << "最终提取到 " << validated_lines.size() << " 条有效直线";

  return validated_lines;
}

void HoughTransform::SetConfig(const HoughTransformConfig& config)
{
  config_ = config;
}

HoughTransformConfig HoughTransform::GetConfig() const
{
  return config_;
}

HoughTransform::DetectionStats HoughTransform::GetLastStats() const
{
  return last_stats_;
}

cv::Mat HoughTransform::PreprocessImage(const cv::Mat& edge_image)
{
  cv::Mat processed = edge_image.clone();

  // 确保是单通道二值图像
  if (processed.channels() > 1)
  {
    cv::cvtColor(processed, processed, cv::COLOR_BGR2GRAY);
  }

  // 应用高斯滤波降噪
  if (processed.channels() == 1)
  {
    cv::GaussianBlur(processed, processed, cv::Size(3, 3), 0.8);
  }

  // 确保图像是二值的
  if (processed.type() != CV_8UC1)
  {
    processed.convertTo(processed, CV_8UC1);
  }

  return processed;
}

int HoughTransform::CalculateAdaptiveThreshold(const cv::Mat& edge_image)
{
  // 计算图像的自适应阈值
  cv::Scalar mean_val = cv::mean(edge_image);
  int base_threshold = static_cast<int>(mean_val[0] * 0.5);

  // 根据图像大小调整阈值
  int image_area = edge_image.rows * edge_image.cols;
  double area_factor = std::min(2.0, image_area / (500.0 * 500.0));

  return std::max(config_.threshold, static_cast<int>(base_threshold * area_factor));
}

std::vector<cv::Vec4i> HoughTransform::RunHoughTransform(const cv::Mat& edge_image)
{
  std::vector<cv::Vec4i> lines;

  // 计算自适应阈值
  int threshold = CalculateAdaptiveThreshold(edge_image);

  // 使用OpenCV的概率霍夫变换
  cv::HoughLinesP(edge_image, lines, config_.rho_resolution, config_.theta_resolution, threshold,
                  config_.min_line_length_pixels, config_.max_line_gap_pixels);

  return lines;
}

std::vector<LineFeature> HoughTransform::ConvertToLineFeatures(const std::vector<cv::Vec4i>& lines, double resolution,
                                                               const Eigen::Vector2d& origin)
{
  std::vector<LineFeature> line_features;
  // 这里是以 x 向上； y 向左 为世界坐标；
  /*
          x ^
            |
            |
    y <-----|
  */
  for (const auto& line : lines)
  {
    LineFeature feature;

    // 转换像素坐标到世界坐标
    feature.start_x = origin.x() - (line[1] * resolution);
    feature.start_y = origin.y() - (line[0] * resolution);
    feature.end_x = origin.x() - (line[3] * resolution);
    feature.end_y = origin.y() - (line[3] * resolution);

    // 同步到Eigen向量
    feature.start_point = Eigen::Vector2d(feature.start_x, feature.start_y);
    feature.end_point = Eigen::Vector2d(feature.end_x, feature.end_y);

    // 计算基本参数
    CalculateLineParameters(feature);

    // 设置检测方法
    feature.detection_method = "hough_transform";
    feature.type = LineType::UNKNOWN;

    line_features.push_back(feature);
  }

  return line_features;
}

std::vector<LineFeature> HoughTransform::ValidateLines(const std::vector<LineFeature>& lines, const cv::Mat& edge_image)
{
  std::vector<LineFeature> validated_lines;

  // 1. 长度滤波
  auto length_filtered = FilterByLength(lines);

  // 2. 角度滤波（如果需要的话）
  auto angle_filtered = FilterByAngle(length_filtered);

  // 3. 去重处理
  auto deduplicated = RemoveOverlappingLines(angle_filtered);

  // 4. 计算置信度并验证
  for (auto& line : deduplicated)
  {
    line.confidence = CalculateConfidence(line, edge_image);

    if (line.confidence >= config_.confidence_threshold)
    {
      validated_lines.push_back(line);
    }
  }

  return validated_lines;
}

double HoughTransform::CalculateConfidence(const LineFeature& line, const cv::Mat& edge_image)
{
  // 计算支持点数
  int support_count = CalculateEdgeSupport(line, edge_image);

  // 计算期望点数（基于线段长度）
  double expected_points = line.length / 0.05;  // 假设5cm分辨率

  // 计算置信度
  double confidence = 0.0;
  if (expected_points > 0)
  {
    confidence = std::min(1.0, static_cast<double>(support_count) / expected_points);
  }

  // 长度因子
  double length_factor = std::min(1.0, line.length / config_.min_line_length_meters);

  // 综合置信度
  return confidence * 0.7 + length_factor * 0.3;
}

int HoughTransform::CalculateEdgeSupport(const LineFeature& line, const cv::Mat& edge_image)
{
  int support_count = 0;
  double tolerance = 2.0;  // 像素容差

  // 将线段转换回像素坐标
  double resolution = 0.05;  // 默认分辨率
  int x1 = static_cast<int>((line.start_x) / resolution);
  int y1 = static_cast<int>((line.start_y) / resolution);
  int x2 = static_cast<int>((line.end_x) / resolution);
  int y2 = static_cast<int>((line.end_y) / resolution);

  // 使用Bresenham算法遍历线段上的点
  int dx = std::abs(x2 - x1);
  int dy = std::abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  int current_x = x1;
  int current_y = y1;

  while (true)
  {
    // 检查当前点周围是否有边缘点
    for (int dy = -tolerance; dy <= tolerance; ++dy)
    {
      for (int dx = -tolerance; dx <= tolerance; ++dx)
      {
        int check_x = current_x + dx;
        int check_y = current_y + dy;

        if (check_x >= 0 && check_x < edge_image.cols && check_y >= 0 && check_y < edge_image.rows)
        {
          if (edge_image.at<uchar>(check_y, check_x) > 0)
          {
            support_count++;
            goto next_point;  // 找到一个支持点就跳到下一个点
          }
        }
      }
    }

  next_point:
    if (current_x == x2 && current_y == y2)
      break;

    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err -= dy;
      current_x += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      current_y += sy;
    }
  }

  return support_count;
}

std::vector<LineFeature> HoughTransform::FilterByAngle(const std::vector<LineFeature>& lines)
{
  // 这里可以根据需要实现角度滤波
  // 目前返回所有线段
  return lines;
}

std::vector<LineFeature> HoughTransform::FilterByLength(const std::vector<LineFeature>& lines)
{
  std::vector<LineFeature> filtered;

  for (const auto& line : lines)
  {
    if (line.length >= config_.min_line_length_meters)
    {
      filtered.push_back(line);
    }
  }

  return filtered;
}

std::vector<LineFeature> HoughTransform::RemoveOverlappingLines(const std::vector<LineFeature>& lines)
{
  std::vector<LineFeature> deduplicated;
  std::vector<bool> merged(lines.size(), false);

  for (size_t i = 0; i < lines.size(); ++i)
  {
    if (merged[i])
      continue;

    LineFeature current = lines[i];

    // 查找重叠的线段
    for (size_t j = i + 1; j < lines.size(); ++j)
    {
      if (merged[j])
        continue;

      if (AreLinesOverlapping(current, lines[j], config_.max_angle_diff, config_.max_distance_diff))
      {
        current = MergeLines(current, lines[j]);
        merged[j] = true;
      }
    }

    deduplicated.push_back(current);
  }

  return deduplicated;
}

bool HoughTransform::AreLinesOverlapping(const LineFeature& line1, const LineFeature& line2, double angle_threshold,
                                         double distance_threshold)
{
  // 检查角度差
  double angle_diff = std::abs(line1.angle - line2.angle);
  if (angle_diff > M_PI)
  {
    angle_diff = 2 * M_PI - angle_diff;
  }

  if (angle_diff > angle_threshold)
  {
    return false;
  }

  // 检查距离差
  double center_distance = (line1.center_x - line2.center_x) * (line1.center_x - line2.center_x) +
                           (line1.center_y - line2.center_y) * (line1.center_y - line2.center_y);
  center_distance = std::sqrt(center_distance);

  return center_distance <= distance_threshold;
}

LineFeature HoughTransform::MergeLines(const LineFeature& line1, const LineFeature& line2)
{
  // 计算合并后的端点
  std::vector<Eigen::Vector2d> points = { line1.start_point, line1.end_point, line2.start_point, line2.end_point };

  // 找到距离最大的两个点作为新的端点
  double max_distance = 0.0;
  Eigen::Vector2d new_start, new_end;

  for (size_t i = 0; i < points.size(); ++i)
  {
    for (size_t j = i + 1; j < points.size(); ++j)
    {
      double distance = (points[i] - points[j]).norm();
      if (distance > max_distance)
      {
        max_distance = distance;
        new_start = points[i];
        new_end = points[j];
      }
    }
  }

  LineFeature merged(new_start, new_end);
  merged.detection_method = "merged_hough";
  merged.confidence = std::max(line1.confidence, line2.confidence);

  return merged;
}

void HoughTransform::CalculateLineParameters(LineFeature& line)
{
  // 计算线段长度
  double dx = line.end_x - line.start_x;
  double dy = line.end_y - line.start_y;
  line.length = std::sqrt(dx * dx + dy * dy);

  // 计算角度（弧度）
  line.angle = std::atan2(dy, dx);

  // 归一化角度到 [0, π)
  if (line.angle < 0)
  {
    line.angle += M_PI;
  }
  else if (line.angle >= M_PI)
  {
    line.angle -= M_PI;
  }

  // 计算中点
  line.center_x = (line.start_x + line.end_x) / 2.0;
  line.center_y = (line.start_y + line.end_y) / 2.0;

  // 计算直线方程参数 (ax + by + c = 0)
  double length = std::sqrt(dx * dx + dy * dy);
  if (length > 1e-6)
  {
    line.a = -dy / length;
    line.b = dx / length;
    line.c = -(line.a * line.center_x + line.b * line.center_y);
  }
  else
  {
    line.a = 0.0;
    line.b = 1.0;
    line.c = -line.center_y;
  }
}

// PerformanceTimer 实现
void HoughTransform::PerformanceTimer::Start()
{
  start_time_ = std::chrono::high_resolution_clock::now();
  running_ = true;
}

void HoughTransform::PerformanceTimer::Stop()
{
  if (running_)
  {
    end_time_ = std::chrono::high_resolution_clock::now();
    running_ = false;
  }
}

double HoughTransform::PerformanceTimer::GetElapsedMilliseconds() const
{
  if (running_)
  {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(end_time_ - start_time_).count();
}

// 工具函数实现
namespace hough_utils
{

double CalculateLineAngle(const LineFeature& line)
{
  return line.angle * 180.0 / M_PI;  // 转换为度
}

double CalculateLineLength(const LineFeature& line)
{
  return line.length;
}

double NormalizeAngle(double angle_degrees)
{
  while (angle_degrees >= 180.0)
  {
    angle_degrees -= 180.0;
  }
  while (angle_degrees < 0.0)
  {
    angle_degrees += 180.0;
  }
  return angle_degrees;
}

double AngleDifference(double angle1, double angle2)
{
  double diff = std::abs(angle1 - angle2);
  if (diff > 180.0)
  {
    diff = 360.0 - diff;
  }
  return diff;
}

double PointToLineDistance(const Eigen::Vector2d& point, const LineFeature& line)
{
  return std::abs(line.a * point.x() + line.b * point.y() + line.c);
}

Eigen::Vector2d LineMidpoint(const LineFeature& line)
{
  return Eigen::Vector2d(line.center_x, line.center_y);
}

Eigen::Vector2d LineDirection(const LineFeature& line)
{
  return (line.end_point - line.start_point).normalized();
}

double PerpendicularDistance(const Eigen::Vector2d& point, const Eigen::Vector2d& line_start,
                             const Eigen::Vector2d& line_direction)
{
  Eigen::Vector2d to_point = point - line_start;
  Eigen::Vector2d perpendicular = line_direction;
  perpendicular.x() = -line_direction.y();
  perpendicular.y() = line_direction.x();

  return std::abs(to_point.dot(perpendicular));
}

}  // namespace hough_utils

}  // namespace cartographer_ros