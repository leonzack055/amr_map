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

#include "cartographer_ros/line_feature.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace cartographer_ros {
namespace line_feature_utils {

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double CalculateDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  return (p2 - p1).norm();
}

double PointToLineDistance(const Eigen::Vector2d& point, 
                         const Eigen::Vector2d& line_start,
                         const Eigen::Vector2d& line_end) {
  // 计算点到线段的最短距离
  Eigen::Vector2d line_vec = line_end - line_start;
  Eigen::Vector2d point_vec = point - line_start;
  
  double line_length_sq = line_vec.squaredNorm();
  if (line_length_sq < 1e-6) {
    // 线段长度接近零，返回点到起点的距离
    return point_vec.norm();
  }
  
  // 计算投影参数
  double t = point_vec.dot(line_vec) / line_length_sq;
  t = std::max(0.0, std::min(1.0, t));  // 限制在线段范围内
  
  // 计算投影点
  Eigen::Vector2d projection = line_start + t * line_vec;
  
  // 返回点到投影点的距离
  return (point - projection).norm();
}

double CalculateLineOverlap(const LineFeature& line1, const LineFeature& line2) {
  // 计算两条线段的重叠度
  // 首先将线段投影到其中一条线的方向上
  
  if (line1.length < 1e-6 || line2.length < 1e-6) {
    return 0.0;
  }
  
  // 使用第一条线的方向向量
  Eigen::Vector2d dir1 = (line1.end_point - line1.start_point).normalized();
  
  // 将第二条线的端点投影到第一条线的方向上
  double l2_start_proj = (line2.start_point - line1.start_point).dot(dir1);
  double l2_end_proj = (line2.end_point - line1.start_point).dot(dir1);
  
  // 确保投影范围正确
  double l2_min = std::min(l2_start_proj, l2_end_proj);
  double l2_max = std::max(l2_start_proj, l2_end_proj);
  
  // 第一条线的投影范围
  double l1_min = 0.0;
  double l1_max = line1.length;
  
  // 计算重叠长度
  double overlap_min = std::max(l1_min, l2_min);
  double overlap_max = std::min(l1_max, l2_max);
  double overlap_length = std::max(0.0, overlap_max - overlap_min);
  
  // 计算重叠比例
  double min_length = std::min(line1.length, line2.length);
  if (min_length < 1e-6) {
    return 0.0;
  }
  
  return overlap_length / min_length;
}

bool AreLinesCollinear(const LineFeature& line1, const LineFeature& line2, 
                      double angle_threshold, double distance_threshold) {
  // 检查角度相似性
  double angle_diff = std::abs(NormalizeAngle(line1.angle - line2.angle));
  if (angle_diff > angle_threshold && angle_diff < (M_PI - angle_threshold)) {
    return false;
  }
  
  // 检查距离相似性
  // 计算两条线中点之间的距离
  Eigen::Vector2d mid1 = (line1.start_point + line1.end_point) / 2.0;
  Eigen::Vector2d mid2 = (line2.start_point + line2.end_point) / 2.0;
  
  double mid_distance = CalculateDistance(mid1, mid2);
  if (mid_distance > distance_threshold) {
    return false;
  }
  
  // 检查端点到另一条线的距离
  double dist1_start = PointToLineDistance(line1.start_point, line2.start_point, line2.end_point);
  double dist1_end = PointToLineDistance(line1.end_point, line2.start_point, line2.end_point);
  double dist2_start = PointToLineDistance(line2.start_point, line1.start_point, line1.end_point);
  double dist2_end = PointToLineDistance(line2.end_point, line1.start_point, line1.end_point);
  
  double max_distance = std::max({dist1_start, dist1_end, dist2_start, dist2_end});
  
  return max_distance <= distance_threshold;
}

LineFeature MergeLines(const LineFeature& line1, const LineFeature& line2) {
  LineFeature merged;
  
  // 找到四个端点中最远的两个点
  std::vector<Eigen::Vector2d> points = {
    line1.start_point, line1.end_point,
    line2.start_point, line2.end_point
  };
  
  double max_distance = 0.0;
  int start_idx = 0, end_idx = 1;
  
  for (int i = 0; i < 4; ++i) {
    for (int j = i + 1; j < 4; ++j) {
      double dist = CalculateDistance(points[i], points[j]);
      if (dist > max_distance) {
        max_distance = dist;
        start_idx = i;
        end_idx = j;
      }
    }
  }
  
  merged.start_point = points[start_idx];
  merged.end_point = points[end_idx];
  merged.length = max_distance;
  merged.angle = std::atan2(
    merged.end_point.y() - merged.start_point.y(),
    merged.end_point.x() - merged.start_point.x());
  
  // 合并置信度和支持点数
  double total_support = line1.support_points + line2.support_points;
  if (total_support > 0) {
    merged.confidence = (line1.confidence * line1.support_points + 
                       line2.confidence * line2.support_points) / total_support;
  } else {
    merged.confidence = (line1.confidence + line2.confidence) / 2.0;
  }
  
  merged.support_points = static_cast<int>(total_support);
  
  // 选择更好的检测方法
  merged.detection_method = (line1.confidence > line2.confidence) ? 
                          line1.detection_method : line2.detection_method;
  
  // 保持原有类型（可以后续重新分类）
  merged.type = line1.type;
  
  return merged;
}

std::string LineTypeToString(LineType type) {
  switch (type) {
    case LineType::WALL:
      return "WALL";
    case LineType::CORRIDOR:
      return "CORRIDOR";
    case LineType::OBSTACLE:
      return "OBSTACLE";
    case LineType::STRUCTURE:
      return "STRUCTURE";
    case LineType::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

LineType StringToLineType(const std::string& type_str) {
  std::string upper_str = type_str;
  std::transform(upper_str.begin(), upper_str.end(), upper_str.begin(), ::toupper);
  
  if (upper_str == "WALL") {
    return LineType::WALL;
  } else if (upper_str == "CORRIDOR") {
    return LineType::CORRIDOR;
  } else if (upper_str == "OBSTACLE") {
    return LineType::OBSTACLE;
  } else if (upper_str == "STRUCTURE") {
    return LineType::STRUCTURE;
  } else {
    return LineType::UNKNOWN;
  }
}

}  // namespace line_feature_utils
}  // namespace cartographer_ros