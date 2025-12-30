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

#ifndef CARTOGRAPHER_ROS_LINE_FEATURE_UTILS_H_
#define CARTOGRAPHER_ROS_LINE_FEATURE_UTILS_H_

#include <vector>
#include <algorithm>
#include <cmath>
#include "cartographer_ros/line_feature.h"
#include <Eigen/Dense>

namespace cartographer_ros {

// 几何工具函数
namespace geometry_utils {

// 角度归一化到[0, 180)度
inline double NormalizeAngle(double angle_degrees) {
  angle_degrees = fmod(angle_degrees, 180.0);
  if (angle_degrees < 0.0) {
    angle_degrees += 180.0;
  }
  return angle_degrees;
}

// 角度差值（度）
inline double AngleDifference(double angle1, double angle2) {
  double diff = fabs(angle1 - angle2);
  if (diff > 90.0) {
    diff = 180.0 - diff;
  }
  return diff;
}

// 弧度转角度
inline double RadiansToDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

// 角度转弧度
inline double DegreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

// 计算两点距离
inline double Distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  return (p1 - p2).norm();
}

// 计算线段长度
inline double LineLength(const LineFeature& line) {
  return Distance(line.start_point, line.end_point);
}

// 计算线段角度（度）
inline double LineAngle(const LineFeature& line) {
  double dx = line.end_point.x() - line.start_point.x();
  double dy = line.end_point.y() - line.start_point.y();
  double angle = RadiansToDegrees(atan2(dy, dx));
  return NormalizeAngle(angle);
}

// 线段中点
inline Eigen::Vector2d LineMidpoint(const LineFeature& line) {
  return (line.start_point + line.end_point) * 0.5;
}

// 线段方向向量（单位向量）
inline Eigen::Vector2d LineDirection(const LineFeature& line) {
  Eigen::Vector2d dir = line.end_point - line.start_point;
  return dir.normalized();
}

// 点到线段的距离
double PointToLineDistance(const Eigen::Vector2d& point,
                        const LineFeature& line);

// 点到线段的投影点
Eigen::Vector2d PointToLineProjection(const Eigen::Vector2d& point,
                                     const LineFeature& line);

// 检查点是否在线段上
bool IsPointOnLineSegment(const Eigen::Vector2d& point,
                        const LineFeature& line,
                        double threshold = 1e-6);

}  // namespace geometry_utils

// 线段操作函数
namespace line_utils {

// 检查两条线段是否平行
bool AreLinesParallel(const LineFeature& line1, const LineFeature& line2,
                    double angle_threshold = 5.0);

// 检查两条线段是否共线
bool AreLinesCollinear(const LineFeature& line1, const LineFeature& line2,
                     double angle_threshold = 5.0, double distance_threshold = 2.0);

// 计算两条线段的交点
std::pair<bool, Eigen::Vector2d> LineIntersection(
    const LineFeature& line1, const LineFeature& line2);

// 检查两条线段是否重叠
bool DoLinesOverlap(const LineFeature& line1, const LineFeature& line2,
                  double angle_threshold = 5.0, double distance_threshold = 2.0);

// 计算线段重叠长度
double LineOverlapLength(const LineFeature& line1, const LineFeature& line2,
                       double angle_threshold = 5.0, double distance_threshold = 2.0);

// 合并两条共线线段
LineFeature MergeCollinearLines(const LineFeature& line1, const LineFeature& line2);

// 连接两条相近的线段
LineFeature ConnectLines(const LineFeature& line1, const LineFeature& line2,
                     double max_gap = 10.0);

// 延长线段到指定长度
LineFeature ExtendLine(const LineFeature& line, double target_length);

// 截断线段到指定区域
std::vector<LineFeature> ClipLineToRegion(
    const LineFeature& line,
    const Eigen::Vector2d& min_bound,
    const Eigen::Vector2d& max_bound);

}  // namespace line_utils

// 线段聚类函数
namespace clustering_utils {

// 线段聚类配置
struct ClusteringConfig {
  double angle_threshold = 10.0;        // 角度阈值（度）
  double distance_threshold = 5.0;        // 距离阈值（像素）
  double min_cluster_size = 3.0;         // 最小聚类大小
  double max_cluster_size = 50.0;        // 最大聚类大小
};

// 聚类结果
struct LineCluster {
  std::vector<size_t> line_indices;
  Eigen::Vector2d centroid;
  double mean_angle;
  double total_length;
  double quality_score;
};

// 基于角度的聚类
std::vector<LineCluster> ClusterByAngle(
    const std::vector<LineFeature>& lines,
    const ClusteringConfig& config);

// 基于位置的聚类
std::vector<LineCluster> ClusterByPosition(
    const std::vector<LineFeature>& lines,
    const ClusteringConfig& config);

// 层次聚类
std::vector<LineCluster> HierarchicalClustering(
    const std::vector<LineFeature>& lines,
    const ClusteringConfig& config);

// 计算聚类中心
Eigen::Vector2d CalculateClusterCentroid(
    const std::vector<LineFeature>& lines);

// 计算聚类平均角度
double CalculateClusterMeanAngle(
    const std::vector<LineFeature>& lines);

// 计算聚类质量分数
double CalculateClusterQuality(const LineCluster& cluster,
                           const std::vector<LineFeature>& lines);

}  // namespace clustering_utils

// 线段过滤函数
namespace filtering_utils {

// 过滤配置
struct FilterConfig {
  double min_length = 10.0;           // 最小长度
  double max_length = 1000.0;          // 最大长度
  double min_confidence = 0.1;          // 最小置信度
  double max_angle = 180.0;             // 最大角度
  double min_angle = 0.0;               // 最小角度
  bool enable_length_filter = true;        // 启用长度过滤
  bool enable_confidence_filter = true;    // 启用置信度过滤
  bool enable_angle_filter = false;        // 启用角度过滤
};

// 按长度过滤
std::vector<LineFeature> FilterByLength(
    const std::vector<LineFeature>& lines,
    double min_length, double max_length);

// 按置信度过滤
std::vector<LineFeature> FilterByConfidence(
    const std::vector<LineFeature>& lines,
    double min_confidence);

// 按角度过滤
std::vector<LineFeature> FilterByAngle(
    const std::vector<LineFeature>& lines,
    double min_angle, double max_angle);

// 按位置过滤
std::vector<LineFeature> FilterByRegion(
    const std::vector<LineFeature>& lines,
    const Eigen::Vector2d& min_bound,
    const Eigen::Vector2d& max_bound);

// 应用多重过滤器
std::vector<LineFeature> ApplyFilters(
    const std::vector<LineFeature>& lines,
    const FilterConfig& config);

}  // namespace filtering_utils

// 线段质量评估函数
namespace quality_utils {

// 质量评估配置
struct QualityConfig {
  double edge_support_weight = 0.4;      // 边缘支持权重
  double length_weight = 0.3;             // 长度权重
  double straightness_weight = 0.2;        // 直线性权重
  double continuity_weight = 0.1;          // 连续性权重
};

// 计算线段质量分数
double CalculateLineQuality(const LineFeature& line,
                        const cv::Mat& edge_image,
                        const QualityConfig& config = QualityConfig());

// 计算边缘支持度
double CalculateEdgeSupport(const LineFeature& line,
                        const cv::Mat& edge_image);

// 计算直线性
double CalculateStraightness(const LineFeature& line,
                         const cv::Mat& edge_image);

// 计算连续性
double CalculateContinuity(const LineFeature& line,
                        const cv::Mat& edge_image);

// 评估线段集合的整体质量
double EvaluateSetQuality(const std::vector<LineFeature>& lines,
                        const cv::Mat& edge_image);

}  // namespace quality_utils

// 统计分析函数
namespace statistics_utils {

// 线段统计信息
struct LineStatistics {
  size_t total_count;
  double total_length;
  double mean_length;
  double std_length;
  double mean_angle;
  double std_angle;
  double mean_confidence;
  double std_confidence;
  size_t horizontal_lines;      // 0±15度
  size_t vertical_lines;        // 90±15度
  size_t diagonal_lines;        // 其他角度
  std::vector<double> length_distribution;
  std::vector<double> angle_distribution;
};

// 计算基本统计信息
LineStatistics CalculateBasicStatistics(
    const std::vector<LineFeature>& lines);

// 计算长度分布
std::vector<double> CalculateLengthDistribution(
    const std::vector<LineFeature>& lines,
    size_t num_bins = 20);

// 计算角度分布
std::vector<double> CalculateAngleDistribution(
    const std::vector<LineFeature>& lines,
    size_t num_bins = 36);

// 生成统计报告
std::string GenerateStatisticsReport(const LineStatistics& stats);

// 可视化统计信息
void VisualizeStatistics(const LineStatistics& stats,
                      const std::string& output_dir);

}  // namespace statistics_utils

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LINE_FEATURE_UTILS_H_