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

#ifndef CARTOGRAPHER_ROS_LSD_DETECTOR_H_
#define CARTOGRAPHER_ROS_LSD_DETECTOR_H_

#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include "cartographer_ros/line_feature.h"

namespace cartographer_ros {

// LSD线段检测器类
class LSDDetector {
public:
  explicit LSDDetector(const LSDConfig& config);
  ~LSDDetector() = default;
  
  // 主要检测接口
  std::vector<LineFeature> DetectLines(
      const cv::Mat& edge_image,
      double resolution);
  
  // 设置配置
  void SetConfig(const LSDConfig& config);
  LSDConfig GetConfig() const;
  
private:
  LSDConfig config_;
  
  // 梯度计算
  struct GradientInfo {
    std::vector<std::vector<double>> gx;  // x方向梯度
    std::vector<std::vector<double>> gy;  // y方向梯度
    std::vector<std::vector<double>> magnitude;  // 梯度幅值
    std::vector<std::vector<double>> angle;  // 梯度角度
    std::vector<std::vector<bool>> used;  // 是否已使用
  };
  
  // 计算图像梯度
  GradientInfo ComputeGradients(const cv::Mat& image);
  
  // 梯度排序
  struct PixelInfo {
    int x, y;
    double magnitude;
    double angle;
    
    bool operator<(const PixelInfo& other) const {
      return magnitude > other.magnitude;  // 降序排列
    }
  };
  
  std::vector<PixelInfo> SortPixelsByGradient(const GradientInfo& gradient_info);
  
  // 区域生长
  struct Region {
    std::vector<std::pair<int, int>> pixels;
    double angle_sum;
    double angle_sum_sq;
    int pixel_count;
    double angle_mean;
    double angle_std;
  };
  
  Region GrowRegion(const GradientInfo& gradient_info,
                   const PixelInfo& seed_pixel,
                   double angle_tolerance);
  
  // 矩形近似
  struct Rectangle {
    double x1, y1, x2, y2;  // 矩形顶点
    double width, height;
    double angle;
    double center_x, center_y;
    double area;
    double density;  // 点密度
  };
  
  Rectangle ApproximateRegion(const Region& region, 
                           const GradientInfo& gradient_info,
                           int image_width, int image_height);
  
  // 验证矩形
  bool ValidateRectangle(const Rectangle& rect, 
                        const Region& region,
                        int image_width, int image_height);
  
  // 矩形转换为线段
  LineFeature RectangleToLine(const Rectangle& rect, double resolution);
  
  // 计算区域统计信息
  void ComputeRegionStatistics(Region& region);
  
  // 角度归一化
  double NormalizeAngle(double angle);
  
  // 角度差值计算
  double AngleDifference(double angle1, double angle2);
  
  // 检查像素是否属于区域
  bool IsPixelInRegion(int x, int y, const Region& region,
                       double angle_tolerance);
  
  // 计算点到直线的距离
  double PointToLineDistance(double x, double y, 
                           double line_x1, double line_y1,
                           double line_x2, double line_y2);
  
  // 计算点到矩形的距离
  double PointToRectangleDistance(double x, double y, const Rectangle& rect);
  
  // 过滤重复线段
  std::vector<LineFeature> FilterDuplicateLines(
      const std::vector<LineFeature>& lines);
  
  // 计算两条线段的重叠度
  double CalculateLineOverlap(const LineFeature& line1, const LineFeature& line2);
  
  // 合并相似线段
  LineFeature MergeLines(const LineFeature& line1, const LineFeature& line2);
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LSD_DETECTOR_H_