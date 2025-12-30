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

#include "cartographer_ros/lsd_detector.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <limits>
#include <opencv2/imgproc.hpp>

using namespace cv;

namespace cartographer_ros {

LSDDetector::LSDDetector(const LSDConfig& config) 
    : config_(config) {}

std::vector<LineFeature> LSDDetector::DetectLines(
    const cv::Mat& edge_image,
    double resolution) {
  
  LOG(INFO) << "开始LSD直线检测...";
  
  int height = edge_image.rows;
  int width = edge_image.cols;
  
  // 1. 计算梯度
  auto gradient_info = ComputeGradients(edge_image);
  LOG(INFO) << "梯度计算完成";
  
  // 2. 按梯度幅值排序像素
  auto sorted_pixels = SortPixelsByGradient(gradient_info);
  LOG(INFO) << "像素排序完成，共 " << sorted_pixels.size() << " 个边缘像素";
  
  // 3. 区域生长和线段检测
  std::vector<LineFeature> lines;
  cv::Mat used = cv::Mat::zeros(height, width, CV_8UC1);
  
  for (const auto& pixel : sorted_pixels) {
    if (used.at<uchar>(pixel.y, pixel.x) > 0) {
      continue;
    }
    
    // 区域生长
    double angle_tolerance = config_.ang_th * M_PI / 180.0;
    auto region = GrowRegion(gradient_info, pixel, angle_tolerance);
    
    if (region.pixel_count < config_.min_support_points) {
      continue;
    }
    
    // 标记已使用的像素
    for (const auto& [px, py] : region.pixels) {
      used.at<uchar>(py, px) = 1;
    }
    
    // 矩形近似
    auto rect = ApproximateRegion(region, gradient_info, width, height);
    
    // 验证矩形
    if (!ValidateRectangle(rect, region, width, height)) {
      continue;
    }
    
    // 转换为线段
    auto line = RectangleToLine(rect, resolution);
    
    // 验证线段
    if (line.length < config_.min_line_length_meters) {
      continue;
    }
    
    line.confidence = rect.density;
    line.support_points = region.pixel_count;
    line.detection_method = "LSD";
    
    lines.push_back(line);
  }
  
  // 4. 过滤重复线段
  auto filtered_lines = FilterDuplicateLines(lines);
  
  LOG(INFO) << "LSD检测完成，提取到 " << filtered_lines.size() << " 条直线";
  
  return filtered_lines;
}

LSDDetector::GradientInfo LSDDetector::ComputeGradients(
    const cv::Mat& image) {
  
  int height = image.rows;
  int width = image.cols;
  
  GradientInfo info;
  info.gx.resize(height, std::vector<double>(width, 0.0));
  info.gy.resize(height, std::vector<double>(width, 0.0));
  info.magnitude.resize(height, std::vector<double>(width, 0.0));
  info.angle.resize(height, std::vector<double>(width, 0.0));
  info.used.resize(height, std::vector<bool>(width, false));
  
  // 使用Sobel算子计算梯度
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      // Sobel X方向
      double gx = static_cast<double>(image.at<uchar>(y-1, x+1)) - static_cast<double>(image.at<uchar>(y-1, x-1)) +
                   2.0 * (static_cast<double>(image.at<uchar>(y, x+1)) - static_cast<double>(image.at<uchar>(y, x-1))) +
                   static_cast<double>(image.at<uchar>(y+1, x+1)) - static_cast<double>(image.at<uchar>(y+1, x-1));
      
      // Sobel Y方向
      double gy = static_cast<double>(image.at<uchar>(y-1, x-1)) + 2.0 * static_cast<double>(image.at<uchar>(y-1, x)) + static_cast<double>(image.at<uchar>(y-1, x+1)) -
                   static_cast<double>(image.at<uchar>(y+1, x-1)) - 2.0 * static_cast<double>(image.at<uchar>(y+1, x)) - static_cast<double>(image.at<uchar>(y+1, x+1));
      
      // 应用高斯平滑
      double scale = config_.scale;
      double sigma = scale * config_.sigma_scale;
      
      info.gx[y][x] = gx * scale;
      info.gy[y][x] = gy * scale;
      
      info.magnitude[y][x] = std::sqrt(gx * gx + gy * gy);
      info.angle[y][x] = std::atan2(gy, gx);
    }
  }
  
  return info;
}

std::vector<LSDDetector::PixelInfo> LSDDetector::SortPixelsByGradient(
    const GradientInfo& gradient_info) {
  
  std::vector<PixelInfo> pixels;
  
  int height = gradient_info.magnitude.size();
  int width = gradient_info.magnitude[0].size();
  
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      if (gradient_info.magnitude[y][x] > 0.0) {
        PixelInfo pixel;
        pixel.x = x;
        pixel.y = y;
        pixel.magnitude = gradient_info.magnitude[y][x];
        pixel.angle = gradient_info.angle[y][x];
        pixels.push_back(pixel);
      }
    }
  }
  
  // 按梯度幅值降序排序
  std::sort(pixels.begin(), pixels.end());
  
  return pixels;
}

LSDDetector::Region LSDDetector::GrowRegion(
    const GradientInfo& gradient_info,
    const PixelInfo& seed_pixel,
    double angle_tolerance) {
  
  Region region;
  region.angle_sum = 0.0;
  region.angle_sum_sq = 0.0;
  region.pixel_count = 0;
  
  std::queue<std::pair<int, int>> queue;
  queue.emplace(seed_pixel.x, seed_pixel.y);
  
  int height = gradient_info.magnitude.size();
  int width = gradient_info.magnitude[0].size();
  cv::Mat visited = cv::Mat::zeros(height, width, CV_8UC1);
  
  visited.at<uchar>(seed_pixel.y, seed_pixel.x) = 1;
  
  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();
    
    region.pixels.emplace_back(x, y);
    
    double angle = gradient_info.angle[y][x];
    region.angle_sum += angle;
    region.angle_sum_sq += angle * angle;
    region.pixel_count++;
    
    // 检查8邻域
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        
        int nx = x + dx;
        int ny = y + dy;
        
        if (ny < 1 || ny >= static_cast<int>(gradient_info.magnitude.size()) - 1 ||
            nx < 1 || nx >= static_cast<int>(gradient_info.magnitude[0].size()) - 1) {
          continue;
        }
        
        if (visited.at<uchar>(ny, nx) > 0 || gradient_info.magnitude[ny][nx] <= 0.0) {
          continue;
        }
        
        // 检查角度相似性
        double angle_diff = AngleDifference(angle, gradient_info.angle[ny][nx]);
        if (angle_diff > angle_tolerance) {
          continue;
        }
        
        visited.at<uchar>(ny, nx) = 1;
        queue.emplace(nx, ny);
      }
    }
  }
  
  // 计算区域统计信息
  ComputeRegionStatistics(region);
  
  return region;
}

LSDDetector::Rectangle LSDDetector::ApproximateRegion(
    const Region& region, 
    const GradientInfo& gradient_info,
    int image_width, int image_height) {
  
  if (region.pixels.empty()) {
    return {};
  }
  
  // 计算区域的中心点
  double center_x = 0.0, center_y = 0.0;
  for (const auto& [x, y] : region.pixels) {
    center_x += x;
    center_y += y;
  }
  center_x /= region.pixel_count;
  center_y /= region.pixel_count;
  
  // 计算主方向
  double main_angle = region.angle_mean;
  double cos_angle = std::cos(main_angle);
  double sin_angle = std::sin(main_angle);
  
  // 计算投影到主方向和垂直方向的坐标
  std::vector<double> projections_parallel;
  std::vector<double> projections_perpendicular;
  
  for (const auto& [x, y] : region.pixels) {
    double dx = x - center_x;
    double dy = y - center_y;
    
    double parallel = dx * cos_angle + dy * sin_angle;
    double perpendicular = -dx * sin_angle + dy * cos_angle;
    
    projections_parallel.push_back(parallel);
    projections_perpendicular.push_back(perpendicular);
  }
  
  // 找到最小和最大投影值
  auto [min_parallel, max_parallel] = std::minmax_element(
      projections_parallel.begin(), projections_parallel.end());
  auto [min_perpendicular, max_perpendicular] = std::minmax_element(
      projections_perpendicular.begin(), projections_perpendicular.end());
  
  // 计算矩形的四个顶点（在旋转坐标系中）
  double width = *max_parallel - *min_parallel;
  double height = *max_perpendicular - *min_perpendicular;
  
  // 转换回世界坐标系
  Rectangle rect;
  rect.center_x = center_x;
  rect.center_y = center_y;
  rect.width = width;
  rect.height = height;
  rect.angle = main_angle;
  rect.area = width * height;
  rect.density = static_cast<double>(region.pixel_count) / rect.area;
  
  // 计算矩形顶点
  double half_width = width / 2.0;
  double half_height = height / 2.0;
  
  // 四个角点（相对于中心）
  double corners[4][2] = {
    {-half_width, -half_height},
    {half_width, -half_height},
    {half_width, half_height},
    {-half_width, half_height}
  };
  
  // 转换到图像坐标系并找到边界
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  
  for (int i = 0; i < 4; ++i) {
    double local_x = corners[i][0] * cos_angle - corners[i][1] * sin_angle + center_x;
    double local_y = corners[i][0] * sin_angle + corners[i][1] * cos_angle + center_y;
    
    min_x = std::min(min_x, local_x);
    max_x = std::max(max_x, local_x);
    min_y = std::min(min_y, local_y);
    max_y = std::max(max_y, local_y);
  }
  
  rect.x1 = min_x;
  rect.y1 = min_y;
  rect.x2 = max_x;
  rect.y2 = max_y;
  
  return rect;
}

bool LSDDetector::ValidateRectangle(const Rectangle& rect, 
                                 const Region& region,
                                 int image_width, int image_height) {
  
  // 检查矩形是否在图像范围内
  if (rect.x1 < 0 || rect.y1 < 0 || 
      rect.x2 >= image_width || rect.y2 >= image_height) {
    return false;
  }
  
  // 检查矩形尺寸
  if (rect.width < config_.min_line_length_meters || 
      rect.height < 1.0) {
    return false;
  }
  
  // 检查密度
  if (rect.density < config_.density_th) {
    return false;
  }
  
  // 检查长宽比
  double aspect_ratio = rect.width / rect.height;
  if (aspect_ratio < 3.0) {  // 线段应该比较细长
    return false;
  }
  
  return true;
}

LineFeature LSDDetector::RectangleToLine(const Rectangle& rect, double resolution) {
  LineFeature line;
  
  // 将矩形的对角线作为线段
  line.start_point = Eigen::Vector2d(rect.x1 * resolution, rect.y1 * resolution);
  line.end_point = Eigen::Vector2d(rect.x2 * resolution, rect.y2 * resolution);
  
  line.length = rect.width * resolution;
  line.angle = rect.angle;
  line.confidence = rect.density;
  line.type = LineType::UNKNOWN;
  
  return line;
}

void LSDDetector::ComputeRegionStatistics(Region& region) {
  if (region.pixel_count == 0) {
    region.angle_mean = 0.0;
    region.angle_std = 0.0;
    return;
  }
  
  region.angle_mean = region.angle_sum / region.pixel_count;
  
  double variance = (region.angle_sum_sq / region.pixel_count) - 
                  (region.angle_mean * region.angle_mean);
  region.angle_std = std::sqrt(std::max(0.0, variance));
}

double LSDDetector::NormalizeAngle(double angle) {
  while (angle < -M_PI) angle += 2 * M_PI;
  while (angle > M_PI) angle -= 2 * M_PI;
  return angle;
}

double LSDDetector::AngleDifference(double angle1, double angle2) {
  double diff = NormalizeAngle(angle1 - angle2);
  return std::abs(diff);
}

std::vector<LineFeature> LSDDetector::FilterDuplicateLines(
    const std::vector<LineFeature>& lines) {
  
  std::vector<LineFeature> filtered_lines;
  std::vector<bool> used(lines.size(), false);
  
  for (size_t i = 0; i < lines.size(); ++i) {
    if (used[i]) continue;
    
    filtered_lines.push_back(lines[i]);
    used[i] = true;
    
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (used[j]) continue;
      
      // 检查是否为重复线段
      double angle_diff = AngleDifference(lines[i].angle, lines[j].angle);
      double distance = PointToLineDistance(
          lines[j].end_point.x(), lines[j].end_point.y(),
          lines[i].start_point.x(), lines[i].start_point.y(),
          lines[i].end_point.x(), lines[i].end_point.y());
      
      double overlap = CalculateLineOverlap(lines[i], lines[j]);
      
      if (angle_diff < 5.0 * M_PI / 180.0 && 
          distance < 0.1 && 
          overlap > 0.8) {
        used[j] = true;
        // 合并线段
        if (lines[j].length > lines[i].length) {
          filtered_lines.back() = lines[j];
        }
      }
    }
  }
  
  return filtered_lines;
}

double LSDDetector::CalculateLineOverlap(const LineFeature& line1, const LineFeature& line2) {
  // 简化的重叠度计算
  double center_dist = std::sqrt(
      std::pow(line1.end_point.x() - line2.end_point.x(), 2) +
      std::pow(line1.end_point.y() - line2.end_point.y(), 2));
  
  double avg_length = (line1.length + line2.length) / 2.0;
  
  if (center_dist > avg_length) {
    return 0.0;
  }
  
  return 1.0 - (center_dist / avg_length);
}

double LSDDetector::PointToLineDistance(double x, double y, 
                                       double line_x1, double line_y1,
                                       double line_x2, double line_y2) {
  double dx = line_x2 - line_x1;
  double dy = line_y2 - line_y1;
  double length_sq = dx * dx + dy * dy;
  
  if (length_sq == 0.0) {
    return std::sqrt((x - line_x1) * (x - line_x1) + (y - line_y1) * (y - line_y1));
  }
  
  double t = std::max(0.0, std::min(1.0, 
      ((x - line_x1) * dx + (y - line_y1) * dy) / length_sq));
  
  double projection_x = line_x1 + t * dx;
  double projection_y = line_y1 + t * dy;
  
  return std::sqrt((x - projection_x) * (x - projection_x) + 
                   (y - projection_y) * (y - projection_y));
}

void LSDDetector::SetConfig(const LSDConfig& config) {
  config_ = config;
}

LSDConfig LSDDetector::GetConfig() const {
  return config_;
}

}  // namespace cartographer_ros