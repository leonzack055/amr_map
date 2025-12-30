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

#ifndef CARTOGRAPHER_ROS_LINE_FEATURE_EXTRACTOR_H_
#define CARTOGRAPHER_ROS_LINE_FEATURE_EXTRACTOR_H_

#include <vector>
#include <memory>
#include <string>
#include "cartographer_ros/line_feature.h"
#include "cartographer_ros/image_preprocessor.h"
#include "cartographer_ros/hough_transform.h"
#include "cartographer_ros/lsd_detector.h"
#include "cartographer/io/image.h"
#include <opencv2/core.hpp>

namespace cartographer_ros {

// 直线特征提取器主类
class LineFeatureExtractor {
public:
  explicit LineFeatureExtractor(const LineExtractionConfig& config);
  ~LineFeatureExtractor() = default;
  
  // 主要提取接口
  std::vector<LineFeature> ExtractLineFeatures(
      const cv::Mat& image,
      double resolution,
      const Eigen::Vector2d& origin = Eigen::Vector2d::Zero());
  
  // 设置配置
  void SetConfig(const LineExtractionConfig& config);
  LineExtractionConfig GetConfig() const;
  
  // 获取统计信息
  struct ExtractionStats {
    int total_pixels;
    int edge_pixels;
    int hough_lines;
    int lsd_lines;
    int final_lines;
    double processing_time_ms;
  };
  
  ExtractionStats GetLastStats() const;
  
private:
  LineExtractionConfig config_;
  std::unique_ptr<ImagePreprocessor> preprocessor_;
  std::unique_ptr<HoughTransform> hough_transform_;
  std::unique_ptr<LSDDetector> lsd_detector_;
  
  mutable ExtractionStats last_stats_;
  
  // 自适应参数调整
  void AdaptParameters(const cv::Mat& image, double resolution);
  
  // 分析图像特征
  struct ImageAnalysis {
    double edge_density;
    double map_complexity;
    int dominant_orientation_count;
    bool is_indoor_map;
  };
  
  ImageAnalysis AnalyzeImage(const cv::Mat& edge_image);
  
  // 调整预处理参数
  void AdjustPreprocessingParams(const ImageAnalysis& analysis);
  
  // 调整检测算法参数
  void AdjustDetectionParams(const ImageAnalysis& analysis, double resolution);
  
  // 后处理线段
  std::vector<LineFeature> PostProcessLines(
      const std::vector<LineFeature>& hough_lines,
      const std::vector<LineFeature>& lsd_lines);
  
  // 线段融合
  std::vector<LineFeature> FuseLines(
      const std::vector<LineFeature>& lines);
  
  // 线段分类
  void ClassifyLines(std::vector<LineFeature>& lines);
  
  // 去重处理
  std::vector<LineFeature> RemoveDuplicates(
      const std::vector<LineFeature>& lines);
  
  // 连接断开的线段
  std::vector<LineFeature> ConnectLines(
      const std::vector<LineFeature>& lines);
  
  // 验证线段质量
  std::vector<LineFeature> ValidateLines(
      const std::vector<LineFeature>& lines);
  
  // 计算线段质量指标
  double CalculateLineQuality(const LineFeature& line,
                          const cv::Mat& edge_image);
  
  // 输出结果
  void OutputResults(const std::vector<LineFeature>& lines,
                   const std::string& base_filename);
  
  // 生成可视化图像
  void GenerateVisualization(const cv::Mat&& original_image,
                         const std::vector<LineFeature>& lines,
                         const std::string& filename);
  
  // 保存统计信息
  void SaveStatistics(const ExtractionStats& stats,
                    const std::string& filename);
  
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

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LINE_FEATURE_EXTRACTOR_H_