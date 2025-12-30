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
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace cartographer_ros {
LineFeatureExtractor::LineFeatureExtractor(const LineExtractionConfig& config) 
    : config_(config) {
  
  // 初始化各个组件
  preprocessor_ = std::make_unique<ImagePreprocessor>(config.preprocessing);
  hough_transform_ = std::make_unique<HoughTransform>(config.hough_transform);
  lsd_detector_ = std::make_unique<LSDDetector>(config.lsd);
  
  // 初始化统计信息
  last_stats_ = {};
}

std::vector<LineFeature> LineFeatureExtractor::ExtractLineFeatures(
    const cv::Mat& image,
    double resolution,
    const Eigen::Vector2d& origin) {
  
  timer_.Start();
  LOG(INFO) << "开始直线特征提取...";
  
  // 1. 自适应参数调整
  if (config_.enable_adaptive_params) {
    AdaptParameters(image, resolution);
  }
  
  // 2. 图像预处理
  cv::Mat edge_image = preprocessor_->ProcessImage(image);
  
  // 3. 使用多种算法检测直线
  std::vector<LineFeature> hough_lines, lsd_lines;
  
  if (config_.hough_transform.enabled) {
    hough_lines = hough_transform_->DetectLines(edge_image, resolution);
    LOG(INFO) << "霍夫变换检测到 " << hough_lines.size() << " 条直线";
  }
  
  if (config_.lsd.enabled) {
    lsd_lines = lsd_detector_->DetectLines(image, resolution);
    LOG(INFO) << "LSD检测到 " << lsd_lines.size() << " 条直线";
  }
  
  // 4. 后处理和融合
  auto final_lines = PostProcessLines(hough_lines, lsd_lines);
  
  // 5. 线段分类
  if (config_.post_processing.enable_classification) {
    ClassifyLines(final_lines);
  }
  
  // 6. 输出结果
  if (config_.output.output_visualization ||
      config_.output.output_json ||
      config_.output.output_csv) {
    OutputResults(final_lines, config_.output.base_filename);
  }
  
  // 7. 更新统计信息
  timer_.Stop();
  last_stats_.total_pixels = image.rows * image.cols;
  last_stats_.edge_pixels = cv::countNonZero(edge_image);
  last_stats_.hough_lines = hough_lines.size();
  last_stats_.lsd_lines = lsd_lines.size();
  last_stats_.final_lines = final_lines.size();
  last_stats_.processing_time_ms = timer_.GetElapsedMilliseconds();
  
  LOG(INFO) << "直线特征提取完成，共提取 " << final_lines.size()
             << " 条直线，耗时 " << last_stats_.processing_time_ms << " ms";
  
  return final_lines;
}

void LineFeatureExtractor::AdaptParameters(
    const cv::Mat& image, double resolution) {
  
  // 先进行基础预处理获取边缘图像用于分析
  auto temp_preprocessor = std::make_unique<ImagePreprocessor>(config_.preprocessing);
  cv::Mat edge_image = temp_preprocessor->ProcessImage(image);
  
  // 分析图像特征
  auto analysis = AnalyzeImage(edge_image);
  
  // 调整预处理参数
  AdjustPreprocessingParams(analysis);
  
  // 调整检测算法参数
  AdjustDetectionParams(analysis, resolution);
  
  LOG(INFO) << "自适应参数调整完成";
}

LineFeatureExtractor::ImageAnalysis LineFeatureExtractor::AnalyzeImage(
    const cv::Mat& edge_image) {
  
  ImageAnalysis analysis;
  
  int total_pixels = edge_image.rows * edge_image.cols;
  int edge_pixels = cv::countNonZero(edge_image);
  std::map<int, int> angle_histogram;
  
  // 计算边缘密度和角度分布
  for (int y = 1; y < edge_image.rows - 1; ++y) {
    for (int x = 1; x < edge_image.cols - 1; ++x) {
      if (edge_image.at<uint8_t>(y, x) > 10) {
        // 计算局部梯度角度
        double gx = edge_image.at<uint8_t>(y, x+1) - edge_image.at<uint8_t>(y, x-1);
        double gy = edge_image.at<uint8_t>(y+1, x) - edge_image.at<uint8_t>(y-1, x);
        double angle = std::atan2(gy, gx) * 180.0 / M_PI;
        int angle_bin = static_cast<int>((angle + 180.0) / 10.0) % 36;
        angle_histogram[angle_bin]++;
      }
    }
  }
  
  analysis.edge_density = static_cast<double>(edge_pixels) / total_pixels;
  
  // 计算复杂度（基于角度分布）
  int max_angle_count = 0;
  for (const auto& [angle, count] : angle_histogram) {
    max_angle_count = std::max(max_angle_count, count);
  }
  
  analysis.dominant_orientation_count = 0;
  for (const auto& [angle, count] : angle_histogram) {
    if (count > max_angle_count * 0.3) {
      analysis.dominant_orientation_count++;
    }
  }
  
  analysis.map_complexity = 1.0 - static_cast<double>(max_angle_count) / edge_pixels;
  analysis.is_indoor_map = (analysis.edge_density > 0.05 &&
                           analysis.dominant_orientation_count >= 2);
  
  return analysis;
}

void LineFeatureExtractor::AdjustPreprocessingParams(const ImageAnalysis& analysis) {
  auto& config = config_.preprocessing;
  
  if (analysis.is_indoor_map) {
    // 室内地图：更强的预处理
    config.enable_morphology = true;
    config.enable_opening = true;
    config.enable_closing = true;
    config.morphology_kernel_size = 3;
    config.canny_low_threshold = 30.0;
    config.canny_high_threshold = 100.0;
  } else {
    // 室外地图：较轻的预处理
    config.enable_morphology = true;
    config.enable_opening = false;
    config.enable_closing = true;
    config.morphology_kernel_size = 5;
    config.canny_low_threshold = 50.0;
    config.canny_high_threshold = 150.0;
  }
  
  // 根据边缘密度调整阈值
  if (analysis.edge_density > 0.1) {
    config.canny_low_threshold *= 1.5;
    config.canny_high_threshold *= 1.5;
  }
  
  // 更新预处理器配置
  preprocessor_ = std::make_unique<ImagePreprocessor>(config);
}

void LineFeatureExtractor::AdjustDetectionParams(
    const ImageAnalysis& analysis, double resolution) {
  
  auto& hough_config = config_.hough_transform;
  auto& lsd_config = config_.lsd;
  
  if (analysis.is_indoor_map) {
    // 室内地图：更严格的参数
    hough_config.threshold = std::max(30, hough_config.threshold);
    hough_config.min_line_length_meters = std::max(0.5, hough_config.min_line_length_meters);
    hough_config.confidence_threshold = std::max(0.7, hough_config.confidence_threshold);
    
    lsd_config.density_th = std::max(0.7, lsd_config.density_th);
    lsd_config.min_line_length_meters = std::max(0.3, lsd_config.min_line_length_meters);
  } else {
    // 室外地图：更宽松的参数
    hough_config.threshold = std::max(20, hough_config.threshold);
    hough_config.min_line_length_meters = std::max(1.0, hough_config.min_line_length_meters);
    hough_config.confidence_threshold = std::max(0.5, hough_config.confidence_threshold);
    
    lsd_config.density_th = std::max(0.5, lsd_config.density_th);
    lsd_config.min_line_length_meters = std::max(0.5, lsd_config.min_line_length_meters);
  }
  
  // 根据分辨率调整参数
  double pixel_to_meter = 1.0 / resolution;
  hough_config.min_line_length_pixels = hough_config.min_line_length_meters * pixel_to_meter;
  hough_config.max_line_gap_pixels = hough_config.max_line_gap_meters * pixel_to_meter;
  
  // 更新检测器配置
  hough_transform_ = std::make_unique<HoughTransform>(hough_config);
  lsd_detector_ = std::make_unique<LSDDetector>(lsd_config);
}

std::vector<LineFeature> LineFeatureExtractor::PostProcessLines(
    const std::vector<LineFeature>& hough_lines,
    const std::vector<LineFeature>& lsd_lines) {
  
  // 合并两种算法的结果
  std::vector<LineFeature> all_lines;
  all_lines.insert(all_lines.end(), hough_lines.begin(), hough_lines.end());
  all_lines.insert(all_lines.end(), lsd_lines.begin(), lsd_lines.end());
  
  // 去重
  if (config_.post_processing.enable_deduplication) {
    all_lines = RemoveDuplicates(all_lines);
  }
  
  // 连接断开的线段
  if (config_.post_processing.enable_connection) {
    all_lines = ConnectLines(all_lines);
  }
  
  // 验证线段质量
  all_lines = ValidateLines(all_lines);
  
  return all_lines;
}

std::vector<LineFeature> LineFeatureExtractor::RemoveDuplicates(
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
      double angle_diff = std::abs(lines[i].angle - lines[j].angle);
      if (angle_diff > M_PI / 2) angle_diff = M_PI - angle_diff;
      
      double center_dist = std::sqrt(
          std::pow(lines[i].end_point.x() - lines[j].end_point.x(), 2) + 
          std::pow(lines[i].end_point.y() - lines[j].end_point.y(), 2));
      
      if (angle_diff < config_.post_processing.duplicate_angle_threshold &&
          center_dist < config_.post_processing.duplicate_distance_threshold) {
        used[j] = true;
        // 保留质量更高的线段
        if (lines[j].confidence > lines[i].confidence) {
          filtered_lines.back() = lines[j];
        }
      }
    }
  }
  
  return filtered_lines;
}

std::vector<LineFeature> LineFeatureExtractor::ConnectLines(
    const std::vector<LineFeature>& lines) {
  
  std::vector<LineFeature> connected_lines = lines;
  std::vector<bool> merged(lines.size(), false);
  
  for (size_t i = 0; i < lines.size(); ++i) {
    if (merged[i]) continue;
    
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (merged[j]) continue;
      
      // 检查是否可以连接
      double angle_diff = std::abs(lines[i].angle - lines[j].angle);
      if (angle_diff > M_PI / 2) angle_diff = M_PI - angle_diff;
      
      if (angle_diff < config_.post_processing.connection_angle_threshold) {
        // 计算端点距离
        double gap1 = std::sqrt(
            std::pow(lines[i].end_point.x() - lines[j].start_point.x(), 2) +
            std::pow(lines[i].end_point.y() - lines[j].start_point.y(), 2));
        
        double gap2 = std::sqrt(
            std::pow(lines[j].end_point.x() - lines[i].start_point.x(), 2) +
            std::pow(lines[j].end_point.y() - lines[i].start_point.y(), 2));
        
        double min_gap = std::min(gap1, gap2);
        
        if (min_gap < config_.post_processing.connection_gap_threshold) {
          // 合并线段
          LineFeature merged_line = line_feature_utils::MergeLines(lines[i], lines[j]);
          connected_lines.push_back(merged_line);
          merged[i] = true;
          merged[j] = true;
          break;
        }
      }
    }
  }
  
  // 移除已合并的线段
  connected_lines.erase(
      std::remove_if(connected_lines.begin(), connected_lines.end(),
                   [&merged, &lines](const LineFeature& line) {
                     auto it = std::find_if(lines.begin(), lines.end(),
                                          [&line](const LineFeature& l) {
                                            return &l == &line;
                                          });
                     if (it != lines.end()) {
                       size_t index = std::distance(lines.begin(), it);
                       return index < merged.size() && merged[index];
                     }
                     return false;
                   }),
      connected_lines.end());
  
  return connected_lines;
}

void LineFeatureExtractor::ClassifyLines(std::vector<LineFeature>& lines) {
  for (auto& line : lines) {
    // 根据长度和角度分类
    if (line.length > config_.post_processing.wall_length_threshold) {
      line.type = LineType::WALL;
    } else if (line.length > config_.post_processing.corridor_width_threshold) {
      line.type = LineType::CORRIDOR;
    } else if (line.length > config_.post_processing.obstacle_length_threshold) {
      line.type = LineType::OBSTACLE;
    } else {
      line.type = LineType::STRUCTURE;
    }
  }
}

std::vector<LineFeature> LineFeatureExtractor::ValidateLines(
    const std::vector<LineFeature>& lines) {
  
  std::vector<LineFeature> validated_lines;
  
  for (const auto& line : lines) {
    // 基本验证
    if (line.length < 0.1 || line.confidence < 0.1) {
      continue;
    }
    
    // 检查端点是否合理
    if (std::isnan(line.start_point.x()) || std::isnan(line.start_point.y()) ||
        std::isnan(line.end_point.x()) || std::isnan(line.end_point.y())) {
      continue;
    }
    
    validated_lines.push_back(line);
  }
  
  return validated_lines;
}

void LineFeatureExtractor::OutputResults(const std::vector<LineFeature>& lines,
                                    const std::string& base_filename) {
  
  if (config_.output.output_json) {
    // 输出JSON格式
    std::ofstream json_file(base_filename + ".json");
    if (json_file.is_open()) {
      json_file << "{\n";
      json_file << "  \"lines\": [\n";
      
      for (size_t i = 0; i < lines.size(); ++i) {
        const auto& line = lines[i];
        json_file << "    {\n";
        json_file << "      \"id\": " << i << ",\n";
        json_file << "      \"start\": [" << line.start_point.x() << ", " << line.start_point.y() << "],\n";
        json_file << "      \"end\": [" << line.end_point.x() << ", " << line.end_point.y() << "],\n";
        json_file << "      \"length\": " << line.length << ",\n";
        json_file << "      \"angle\": " << line.angle << ",\n";
        json_file << "      \"confidence\": " << line.confidence << ",\n";
        json_file << "      \"type\": \"" << line_feature_utils::LineTypeToString(line.type) << "\",\n";
        json_file << "      \"method\": \"" << line.detection_method << "\"\n";
        json_file << "    }" << (i < lines.size() - 1 ? "," : "") << "\n";
      }
      
      json_file << "  ]\n";
      json_file << "}\n";
      json_file.close();
    }
  }
  
  if (config_.output.output_csv) {
    // 输出CSV格式
    std::ofstream csv_file(base_filename + ".csv");
    if (csv_file.is_open()) {
      csv_file << "id,start_x,start_y,end_x,end_y,length,angle,confidence,type,method\n";
      
      for (size_t i = 0; i < lines.size(); ++i) {
        const auto& line = lines[i];
        csv_file << i << ","
                  << line.start_point.x() << ","
                  << line.start_point.y() << ","
                  << line.end_point.x() << ","
                  << line.end_point.y() << ","
                  << line.length << ","
                  << line.angle << ","
                  << line.confidence << ","
                  << line_feature_utils::LineTypeToString(line.type) << ","
                  << line.detection_method << "\n";
      }
      csv_file.close();
    }
  }
  
  if (config_.output.output_statistics) {
    SaveStatistics(last_stats_, base_filename + "_stats.txt");
  }
}

void LineFeatureExtractor::SetConfig(const LineExtractionConfig& config) {
  config_ = config;
  
  // 重新初始化组件
  preprocessor_ = std::make_unique<ImagePreprocessor>(config.preprocessing);
  hough_transform_ = std::make_unique<HoughTransform>(config.hough_transform);
  lsd_detector_ = std::make_unique<LSDDetector>(config.lsd);
}

LineExtractionConfig LineFeatureExtractor::GetConfig() const {
  return config_;
}

LineFeatureExtractor::ExtractionStats LineFeatureExtractor::GetLastStats() const {
  return last_stats_;
}

void LineFeatureExtractor::SaveStatistics(const ExtractionStats& stats,
                                       const std::string& filename) {
  std::ofstream file(filename);
  if (file.is_open()) {
    file << "直线特征提取统计信息\n";
    file << "===================\n\n";
    file << "总像素数: " << stats.total_pixels << "\n";
    file << "边缘像素数: " << stats.edge_pixels << "\n";
    file << "边缘密度: " << (static_cast<double>(stats.edge_pixels) / stats.total_pixels * 100) << "%\n";
    file << "霍夫变换检测线段数: " << stats.hough_lines << "\n";
    file << "LSD检测线段数: " << stats.lsd_lines << "\n";
    file << "最终线段数: " << stats.final_lines << "\n";
    file << "处理时间: " << stats.processing_time_ms << " ms\n";
    file.close();
  }
}

// PerformanceTimer实现
void LineFeatureExtractor::PerformanceTimer::Start() {
  start_time_ = std::chrono::high_resolution_clock::now();
  running_ = true;
}

void LineFeatureExtractor::PerformanceTimer::Stop() {
  if (running_) {
    end_time_ = std::chrono::high_resolution_clock::now();
    running_ = false;
  }
}

double LineFeatureExtractor::PerformanceTimer::GetElapsedMilliseconds() const {
  if (running_) {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
    return duration.count() / 1000.0;
  } else {
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_);
    return duration.count() / 1000.0;
  }
}

}  // namespace cartographer_ros