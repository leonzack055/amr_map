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
#include "cartographer_ros/line_feature_utils.h"
#include "cartographer/io/image.h"
#include <iostream>
#include <fstream>
#include <chrono>

namespace cartographer_ros {

// 创建测试图像
std::unique_ptr<::cartographer::io::Image> CreateTestImage(int width, int height) {
  auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height));
  auto context = ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
  
  // 设置白色背景
  cairo_set_source_rgb(context.get(), 1.0, 1.0, 1.0);
  cairo_paint(context.get());
  
  // 设置黑色线条
  cairo_set_source_rgb(context.get(), 0.0, 0.0, 0.0);
  cairo_set_line_width(context.get(), 2.0);
  
  // 绘制一些测试直线
  // 水平线
  cairo_move_to(context.get(), 50, 100);
  cairo_line_to(context.get(), 350, 100);
  cairo_stroke(context.get());
  
  // 垂直线
  cairo_move_to(context.get(), 200, 50);
  cairo_line_to(context.get(), 200, 250);
  cairo_stroke(context.get());
  
  // 对角线
  cairo_move_to(context.get(), 100, 50);
  cairo_line_to(context.get(), 300, 250);
  cairo_stroke(context.get());
  
  // 矩形
  cairo_rectangle(context.get(), 80, 120, 60, 80);
  cairo_stroke(context.get());
  
  return std::make_unique<::cartographer::io::Image>(std::move(surface));
}

// 保存测试图像
void SaveTestImage(const ::cartographer::io::Image& image, const std::string& filename) {
  std::ofstream file(filename + ".pgm");
  if (!file.is_open()) {
    std::cerr << "无法保存测试图像: " << filename << std::endl;
    return;
  }
  
  int width = image.width();
  int height = image.height();
  
  file << "P5\n" << width << " " << height << "\n255\n";
  
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto pixel = image.GetPixel(x, y);
      // 转换为灰度
      uint8_t gray = static_cast<uint8_t>(0.299 * pixel[0] + 0.587 * pixel[1] + 0.114 * pixel[2]);
      file.write(reinterpret_cast<const char*>(&gray), 1);
    }
  }
  
  file.close();
  std::cout << "测试图像已保存: " << filename << ".pgm" << std::endl;
}

// 验证直线特征
void ValidateLineFeatures(const std::vector<LineFeature>& lines) {
  std::cout << "\n=== 直线特征验证 ===" << std::endl;
  std::cout << "检测到的直线数量: " << lines.size() << std::endl;
  
  int wall_count = 0, corridor_count = 0, obstacle_count = 0, structure_count = 0;
  double total_length = 0.0;
  double avg_confidence = 0.0;
  
  for (const auto& line : lines) {
    total_length += line.length;
    avg_confidence += line.confidence;
    
    switch (line.type) {
      case LineType::WALL:
        wall_count++;
        break;
      case LineType::CORRIDOR:
        corridor_count++;
        break;
      case LineType::OBSTACLE:
        obstacle_count++;
        break;
      case LineType::STRUCTURE:
        structure_count++;
        break;
      default:
        break;
    }
    
    std::cout << "直线: 起点(" << line.start_point.x() << ", " << line.start_point.y() 
               << ") -> 终点(" << line.end_point.x() << ", " << line.end_point.y() 
               << "), 长度=" << line.length 
               << ", 角度=" << line.angle * 180.0 / M_PI << "°"
               << ", 置信度=" << line.confidence
               << ", 类型=" << line_feature_utils::LineTypeToString(line.type)
               << ", 方法=" << line.detection_method << std::endl;
  }
  
  if (!lines.empty()) {
    avg_confidence /= lines.size();
  }
  
  std::cout << "\n统计信息:" << std::endl;
  std::cout << "  总长度: " << total_length << std::endl;
  std::cout << "  平均长度: " << (lines.empty() ? 0.0 : total_length / lines.size()) << std::endl;
  std::cout << "  平均置信度: " << avg_confidence << std::endl;
  std::cout << "  墙壁: " << wall_count << std::endl;
  std::cout << "  走廊: " << corridor_count << std::endl;
  std::cout << "  障碍物: " << obstacle_count << std::endl;
  std::cout << "  结构: " << structure_count << std::endl;
}

// 性能测试
void PerformanceTest() {
  std::cout << "\n=== 性能测试 ===" << std::endl;
  
  // 创建不同大小的测试图像
  std::vector<std::pair<int, int>> test_sizes = {
    {400, 400}, {800, 800}, {1200, 1200}
  };
  
  for (const auto& [width, height] : test_sizes) {
    std::cout << "\n测试图像大小: " << width << "x" << height << std::endl;
    
    auto test_image = CreateTestImage(width, height);
    
    // 测试不同配置
    std::vector<std::string> presets = {"fast", "indoor", "high_precision"};
    
    for (const auto& preset : presets) {
      std::cout << "  预设: " << preset << std::endl;
      
      LineExtractionConfig config;
      config.enable_line_extraction = true;
      config.enable_adaptive_params = false;  // 禁用自适应以保持一致性
      config.enable_debug_output = false;
      
      // 应用预设配置
      if (preset == "fast") {
        config.hough_transform.threshold = 20;
        config.hough_transform.confidence_threshold = 0.4;
        config.lsd.enabled = false;
      } else if (preset == "indoor") {
        config.hough_transform.threshold = 40;
        config.hough_transform.confidence_threshold = 0.7;
        config.lsd.enabled = true;
      } else if (preset == "high_precision") {
        config.hough_transform.threshold = 60;
        config.hough_transform.confidence_threshold = 0.8;
        config.lsd.enabled = true;
      }
      
      LineFeatureExtractor extractor(config);
      
      auto start_time = std::chrono::high_resolution_clock::now();
      auto lines = extractor.ExtractLineFeatures(*test_image, 0.05);
      auto end_time = std::chrono::high_resolution_clock::now();
      
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          end_time - start_time);
      
      auto stats = extractor.GetLastStats();
      
      std::cout << "    处理时间: " << duration.count() << " ms" << std::endl;
      std::cout << "    检测线段数: " << lines.size() << std::endl;
      std::cout << "    霍夫变换: " << stats.hough_lines << std::endl;
      std::cout << "    LSD: " << stats.lsd_lines << std::endl;
      std::cout << "    最终线段: " << stats.final_lines << std::endl;
    }
  }
}

// 准确性测试
void AccuracyTest() {
  std::cout << "\n=== 准确性测试 ===" << std::endl;
  
  // 创建包含已知直线的测试图像
  auto test_image = CreateTestImage(400, 300);
  
  LineExtractionConfig config;
  config.enable_line_extraction = true;
  config.enable_adaptive_params = true;
  config.enable_debug_output = true;
  
  LineFeatureExtractor extractor(config);
  auto lines = extractor.ExtractLineFeatures(*test_image, 0.05);
  
  // 验证检测结果
  std::vector<std::pair<double, double>> expected_lines = {
    {0.0, 100.0},      // 水平线 y=100
    {200.0, 0.0},        // 垂直线 x=200
    {63.4, 50.0},        // 对角线，角度约63.4度
  };
  
  std::cout << "期望的直线:" << std::endl;
  for (const auto& [angle, position] : expected_lines) {
    std::cout << "  角度: " << angle << "°, 位置: " << position << std::endl;
  }
  
  std::cout << "\n检测到的直线:" << std::endl;
  for (const auto& line : lines) {
    double angle_deg = line.angle * 180.0 / M_PI;
    std::cout << "  角度: " << angle_deg << "°, 长度: " << line.length 
               << ", 置信度: " << line.confidence << std::endl;
  }
  
  // 简单的准确性评估
  if (lines.size() >= 3) {
    std::cout << "\n准确性评估: 通过 - 检测到足够的直线特征" << std::endl;
  } else {
    std::cout << "\n准确性评估: 失败 - 检测到的直线特征不足" << std::endl;
  }
}

}  // namespace cartographer_ros

int main(int argc, char** argv) {
  std::cout << "Cartographer 直线特征提取测试程序" << std::endl;
  std::cout << "================================" << std::endl;
  
  try {
    // 1. 创建测试图像
    std::cout << "\n1. 创建测试图像..." << std::endl;
    auto test_image = cartographer_ros::CreateTestImage(400, 300);
    cartographer_ros::SaveTestImage(*test_image, "test_image");
    
    // 2. 基本功能测试
    std::cout << "\n2. 基本功能测试..." << std::endl;
    cartographer_ros::LineExtractionConfig config;
    config.enable_line_extraction = true;
    config.enable_adaptive_params = true;
    config.enable_debug_output = true;
    
    cartographer_ros::LineFeatureExtractor extractor(config);
    auto lines = extractor.ExtractLineFeatures(*test_image, 0.05);
    cartographer_ros::ValidateLineFeatures(lines);
    
    // 3. 性能测试
    cartographer_ros::PerformanceTest();
    
    // 4. 准确性测试
    cartographer_ros::AccuracyTest();
    
    std::cout << "\n=== 测试完成 ===" << std::endl;
    
  } catch (const std::exception& e) {
    std::cerr << "测试过程中发生错误: " << e.what() << std::endl;
    return 1;
  }
  
  return 0;
}