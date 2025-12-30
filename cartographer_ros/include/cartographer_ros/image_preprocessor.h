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

#ifndef CARTOGRAPHER_ROS_IMAGE_PREPROCESSOR_H_
#define CARTOGRAPHER_ROS_IMAGE_PREPROCESSOR_H_

#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include "cartographer_ros/line_feature.h"

namespace cartographer_ros {

// 图像预处理器类
class ImagePreprocessor {
public:
  explicit ImagePreprocessor(const PreprocessingConfig& config);
  ~ImagePreprocessor() = default;
  
  // 主要预处理接口
  cv::Mat ProcessImage(const cv::Mat& image);
  
  // 分步骤预处理
  cv::Mat ConvertToGrayscale(const cv::Mat& image);
      
  cv::Mat ApplyThreshold(const cv::Mat& grayscale);
      
  cv::Mat ApplyDenoise(const cv::Mat& binary);
      
  cv::Mat ApplyMorphology(const cv::Mat& denoised);
      
  cv::Mat DetectEdges(const cv::Mat& morphological);
      
  // 调试接口
  void SaveIntermediateImage(const cv::Mat& image, const std::string& filename);
  
private:
  PreprocessingConfig config_;
  
  // Otsu阈值计算
  double CalculateOtsuThreshold(const cv::Mat& image);
  
  // 局部自适应阈值
  double CalculateLocalThreshold(const cv::Mat& image,
                              int center_x, int center_y, int block_size);
  
  // 中值滤波
  uint8_t MedianFilter(const cv::Mat& image, int x, int y, int kernel_size);
  
  // 高斯滤波
  uint8_t GaussianFilter(const cv::Mat& image, int x, int y, int kernel_size, double sigma);
  
  // 形态学操作
  cv::Mat MorphologyOpening(const cv::Mat& image, int kernel_size);
      
  cv::Mat MorphologyClosing(const cv::Mat& image, int kernel_size);
  
  // 腐蚀和膨胀
  cv::Mat MorphologyErosion(const cv::Mat& image, int kernel_size);
      
  cv::Mat MorphologyDilation(const cv::Mat& image, int kernel_size);
  
  // Canny边缘检测
  std::pair<cv::Mat, cv::Mat> CalculateGradients(const cv::Mat& image);
      
  cv::Mat NonMaximumSuppression(const cv::Mat& magnitude, const cv::Mat& direction);
      
  cv::Mat DoubleThresholdLinking(const cv::Mat& suppressed,
                              double low_threshold, double high_threshold);
  
  // 边缘连接
  void EdgeLinking(cv::Mat& edges, int x, int y, double low_threshold);
  
  // 工具函数
  bool IsInBounds(const cv::Mat& image, int x, int y);
  
  std::vector<std::pair<int, int>> GetMorphologyKernel(int kernel_size,
                                                    const std::string& shape);
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_IMAGE_PREPROCESSOR_H_