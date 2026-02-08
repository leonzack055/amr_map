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

#include "cartographer_ros/image_preprocessor.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;

namespace cartographer_ros {

ImagePreprocessor::ImagePreprocessor(const PreprocessingConfig& config) 
    : config_(config) {}

cv::Mat ImagePreprocessor::ProcessImage(const cv::Mat& image) {
  
  LOG(INFO) << "开始图像预处理...";
  
  // 1. 转换为灰度图
  cv::Mat grayscale = ConvertToGrayscale(image);
  LOG(INFO) << "灰度转换完成";
  
  if (config_.enable_debug_output) {
    SaveIntermediateImage(grayscale, "01_grayscale.png");
  } else {
    LOG(ERROR) << "没有设置DEBUG模式，enable_debug_output is false";
  }
  
  // 2. 阈值化
  cv::Mat binary = ApplyThreshold(grayscale);
  LOG(INFO) << "阈值化完成";
  
  if (config_.enable_debug_output) {
    SaveIntermediateImage(binary, "02_binary.png");
  }
  
  // 3. 降噪
  cv::Mat denoised = ApplyDenoise(binary);
  LOG(INFO) << "降噪完成";
  
  if (config_.enable_debug_output) {
    SaveIntermediateImage(denoised, "03_denoised.png");
  }
  
  // 4. 形态学操作
  cv::Mat morphological = ApplyMorphology(denoised);
  LOG(INFO) << "形态学操作完成";
  
  if (config_.enable_debug_output) {
    SaveIntermediateImage(morphological, "04_morphological.png");
  }
  
  // 5. 边缘检测
  cv::Mat edges = DetectEdges(morphological);
  LOG(INFO) << "边缘检测完成";
  
  if (config_.enable_debug_output) {
    SaveIntermediateImage(edges, "05_edges.png");
  }
  
  LOG(INFO) << "图像预处理完成";
  return edges;
}

cv::Mat ImagePreprocessor::ConvertToGrayscale(const cv::Mat& image) {
  
  cv::Mat grayscale;
  
  if (image.channels() == 3) {
    // BGR转灰度
    cvtColor(image, grayscale, COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    // BGRA转灰度
    cvtColor(image, grayscale, COLOR_BGRA2GRAY);
  } else {
    // 已经是灰度图
    grayscale = image.clone();
  }
  
  return grayscale;
}

cv::Mat ImagePreprocessor::ApplyThreshold(const cv::Mat& grayscale) {
  
  cv::Mat binary;
  
  if (config_.use_otsu_threshold) {
    // 全局Otsu阈值
    // double otsu_threshold = CalculateOtsuThreshold(grayscale);
    double otsu_threshold = config_.manual_threshold;
    LOG(INFO) << "Otsu阈值: " << otsu_threshold;
    threshold(grayscale, binary, otsu_threshold, 255, THRESH_BINARY);
  } else {
    // 局部自适应阈值
    adaptiveThreshold(grayscale, binary, 255,
                         ADAPTIVE_THRESH_GAUSSIAN_C,
                         THRESH_BINARY,
                         config_.adaptive_block_size,
                         config_.adaptive_c);
  }
  
  return binary;
}

double ImagePreprocessor::CalculateOtsuThreshold(const cv::Mat& image) {
  
  // 使用OpenCV的Otsu阈值计算
  cv::Mat binary;
  double otsu_value = threshold(image, binary, 0, 255,
                                     THRESH_BINARY | THRESH_OTSU);
  
  return otsu_value;
}

double ImagePreprocessor::CalculateLocalThreshold(
    const cv::Mat& image,
    int center_x, int center_y, int block_size) {
  
  int half_block = block_size / 2;
  int start_x = std::max(0, center_x - half_block);
  int end_x = std::min(image.cols - 1, center_x + half_block);
  int start_y = std::max(0, center_y - half_block);
  int end_y = std::min(image.rows - 1, center_y + half_block);
  
  cv::Rect roi(start_x, start_y, end_x - start_x + 1, end_y - start_y + 1);
  cv::Mat roi_image = image(roi);
  
  cv::Scalar mean = cv::mean(roi_image);
  return mean[0] - config_.adaptive_c;
}

cv::Mat ImagePreprocessor::ApplyDenoise(const cv::Mat& binary) {
  
  cv::Mat denoised;
  
  if (config_.enable_median_filter) {
    medianBlur(binary, denoised, config_.filter_kernel_size);
  } else if (config_.enable_gaussian_filter) {
    GaussianBlur(binary, denoised,
                     cv::Size(config_.filter_kernel_size, config_.filter_kernel_size),
                     config_.gaussian_sigma);
  } else {
    denoised = binary.clone();
  }
  
  return denoised;
}

cv::Mat ImagePreprocessor::ApplyMorphology(const cv::Mat& denoised) {
  
  if (!config_.enable_morphology) {
    return denoised.clone();
  }
  
  cv::Mat result = denoised.clone();
  
  if (config_.enable_opening) {
    result = MorphologyOpening(result, config_.morphology_kernel_size);
  }
  
  if (config_.enable_closing) {
    result = MorphologyClosing(result, config_.morphology_kernel_size);
  }
  
  return result;
}

cv::Mat ImagePreprocessor::MorphologyOpening(const cv::Mat& image, int kernel_size) {
  // 开运算 = 腐蚀 + 膨胀
  cv::Mat eroded = MorphologyErosion(image, kernel_size);
  return MorphologyDilation(eroded, kernel_size);
}

cv::Mat ImagePreprocessor::MorphologyClosing(const cv::Mat& image, int kernel_size) {
  // 闭运算 = 膨胀 + 腐蚀
  cv::Mat dilated = MorphologyDilation(image, kernel_size);
  return MorphologyErosion(dilated, kernel_size);
}

cv::Mat ImagePreprocessor::MorphologyErosion(const cv::Mat& image, int kernel_size) {
  
  cv::Mat result;
  auto kernel = getStructuringElement(MORPH_RECT,
                                        cv::Size(kernel_size, kernel_size));
  erode(image, result, kernel);
  
  return result;
}

cv::Mat ImagePreprocessor::MorphologyDilation(const cv::Mat& image, int kernel_size) {
  
  cv::Mat result;
  auto kernel = getStructuringElement(MORPH_RECT,
                                        cv::Size(kernel_size, kernel_size));
  dilate(image, result, kernel);
  
  return result;
}

cv::Mat ImagePreprocessor::DetectEdges(const cv::Mat& morphological) {
  
  if (!config_.enable_canny) {
    return morphological.clone();
  }
  
  cv::Mat edges;
  Canny(morphological, edges,
             config_.canny_low_threshold,
             config_.canny_high_threshold,
             config_.canny_kernel_size,
             config_.use_l2_gradient);
  
  return edges;
}

std::pair<cv::Mat, cv::Mat> ImagePreprocessor::CalculateGradients(const cv::Mat& image) {
  
  cv::Mat gx, gy;
  Sobel(image, gx, CV_64F, 1, 0, 3);
  Sobel(image, gy, CV_64F, 0, 1, 3);
  
  cv::Mat magnitude, direction;
  cartToPolar(gx, gy, magnitude, direction, config_.use_l2_gradient);
  
  return {magnitude, direction};
}

cv::Mat ImagePreprocessor::NonMaximumSuppression(const cv::Mat& magnitude,
                                               const cv::Mat& direction) {
  
  // 使用OpenCV的Canny内部实现，这里简化处理
  cv::Mat suppressed;
  Canny(magnitude, suppressed, 0, 255, 3);
  return suppressed;
}

cv::Mat ImagePreprocessor::DoubleThresholdLinking(const cv::Mat& suppressed,
                                               double low_threshold,
                                               double high_threshold) {
  
  cv::Mat edges;
  threshold(suppressed, edges, high_threshold, 255, THRESH_BINARY);
  return edges;
}

bool ImagePreprocessor::IsInBounds(const cv::Mat& image, int x, int y) {
  return y >= 0 && y < image.rows && x >= 0 && x < image.cols;
}

std::vector<std::pair<int, int>> ImagePreprocessor::GetMorphologyKernel(
    int kernel_size, const std::string& shape) {
  
  std::vector<std::pair<int, int>> kernel;
  int half_kernel = kernel_size / 2;
  
  if (shape == "rectangle") {
    for (int dy = -half_kernel; dy <= half_kernel; ++dy) {
      for (int dx = -half_kernel; dx <= half_kernel; ++dx) {
        kernel.emplace_back(dx, dy);
      }
    }
  } else if (shape == "circle") {
    for (int dy = -half_kernel; dy <= half_kernel; ++dy) {
      for (int dx = -half_kernel; dx <= half_kernel; ++dx) {
        if (dx * dx + dy * dy <= half_kernel * half_kernel) {
          kernel.emplace_back(dx, dy);
        }
      }
    }
  }
  
  return kernel;
}

void ImagePreprocessor::SaveIntermediateImage(const cv::Mat& image,
                                           const std::string& filename) {
  
  if (!imwrite(filename, image)) {
    LOG(ERROR) << "无法保存中间图像: " << filename;
    return;
  }
  
  LOG(INFO) << "中间图像已保存: " << filename;
}

}  // namespace cartographer_ros