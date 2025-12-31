#include "cartographer_ros/cv_lsd_detector.h"
#include "cartographer_ros/line_feature.h"
#include "Eigen/Core"
#include <opencv2/opencv.hpp>

namespace cartographer_ros
{
CvLsdDetector::CvLsdDetector(const LSDConfig& config) : config_(config)
{
}
std::vector<LineFeature> CvLsdDetector::DetectLines(const cv::Mat& image, double resolution)
{
  cv::Ptr detector_ = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
  std::vector<cv::Vec4f> lines;
  detector_->detect(image, lines); // 核心检测函数
  std::vector<LineFeature> featureLines;
  cv::Mat line_img;
  cv::cvtColor(image, line_img, cv::COLOR_GRAY2BGR); // 灰度图转彩色图，用于绘制彩色直线
  detector_->drawSegments(line_img, lines);       // 绘制直线段（默认红色，可自定义）
  cv::imshow("lsd", line_img); // 显示直线检测结果
  cv::waitKey(0);
  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4f line = lines[i];
    // 提取端点（强制转为 int 类型，因为坐标是像素整数）
    int x1 = static_cast<int>(line[0]);
    int y1 = static_cast<int>(line[1]);
    int x2 = static_cast<int>(line[2]);
    int y2 = static_cast<int>(line[3]);
    LineFeature featureLine = LineFeature(Eigen::Vector2d(x1,y1), Eigen::Vector2d(x2, y2));
    featureLine.type = LineType::STRUCTURE;
    featureLines.push_back(featureLine);
  }
  cv::imwrite("06-lsd.png", line_img);
  return featureLines;
}
}  // namespace cartographer_ros