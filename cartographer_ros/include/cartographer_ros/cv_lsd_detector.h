#ifndef CARTOGRAPHER_ROS_CV_LSD_DETECTOR_H_
#define CARTOGRAPHER_ROS_CV_LSD_DETECTOR_H_

#include <vector>
#include <memory>
#include "cartographer_ros/line_feature.h"
#include <opencv2/opencv.hpp>
#include <random>
namespace cartographer_ros
{
class CvLsdDetector
{
public:
  explicit CvLsdDetector(const LSDConfig& config);
  ~CvLsdDetector() = default;
  std::vector<LineFeature> DetectLines(const cv::Mat& image, double resolution);

private:
  std::mt19937 random_generator_;
  LSDConfig config_;
  cv::Ptr<cv::LineSegmentDetector> lsd_detector_;
};
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CV_LSD_DETECTOR_H_