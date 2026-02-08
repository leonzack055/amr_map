#ifndef CARTOGRAPHER_ROS_COLLECTED_POINTCLOUD_H_
#define CARTOGRAPHER_ROS_COLLECTED_POINTCLOUD_H_

#include <vector>
#include <memory>
#include "cartographer_ros/line_feature.h"
#include "cartographer/mapping/pose_graph.h"
#include <pcl/point_types.h>
#include <pcl/sse.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/2d/convolution.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core.hpp>
#include <random>
namespace cartographer_ros
{
class CollectedPointCloudFrom2d
{
public:
  explicit CollectedPointCloudFrom2d();
  ~CollectedPointCloudFrom2d() = default;
  bool collectPointCloud(cartographer::mapping::PoseGraphInterface* posegraph_ptr);
  bool savePointCloud(const std::string& file_name);
  bool filterPointCloud(const double& leaf_size);
  bool imageFilterPointCloud(const double& resolution, const cv::Mat& seg_img);
  void getLineFeatures();
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
  std::shared_ptr<cartographer::mapping::PoseGraphInterface> pose_graph_;
};
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_COLLECTED_POINTCLOUD_H_