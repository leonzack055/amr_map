#include <cartographer/sensor/point_cloud.h>
#include <cartographer_ros/collected_pointcloud.h>
#include <glog/logging.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <pcl/filters/impl/bilateral.hpp>
#include <random>
#include <string>

namespace cartographer_ros {
using namespace cartographer::mapping;
using namespace cartographer::transform;

pcl::PointCloud<pcl::PointXYZ>::Ptr
ConvertCartographerPointCloudToPCLPointCloud(
    const cartographer::sensor::PointCloud& point_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  // TODO: compressed_point_cloud 没有 intensity
  // 数据，所以在serialize_data过程中没有保存intensity数据
  // if(point_cloud.intensities().empty()) {
  //   LOG(FATAL) << "No intensity data in point cloud";
  // }
  for (int i = 0; i < point_cloud.size(); i++) {
    pcl_point_cloud->points.push_back(
        pcl::PointXYZ(point_cloud.points()[i].position.x(),
                      point_cloud.points()[i].position.y(),
                      point_cloud.points()[i].position.z()));
  }
  return pcl_point_cloud;
}

CollectedPointCloudFrom2d::CollectedPointCloudFrom2d() {}

bool CollectedPointCloudFrom2d::collectPointCloud(
    cartographer::mapping::PoseGraphInterface* posegraph_ptr) {
  point_cloud_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // get all nodes pointclouds
  auto trajectory_nodes = posegraph_ptr->GetTrajectoryNodes();
  for (const auto& node_id_data : trajectory_nodes) {
    NodeId node_id = node_id_data.id;
    Rigid3d node_pose = node_id_data.data.global_pose;
    Rigid3d gravityRotaion =
        Rigid3d::Rotation(node_id_data.data.constant_data->gravity_alignment);
    Rigid3d node_pose_wi_gravity = node_pose * gravityRotaion.inverse();
    // pointcloud
    // if (node_id_data.data.constant_data->filtered_gravity_aligned_point_cloud
    //         .intensities()
    //         .size() > 0) {
    //   LOG(INFO) << "node_id: " << node_id.trajectory_id << ":"
    //             << node_id.node_index << " has intensity";
    // } else {
    //   LOG(ERROR) << "node_id: " << node_id.trajectory_id << ":"
    //              << node_id.node_index << " has no intensity";
    // }

    cartographer::sensor::PointCloud node_pointcloud =
        cartographer::sensor::TransformPointCloud(
            node_id_data.data.constant_data
                ->filtered_gravity_aligned_point_cloud,
            node_pose_wi_gravity.cast<float>());
    auto pcl_node_pointcloud =
        ConvertCartographerPointCloudToPCLPointCloud(node_pointcloud);
    point_cloud_->insert(point_cloud_->end(), pcl_node_pointcloud->begin(),
                         pcl_node_pointcloud->end());
  }
  if (point_cloud_->size() == 0) {
    LOG(INFO) << "No pointcloud collected";
    return false;
  }
  return true;
}

bool CollectedPointCloudFrom2d::filterPointCloud(const double& leaf_size) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // ========== 2. 配置统计滤波 ==========
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(point_cloud_);  // 设置输入点云
  sor.setMeanK(100);  // 邻域点数量（核心参数，推荐20-100）
  sor.setStddevMulThresh(2.0);  // 标准差倍数（核心参数，推荐1.0-2.0）
  // sor.setNegative(true);               //
  // 可选：反向滤波（保留离群点，用于验证）
  sor.filter(*filtered_pointcloud);  // 执行滤波

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(filtered_pointcloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*filtered_pointcloud);
  // 配置半径滤波
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
  ror.setInputCloud(filtered_pointcloud);  // 输入点云
  ror.setRadiusSearch(0.25);  // 搜索半径（单位：米，核心参数）
  ror.setMinNeighborsInRadius(10);  // 半径内最小点数（核心参数）
  // ror.setNegative(true);               // 可选：反向滤波
  ror.filter(*point_cloud_);
  // 双边滤波查找
  // pcl::BilateralFilter<pcl::PointXYZ> bf;
  // bf.setInputCloud(filtered_pointcloud);
  // bf.setHalfSize(0.05);          // 空间域半径（单位：米，核心参数）
  // bf.setStdDev(0.1);             // 值域标准差（核心参数）
  // bf.filter(*point_cloud_);
  return true;
}

bool CollectedPointCloudFrom2d::savePointCloud(const std::string& file_name) {
  point_cloud_->width = point_cloud_->points.size();
  point_cloud_->height = 1;
  point_cloud_->is_dense = false;
  bool save_xyz_success = pcl::io::savePCDFile(file_name, *point_cloud_, true);
  if (save_xyz_success) {
    std::cout << "普通 PointXYZ 点云保存 PCD 成功！" << std::endl;
  } else {
    std::cerr << "普通 PointXYZ 点云保存 PCD 失败！" << std::endl;
    return -1;
  }

  return true;
}

bool CollectedPointCloudFrom2d::imageFilterPointCloud(const double& resolution,
                                                      const cv::Mat& seg_img) {
  return false;
}
void CollectedPointCloudFrom2d::getLineFeatures() {}

}  // namespace cartographer_ros