#ifndef POSE_INTERPOLATOR_NODE_HPP
#define POSE_INTERPOLATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <Eigen/Geometry>
#include <rclcpp/timer.hpp>
// 包含自定义消息头文件
#include "amr_ros_msg/msg/global_pose.hpp"
#include <std_msgs/msg/u_int8.hpp>  // 添加UInt8消息头文件

class PoseInterpolator : public rclcpp::Node {
public:
  PoseInterpolator();

private:
  void tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void qr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void relocate_state_callback(const std_msgs::msg::UInt8::SharedPtr msg);  // 添加状态回调
  void timer_callback();
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qr_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr relocate_state_sub_;  // 状态订阅器
  rclcpp::Publisher<amr_ros_msg::msg::GlobalPose>::SharedPtr global_pose_pub_;  // 修改为自定义消息类型
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_tracked_pose_;
  geometry_msgs::msg::PoseStamped last_qr_pose_;
  nav_msgs::msg::Odometry last_odom_;
  nav_msgs::msg::Odometry ref_odom_;
  rclcpp::Time last_tracked_pose_time_;
  bool ref_odom_initialized_;
  bool data_valid_;
  bool qr_pose_active_;  // QR位姿激活标志
  uint8_t current_relocate_state_;  // 当前重定位状态
  
  // Transformation helpers
  Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose);
  geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d& transform);
};

#endif  // POSE_INTERPOLATOR_NODE_HPP