#include "pose_interpolator/pose_interpolator_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/u_int8.hpp>  // 添加UInt8消息头文件

using namespace std::chrono_literals;

PoseInterpolator::PoseInterpolator()
: Node("pose_interpolator"),
  ref_odom_initialized_(false),
  data_valid_(false),
  qr_pose_active_(false),
  current_relocate_state_(0) {  // 初始化为0
  
  // Parameters
  this->declare_parameter("output_frequency", 50.0);
  double frequency = this->get_parameter("output_frequency").as_double();
  
  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Subscribers
  tracked_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/tracked_pose", 10,
    std::bind(&PoseInterpolator::tracked_pose_callback, this, std::placeholders::_1));
  
  // QR pose subscriber
  qr_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/global_pose_qr", 10,
    std::bind(&PoseInterpolator::qr_pose_callback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 100,
    std::bind(&PoseInterpolator::odom_callback, this, std::placeholders::_1));
  
  // 添加状态订阅器
  relocate_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "/set_init_relocate_state", 10,
    std::bind(&PoseInterpolator::relocate_state_callback, this, std::placeholders::_1));
  
  // Publisher - changed to GlobalPose type
  global_pose_pub_ = this->create_publisher<amr_ros_msg::msg::GlobalPose>(
    "/global_pose", 10);

  // Timer for interpolation
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / frequency),
    std::bind(&PoseInterpolator::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Pose interpolator node initialized");
}

void PoseInterpolator::relocate_state_callback(
  const std_msgs::msg::UInt8::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_relocate_state_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "收到重定位状态: %d", current_relocate_state_);
}

void PoseInterpolator::tracked_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (qr_pose_active_) {
    return;
  }
  
  // Update reference data
  last_tracked_pose_ = *msg;
  last_tracked_pose_time_ = msg->header.stamp;
  
  if (last_odom_.header.stamp.sec != 0) {
    ref_odom_ = last_odom_;
    ref_odom_initialized_ = true;
    data_valid_ = true;
  }
}

void PoseInterpolator::qr_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  qr_pose_active_ = true;
  last_qr_pose_ = *msg;
  
  // 创建并发布GlobalPose消息
  auto global_msg = amr_ros_msg::msg::GlobalPose();
  global_msg.global_pose = *msg;
  global_msg.global_pose.header.stamp = this->now();  // 更新时间戳
  global_msg.relocate_state = current_relocate_state_;  // 使用当前状态值
  global_pose_pub_->publish(global_msg);
  
  RCLCPP_INFO(this->get_logger(), "Switched to QR pose source");
}

void PoseInterpolator::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg) {
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (qr_pose_active_) {
    return;
  }
  
  last_odom_ = *msg;
}

void PoseInterpolator::timer_callback() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (qr_pose_active_) {
    // 创建并发布GlobalPose消息
    auto global_msg = amr_ros_msg::msg::GlobalPose();
    global_msg.global_pose = last_qr_pose_;
    global_msg.global_pose.header.stamp = this->now();  // 更新时间戳
    global_msg.relocate_state = current_relocate_state_;  // 使用当前状态值
    global_pose_pub_->publish(global_msg);
    return;
  }
  
  // 以下为原始插值逻辑
  if (!data_valid_ || !ref_odom_initialized_) {
    return;
  }
  
  auto now = this->get_clock()->now();
  double dt = (now - last_tracked_pose_time_).seconds();
  
  if (dt < 0.02) {
    return;
  }
  
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      last_tracked_pose_.header.frame_id,
      ref_odom_.header.frame_id,
      tf2::TimePointZero);
    
    Eigen::Isometry3d T_map_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d T_odom_base_ref = pose_to_eigen(ref_odom_.pose.pose);
    Eigen::Isometry3d T_odom_base_current = pose_to_eigen(last_odom_.pose.pose);
    
    Eigen::Isometry3d T_ref_current = T_odom_base_ref.inverse() * T_odom_base_current;
    
    Eigen::Isometry3d T_map_base_ref = pose_to_eigen(last_tracked_pose_.pose);
    Eigen::Isometry3d T_map_base_current = T_map_base_ref * T_map_odom * T_ref_current * T_map_odom.inverse();
    
    // 创建GlobalPose消息
    auto global_msg = amr_ros_msg::msg::GlobalPose();
    global_msg.global_pose.header.stamp = now;
    global_msg.global_pose.header.frame_id = last_tracked_pose_.header.frame_id;
    global_msg.global_pose.pose = eigen_to_pose(T_map_base_current);
    global_msg.relocate_state = current_relocate_state_;  // 使用当前状态值
    
    // Publish
    global_pose_pub_->publish(global_msg);
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
  }
}

Eigen::Isometry3d PoseInterpolator::pose_to_eigen(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x,
    pose.position.y,
    pose.position.z);
  transform.linear() = Eigen::Quaterniond(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z).toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose PoseInterpolator::eigen_to_pose(const Eigen::Isometry3d& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  
  Eigen::Quaterniond q(transform.linear());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  
  return pose;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseInterpolator>());
  rclcpp::shutdown();
  return 0;
}