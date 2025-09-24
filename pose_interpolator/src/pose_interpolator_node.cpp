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

  last_qr_pose_time_ = this->now();  // 初始化为节点启动时间
  
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Parameters
  this->declare_parameter("output_frequency", 50.0);
  double frequency = this->get_parameter("output_frequency").as_double();
  this->declare_parameter("receive_qr_time", 2.0);      // 默认稳定2
  this->declare_parameter("odom_stop_time", 2.0);        // 
  this->declare_parameter("relocate_freeze_time", 20.0);      // 默认暂停20秒
  this->declare_parameter("limit_range_between_qr_cart", 0.1);
  this->declare_parameter("speed_threshold", 0.01); // 默认车速阈值0.01m/s
  this->declare_parameter("publish_map_qr_tf", false);  // 默认不发布TF变换
  this->declare_parameter("qr_timeout_sec", 2.0);  // 默认发布TF变换
  receive_qr_time_ = this->get_parameter("receive_qr_time").as_double();
  odom_stop_time_ = this->get_parameter("odom_stop_time").as_double();
  relocate_freeze_time_ = this->get_parameter("relocate_freeze_time").as_double();
  limit_range_between_qr_cart_ = this->get_parameter("limit_range_between_qr_cart").as_double();
  speed_threshold_ = this->get_parameter("speed_threshold").as_double();
  publish_map_qr_tf_ = this->get_parameter("publish_map_qr_tf").as_bool();
  qr_timeout_ = this->get_parameter("qr_timeout_sec").as_double();  // 用参数初始化超时时间
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
  initial_pose_pub_ = this->create_publisher<amr_ros_msg::msg::PoseWithTypeStamped>(
    "/initial_pose", 10);
  // Timer for interpolation
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / frequency),
    std::bind(&PoseInterpolator::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Pose interpolator node initialized");
}

void PoseInterpolator::relocate_state_callback(
  const std_msgs::msg::UInt8::SharedPtr msg) {
  current_relocate_state_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "收到重定位状态: %d", current_relocate_state_);
}

void PoseInterpolator::tracked_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    
  // if (qr_pose_active_) {
  //   return;
  // }
  
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
  // if (qr_pose_active_ && (this->now() - last_qr_pose_time_ > qr_timeout_)) {
  //   qr_pose_active_ = false;
  //   RCLCPP_WARN(this->get_logger(), "QR pose timed out, deactivating");
  // }

  // RCLCPP_INFO(this->get_logger(), "i receive a qr pose");

  if (qr_pose_active_ == false)
  {
      // RCLCPP_INFO(this->get_logger(), "qr_pose_active_ is false");

    start_receive_qr_time_ = this->now();
    last_start_relocation_time_ = this->now();
  }
  
  qr_pose_active_ = true;
  last_qr_pose_ = *msg;
  last_qr_pose_time_ = this->now();  // 更新时间戳
  if (qr_pose_active_ && (this->now() - start_receive_qr_time_).seconds() >= receive_qr_time_)
  {
    // RCLCPP_INFO(this->get_logger(), "receive 2 second and start reloc");
    start_relocation_process();
    RCLCPP_INFO(this->get_logger(), "QR pose active and stable for %.1f seconds", receive_qr_time_);
  }
  
}

void PoseInterpolator::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg) {  
  // if (qr_pose_active_) {
  //   return;
  // }
  // RCLCPP_INFO(this->get_logger(), "input   last_odom_.header.stamp.sec %d", msg->header.stamp.sec);
  last_odom_ = *msg;
}



void PoseInterpolator::start_relocation_process() {
// RCLCPP_INFO(this->get_logger(), "im in relocation_process");
  double dist = std::sqrt(
            abs(last_tracked_pose_.pose.position.x - last_qr_pose_.pose.position.x) * abs(last_tracked_pose_.pose.position.x - last_qr_pose_.pose.position.x)+ 
            abs(last_tracked_pose_.pose.position.y - last_qr_pose_.pose.position.y) * abs(last_tracked_pose_.pose.position.y - last_qr_pose_.pose.position.y)
          );
  
  // RCLCPP_INFO(this->get_logger(), "last_tracked_pose_pose.position.x: %f", last_tracked_pose_.pose.position.x);
  // RCLCPP_INFO(this->get_logger(), "last_tracked_pose_.pose.position.y: %f", last_tracked_pose_.pose.position.y);
  // RCLCPP_INFO(this->get_logger(), "last_qr_pose_.pose.position.x: %f", last_qr_pose_.pose.position.x);
  // RCLCPP_INFO(this->get_logger(), "last_qr_pose_.pose.position.y: %f", last_qr_pose_.pose.position.y);
  if (this->now() - last_start_relocation_time_ > rclcpp::Duration::from_seconds(relocate_freeze_time_) && dist > limit_range_between_qr_cart_)
    {        
      pub_relocate_state_ = true;
    }
  // relocation_state_ = uint8(2);
  double speed = std::sqrt(last_odom_.twist.twist.linear.x * last_odom_.twist.twist.linear.x + last_odom_.twist.twist.linear.y * last_odom_.twist.twist.linear.y);
  RCLCPP_INFO(this->get_logger(), "speed %f ",speed);
  if (speed < speed_threshold_)
  {
    RCLCPP_INFO(this->get_logger(), "speed is zero ");


    double timedist = (this->now() - last_start_relocation_time_).seconds();
    RCLCPP_INFO(this->get_logger(), "dist_time: %f", timedist);
    RCLCPP_INFO(this->get_logger(), "relocate_freeze_time_: %f", relocate_freeze_time_);
    RCLCPP_INFO(this->get_logger(), "dist: %f", dist);
    RCLCPP_INFO(this->get_logger(), "limit_range_between_qr_cart_: %f", limit_range_between_qr_cart_);
    if (((this->now() - last_start_relocation_time_).seconds() > relocate_freeze_time_) && (dist > limit_range_between_qr_cart_))
    {
      RCLCPP_INFO(this->get_logger(), "pub reloc mseeage ");
      publish_auto_relocation();
      last_start_relocation_time_ = this->now();
    }
  }

}



void PoseInterpolator::publish_auto_relocation() {
  // 发布自动重定位消息
  auto initial_pose_msg = amr_ros_msg::msg::PoseWithTypeStamped();
  initial_pose_msg.type = "M";
  initial_pose_msg.inital_pose.header.stamp = this->now();
  initial_pose_msg.inital_pose.header.frame_id = "map";
  initial_pose_msg.inital_pose.pose = last_qr_pose_.pose;  // 使用global_pose_qr的位姿
  
  initial_pose_pub_->publish(initial_pose_msg);
  pub_relocate_state_ = false;
  RCLCPP_INFO(this->get_logger(), "发布自动重定位消息,并停止重定位为2的发送");
  RCLCPP_INFO(this->get_logger(), "pub finish ");

}


void PoseInterpolator::timer_callback() {  

  if (qr_pose_active_ && ((this->now() - last_qr_pose_time_).seconds() > qr_timeout_)) {
    qr_pose_active_ = false;
  }
  // RCLCPP_INFO(this->get_logger(), "1 ");
  // RCLCPP_INFO(this->get_logger(), "last_odom_.header.stamp.sec %d", last_odom_.header.stamp.sec);
  if (last_odom_.header.stamp.sec != 0) {
      ref_odom_ = last_odom_;
      ref_odom_initialized_ = true;
      data_valid_ = true;
    }
  if (!data_valid_ || !ref_odom_initialized_) {
    return;
  }
  // RCLCPP_INFO(this->get_logger(), "2 ");
  auto now = this->get_clock()->now();
  double dt = (now - last_qr_pose_.header.stamp).seconds();
  
  if (dt < 0.02) {
    return;
  }
  // RCLCPP_WARN(this->get_logger(), "its going on ");
  try {
    if (qr_pose_active_) {
      // 直接使用QR位姿和里程计数据计算变换
      // RCLCPP_INFO(this->get_logger(), "3 ");
      Eigen::Isometry3d T_map_qr_base_ref = pose_to_eigen(last_qr_pose_.pose);
      Eigen::Isometry3d T_odom_base_ref = pose_to_eigen(ref_odom_.pose.pose);
      Eigen::Isometry3d T_odom_base_current = pose_to_eigen(last_odom_.pose.pose);
      
      Eigen::Isometry3d T_ref_current = T_odom_base_ref.inverse() * T_odom_base_current;
      
      Eigen::Isometry3d T_map_qr_base_current = T_map_qr_base_ref * T_ref_current;
      
      // 创建GlobalPose消息
      auto global_msg = amr_ros_msg::msg::GlobalPose();
      global_msg.global_pose.header.stamp = now;
      global_msg.global_pose.header.frame_id = "map_qr";
      global_msg.global_pose.pose = eigen_to_pose(T_map_qr_base_current);
      if(pub_relocate_state_ == true)
      {
        global_msg.relocate_state = uint(2);
      }
      else
      {
        global_msg.relocate_state = current_relocate_state_;
      }
      
      
      // Publish
      global_pose_pub_->publish(global_msg);
      
      // 发布map_qr到odom的TF变换
      if(publish_map_qr_tf_)
      {
        // RCLCPP_INFO(this->get_logger(), "4 ");
      Eigen::Isometry3d T_map_qr_odom = T_map_qr_base_current * T_odom_base_current.inverse();
      
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = now;
      tf_msg.header.frame_id = "map_qr";
      tf_msg.child_frame_id = "odom";
      tf_msg.transform.translation.x = T_map_qr_odom.translation().x();
      tf_msg.transform.translation.y = T_map_qr_odom.translation().y();
      tf_msg.transform.translation.z = T_map_qr_odom.translation().z();
      
      Eigen::Quaterniond q(T_map_qr_odom.linear());
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      
      tf_broadcaster_->sendTransform(tf_msg);
      }
    } else {
      // RCLCPP_INFO(this->get_logger(), "5 ");
      // 没有qr输入
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
      global_msg.relocate_state = current_relocate_state_;
      
      // Publish
      global_pose_pub_->publish(global_msg);
    }
    
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