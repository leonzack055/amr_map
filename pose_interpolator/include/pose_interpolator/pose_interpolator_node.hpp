#ifndef POSE_INTERPOLATOR_NODE_HPP
#define POSE_INTERPOLATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h> 
#include <mutex>
#include <Eigen/Geometry>
#include <rclcpp/timer.hpp>
// 包含自定义消息头文件
#include "amr_ros_msg/msg/global_pose.hpp"

#include "amr_ros_msg/msg/pose_with_type_stamped.hpp"
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
  rclcpp::Publisher<amr_ros_msg::msg::PoseWithTypeStamped>::SharedPtr initial_pose_pub_;  // 自动重定位发布器
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_qr_pose_time_;  

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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

  void publish_auto_relocation();
  void start_relocation_process();

  double receive_qr_time_;       // QR稳定接收时间（秒）
  double odom_stop_time_;       // 车速为0持续时间（秒）
  double relocate_freeze_time_;// 重定位冷却时间（秒）
  bool  relocate_frozen_ = true; // 重定位冷却标志
  double limit_range_between_qr_cart_;//反光柱与cartographer的限定距离，超过这个距离才会进行重定位
  bool publish_map_qr_tf_;          // 是否发布map_qr->odom的TF
  double speed_threshold_ = 0.01; // 车速阈值（m/s）
  double qr_timeout_ = 2.0; //
  bool pub_relocate_state_ = false;//是否发布重定位状态，如果发布了，就不再使用外部输入的重定位状态
  rclcpp::Time start_receive_qr_time_;  //开始接收qr数据的时间
  rclcpp::Time last_start_relocation_time_;//开始进行上次重定位的时间
  // Transformation helpers

  Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose);
  geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d& transform);
};

#endif  // POSE_INTERPOLATOR_NODE_HPP