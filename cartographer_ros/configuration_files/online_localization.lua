-- Copyright 2024 [Your Name]
-- 修复的纯定位模式配置

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_pose_extrapolator = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.001,
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1., 
  -- 发布tracking_frame->map的坐标，tf默认是发布的但有风险
  publish_tracked_pose = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 1
MAP_BUILDER.collate_by_trajectory = true

TRAJECTORY_BUILDER.collate_landmarks = false
-- TRAJECTORY_BUILDER.load_state_frozen = true
-- TRAJECTORY_BUILDER.pure_localization = true
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
  }
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 地图大小
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30
-- 节点更新
TRAJECTORY_BUILDER_2D.motion_filter = {
  max_time_seconds = 5.,
  max_distance_meters = 0.2,
  max_angle_radians = math.rad(10.),
}
-- 使用CSM定位
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(15.),
    translation_delta_cost_weight = 1e-2,
    rotation_delta_cost_weight = 1e-1,
  }
-- ceres匹配参数
-- 调整匹配权重
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 15
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 25
TRAJECTORY_BUILDER_2D.ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 20.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 5,
      num_threads = 2,
    },
  }

-- 优化定位参数
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.15
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 15.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.

-- 优化全局约束定位参数
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.5e5
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2e4
-- 全局匹配器
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher = {
  linear_search_window = 5.,
  angular_search_window = math.rad(20.),
  branch_and_bound_depth = 7,
}
POSE_GRAPH.constraint_builder.ceres_scan_matcher = {
  occupied_space_weight = 20.,
  translation_weight = 10.,
  rotation_weight = 1e2,
  ceres_solver_options = {
    use_nonmonotonic_steps = true,
    max_num_iterations = 8,
    num_threads = 4,
  },
}

-- 优化问题设置
POSE_GRAPH.optimization_problem.acceleration_weight = 0.1 * 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 0.1 * 3e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
POSE_GRAPH.matcher_translation_weight = 2e5
POSE_GRAPH.matcher_rotation_weight = 2e5
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.global_constraint_search_after_n_seconds = 10

return options
