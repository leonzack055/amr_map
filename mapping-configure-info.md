# Cartographer ROS Lua 参数配置说明文档

> 本文档基于 [`lifecycle_offline_reflector_node.cpp`](cartographer_ros/src/lifecycle_offline_reflector_node.cpp) 回溯遍历建图应用，重点介绍 Cartographer Lua 配置文件中直接影响建图质量的核心参数及其调优策略。

---

## 目录

1. [配置文件结构](#配置文件结构)
2. [核心参数优先级速查](#核心参数优先级速查)
3. [TRAJECTORY_BUILDER_2D 参数](#trajectory_builder_2d-参数)
4. [POSE_GRAPH 参数](#pose_graph-参数)
5. [MAP_BUILDER 参数](#map_builder-参数)
6. [反光柱建图特殊配置](#反光柱建图特殊配置)
7. [问题诊断与参数调节指南](#问题诊断与参数调节指南)
8. [参数联动调节策略](#参数联动调节策略)

---

## 配置文件结构

```
configuration_files/
├── map_builder.lua           # 地图构建器配置
│   └── pose_graph.lua        # 位姿图配置（被引用）
├── trajectory_builder.lua    # 轨迹构建器配置
│   ├── trajectory_builder_2d.lua  # 2D 轨迹配置
│   └── trajectory_builder_3d.lua  # 3D 轨迹配置
└── offline_bdy_amr2.lua      # 项目主配置文件
```

### 文件依赖关系

```lua
-- offline_bdy_amr2.lua
include "map_builder.lua"      -- 包含 pose_graph.lua
include "trajectory_builder.lua" -- 包含 trajectory_builder_2d.lua

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  -- ... 其他参数
}
```

---

## 核心参数优先级速查

### ⭐⭐⭐ 最重要参数（直接影响建图成败）

| 参数 | 位置 | 默认值 | 影响 |
|------|------|--------|------|
| `resolution` | submaps | 0.05 | 地图分辨率，决定精度与内存平衡 |
| `min_score` | constraint_builder | 0.55 | 回环检测阈值，过低导致误检，过高导致漏检 |
| `num_range_data` | submaps | 90 | 子图大小，影响前端累积误差 |
| `occupied_space_weight` | ceres_scan_matcher | 1.0 | 激光匹配权重，影响前端跟踪质量 |

### ⭐⭐ 重要参数（显著影响建图质量）

| 参数 | 位置 | 默认值 | 影响 |
|------|------|--------|------|
| `optimize_every_n_nodes` | POSE_GRAPH | 90 | 后端优化频率 |
| `max_constraint_distance` | constraint_builder | 15.0 | 约束搜索距离 |
| `hit_probability` | probability_grid | 0.55 | 障碍物确认速度 |
| `translation_weight` | ceres_scan_matcher | 10.0 | 平移约束强度 |
| `odometry_translation_weight` | optimization_problem | 1e5 | 里程计信任程度 |

### ⭐ 辅助参数（微调优化）

| 参数 | 位置 | 默认值 | 影响 |
|------|------|--------|------|
| `voxel_filter_size` | TRAJECTORY_BUILDER_2D | 0.025 | 点云降采样粒度 |
| `global_sampling_ratio` | POSE_GRAPH | 0.003 | 全局回环采样率 |
| `huber_scale` | optimization_problem | 1e1 | 异常值鲁棒性 |

---

## TRAJECTORY_BUILDER_2D 参数

### 完整参数结构

```lua
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = 0.,
  max_range = 30.,
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.025,

  adaptive_voxel_filter = { ... },
  loop_closure_adaptive_voxel_filter = { ... },
  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = { ... },
  ceres_scan_matcher = { ... },
  motion_filter = { ... },
  imu_gravity_time_constant = 10.,
  pose_extrapolator = { ... },
  submaps = { ... },
}
```

---

### 1. 激光范围过滤参数

**配置位置**: `TRAJECTORY_BUILDER_2D`

```lua
min_range = 0.              -- 最小有效距离（米）
max_range = 30.             -- 最大有效距离（米）
min_z = -0.8                -- 最小 Z 值（过滤地面）
max_z = 2.                  -- 最大 Z 值（过滤天花板）
missing_data_ray_length = 5.  -- 缺失数据射线长度
```

| 参数 | 作用 | 调节建议 |
|------|------|----------|
| `min_range` | 过滤近距离噪点（机器人本体反射） | 根据激光安装位置设置，通常 0.3-0.5m |
| `max_range` | 过滤远距离低置信度点 | 根据激光规格设置，过大可能导致边缘模糊 |
| `min_z` / `max_z` | 过滤 3D 点云中的地面和天花板 | 2D 激光可忽略 |
| `missing_data_ray_length` | 处理激光缺失区域 | 设为极小值可忽略缺失数据 |

**问题诊断**：
- **地图边缘模糊** → 减小 `max_range`
- **近处障碍物丢失** → 减小 `min_range`

---

### 2. 体素滤波器参数

```lua
voxel_filter_size = 0.025   -- 体素滤波大小（米）,进入后端前进一步降采样
```

**作用**：对原始点云进行降采样，减少计算量。

| 值 | 影响 |
|----|------|
| 小值（0.01-0.025） | 保留更多细节，计算量大 |
| 大值（0.05-0.1） | 降采样明显，计算快，可能丢失细节 |

---

### 3. 自适应体素滤波器

```lua
adaptive_voxel_filter = {
  max_length = 0.5,         -- 体素最大边长
  min_num_points = 200,     -- 最小点数
  max_range = 50.,          -- 最大距离
}
```

**作用**：动态调整体素大小，确保每个体素内有足够点数用于扫描匹配。

| 参数 | 调节建议 |
|------|----------|
| `max_length` | 值越小，点云越密集，匹配精度越高 |
| `min_num_points` | 扫描匹配所需最小点数，环境空旷时减小 |

```

---

### 4. 实时相关扫描匹配器

```lua
use_online_correlative_scan_matching = false  -- 是否启用

real_time_correlative_scan_matcher = {
  linear_search_window = 0.1,              -- 线性搜索窗口（米）
  angular_search_window = math.rad(20.),   -- 角度搜索窗口（弧度）
  translation_delta_cost_weight = 1e-1,    -- 平移代价权重
  rotation_delta_cost_weight = 1e-1,       -- 旋转代价权重
}
```

**作用**：在 Ceres 优化前进行粗略匹配，提供更好的初始值。

| 场景 | 建议 |
|------|------|
| 里程计质量差 | 启用此选项 |
| 前端建图匹配差 | 启用此选项 |
| 计算资源有限 | 禁用此选项 |
| 快速移动 | 增大搜索窗口 |

---

### 5. Ceres 扫描匹配器 ⭐ 重要

```lua
ceres_scan_matcher = {
  occupied_space_weight = 1.,    -- 占用空间权重
  translation_weight = 10.,      -- 平移约束权重
  rotation_weight = 40.,         -- 旋转约束权重
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 20,
    num_threads = 1,
  },
}
```

**作用**：将当前扫描与局部地图进行匹配，计算位姿增量。

| 参数 | 作用 | 调节策略 |
|------|------|----------|
| `occupied_space_weight` | 激光点与地图匹配程度 | 增大可提高匹配精度，但可能陷入局部最优 |
| `translation_weight` | 限制平移变化 | 里程计可靠时增大（如 20-30） |
| `rotation_weight` | 限制旋转变化 | IMU 不可靠时增大 |

**调优示例**：
```lua
-- 提高匹配精度
occupied_space_weight = 1e1
translation_weight = 30
rotation_weight = 40
```

---

### 6. 运动滤波器

```lua
motion_filter = {
  max_time_seconds = 5.,              -- 最大时间间隔
  max_distance_meters = 0.2,          -- 最大移动距离
  max_angle_radians = math.rad(10.),   -- 最大旋转角度
}
```

**作用**：只有超过阈值才插入新数据，过滤静态冗余数据。

| 问题 | 调节方向 |
|------|----------|
| 机器人移动缓慢，地图更新慢 | 增大 `max_time_seconds`减少子图数量 |
| 静态场景数据过多 | 增大 `max_distance_meters` |
| 转向时数据丢失 | 减小 `max_angle_radians` |

---

### 7. 子图参数 ⭐ 重要

```lua
submaps = {
  num_range_data = 90,                -- 每个子图包含的扫描帧数
  grid_options_2d = {
    grid_type = "PROBABILITY_GRID",   -- 栅格类型
    resolution = 0.05,                -- 地图分辨率（米/像素）
  },
  range_data_inserter = {
    range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
    probability_grid_range_data_inserter = {
      insert_free_space = true,
      hit_probability = 0.55,         -- 击中概率
      miss_probability = 0.49,        -- 未击中概率
    },
  },
}
```

#### 7.1 resolution（分辨率）⭐

| 值 | 影响 | 适用场景 |
|----|------|----------|
| 0.02-0.05 | 高精度，细节丰富，内存大 | 小范围高精度建图 |
| 0.05-0.10 | 平衡 | 通用场景（推荐） |
| 0.10-0.20 | 低精度，内存友好 | 大范围粗略建图 |

#### 7.2 num_range_data（子图大小）

| 值 | 影响 |
|----|------|
| 30-60 | 子图小，更新快，适合动态环境，但motion_filter过大时pose_graph的误差可能扭曲 |
| 60-90 | 平衡（推荐） |
| 90-120 | 子图大，前端误差小，适合静态环境; 前端局部内扭曲可能会导致无法形成全连图 |

#### 7.3 概率更新参数

| 参数 | 作用 |
|------|------|
| `hit_probability` | 增大则障碍物更易被确认 |
| `miss_probability` | 减小则空闲区域更易被确认 |

**调优示例**：
```lua
-- 障碍物轮廓更清晰
hit_probability = 0.60
miss_probability = 0.49

-- 动态障碍物残留更少
hit_probability = 0.50
miss_probability = 0.48
```
😺😺😺 **·直接影响地图质量·由前端匹配主导影响** 😺😺😺

---

### 8. 运动滤波器详解

运动滤波器决定了何时将新的扫描数据插入子图，对建图质量有重要影响：

```lua
motion_filter = {
  max_time_seconds = 5.,              -- 最大时间间隔（秒）
  max_distance_meters = 0.2,          -- 最大移动距离（米）
  max_angle_radians = math.rad(1.),   -- 最大旋转角度（弧度）
}
```

**工作原理**：只有当以下任一条件满足时，才会插入新数据：
- 距离上次插入超过 `max_time_seconds` 秒
- 移动距离超过 `max_distance_meters` 米
- 旋转角度超过 `max_angle_radians` 弧度

**调节建议**：

| 场景 | 参数调节 | 原因 |
|------|----------|------|
| 机器人移动缓慢 | 减小 `max_time_seconds` (如 2-3s) | 确保静态时也有数据更新 |
| 高速移动 | 增大 `max_distance_meters` (如 0.3-0.5m) | 避免数据过于稀疏 |
| 频繁转向 | 减小 `max_angle_radians` (如 math.rad(5.)) | 捕捉转向细节 |
| 数据量过大 | 增大所有阈值 | 减少冗余数据，降低计算量 |

**常见问题诊断**：
- **转向处地图断裂** → 减小 `max_angle_radians`
- **子图数量过多** → 增大 `max_distance_meters` 和 `max_angle_radians`
- **静态区域地图空白** → 减小 `max_time_seconds`

---

### 9. IMU 相关参数

```lua
use_imu_data = true                    -- 是否使用 IMU
imu_gravity_time_constant = 10.        -- IMU 重力时间常数
pose_extrapolator = {
  use_imu_based = false,               -- 是否使用基于 IMU 的外推器
  constant_velocity = {
    imu_gravity_time_constant = 10.,
    pose_queue_duration = 0.001,
  },
  imu_based = { ... },
}
```

| 场景 | 建议 |
|------|------|
| 无 IMU 或 IMU 不可靠 | `use_imu_data = false` |
| IMU 质量好 | `use_imu_based = true` |

---

## POSE_GRAPH 参数

### 完整参数结构

```lua
POSE_GRAPH = {
  optimize_every_n_nodes = 90,
  constraint_builder = { ... },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  optimization_problem = { ... },
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,
}
```

---

### 1. 约束构建器 ⭐ 重要

```lua
constraint_builder = {
  sampling_ratio = 0.3,                -- 采样率
  max_constraint_distance = 15.,       -- 最大约束距离
  min_score = 0.55,                    -- 最小匹配分数阈值
  global_localization_min_score = 0.6, -- 全局定位最小分数
  loop_closure_translation_weight = 1.1e4,
  loop_closure_rotation_weight = 1e5,
  log_matches = true,
  
  fast_correlative_scan_matcher = {
    linear_search_window = 7.,         -- 线性搜索窗口
    angular_search_window = math.rad(30.),  -- 角度搜索窗口
    branch_and_bound_depth = 7,        -- 分支定界深度
  },
  
  ceres_scan_matcher = {
    occupied_space_weight = 20.,
    translation_weight = 10.,
    rotation_weight = 1.,
    ceres_solver_options = { ... },
  },
}
```

#### 1.1 min_score（回环检测阈值）⭐

| 值 | 影响 |
|----|------|
| 0.50-0.55 | 宽松，回环检测多，可能有误检 |
| 0.55-0.65 | 平衡（推荐） |
| 0.65-0.75 | 严格，回环检测少，更可靠 |

**问题诊断**：
- **回环检测失败** → 降低 `min_score`
- **误检测导致地图扭曲** → 提高 `min_score`

#### 1.2 搜索窗口参数

```lua
linear_search_window = 7.              -- 米
angular_search_window = math.rad(30.)  -- 弧度（约 17 度）
```

| 场景 | 调节方向 |
|------|----------|
| 累积误差大 | 增大搜索窗口 |
| 计算资源有限 | 减小搜索窗口 |
| 环境相似度高（走廊） | 减小角度窗口 |

#### 1.3 branch_and_bound_depth

| 值 | 影响 |
|----|------|
| 小值（5-6） | 搜索快，可能漏检 |
| 大值（7-8） | 搜索慢，更准确 |

---

### 2. 优化问题参数 ⭐ 重要

```lua
optimization_problem = {
  huber_scale = 1e1,                        -- Huber 损失尺度
  acceleration_weight = 1.1e2,              -- 加速度权重
  rotation_weight = 1.6e4,                  -- 旋转权重
  local_slam_pose_translation_weight = 1e5, -- 局部(前端) SLAM 平移权重
  local_slam_pose_rotation_weight = 1e5,    -- 局部(前羰) SLAM 旋转权重
  odometry_translation_weight = 1e5,        -- 里程计平移权重
  odometry_rotation_weight = 1e5,           -- 里程计旋转权重
  fixed_frame_pose_translation_weight = 1e1,
  fixed_frame_pose_rotation_weight = 1e2,
  log_solver_summary = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 50,
    num_threads = 7,
  },
}
```

#### 2.1 权重配置原则

| 数据源 | 参数 | 可靠时建议 |
|--------|------|------------|
| 里程计 | `odometry_translation_weight` | 增大（1e5-1e6） |
| 里程计 | `odometry_rotation_weight` | 增大（1e4-1e5） |
| 局部 SLAM | `local_slam_pose_*_weight` | 增大（1e5） |
| IMU | `rotation_weight` | 增大（1e5） |

#### 2.2 huber_scale

**作用**：控制 Huber 损失函数对异常值的敏感度。

| 值 | 影响 |
|----|------|
| 小值 | 对异常值敏感，优化激进 |
| 大值 | 对异常值鲁棒，优化保守 |

---

### 3. 优化频率参数

```lua
optimize_every_n_nodes = 90        -- 每N个节点优化一次
max_num_final_iterations = 200     -- 最终优化迭代次数
global_constraint_search_after_n_seconds = 10.  -- 全局约束搜索间隔
```

| 参数 | 值 | 影响 |
|------|-----|------|
| `optimize_every_n_nodes` | 30-60 | 频繁优化，实时性好，开销大 |
| `optimize_every_n_nodes` | 90-120 | 批量优化，适合离线建图 |
| `max_num_final_iterations` | 100-500 | 最终优化质量 |

---

### 4. 全局采样参数

```lua
global_sampling_ratio = 0.003      -- 全局约束采样率,进行分支定界搜索匹配
constraint.sampling_ratio = 0.3               -- 局部约束采样率，直接初值匹配
```

**作用**：控制回环检测的采样密度。

| 场景 | 调节方向 |
|------|----------|
| 大环境建图,子图质量好但无法形成闭环 | 增大 `global_sampling_ratio`,但性能指数下降 |
| 回环检测不足 | 增大 `constraint.sampling_ratio` |

---

## 反光柱建图特殊配置

基于 [`lifecycle_offline_reflector_node.cpp`](cartographer_ros/src/lifecycle_offline_reflector_node.cpp) 的反光柱建图应用，有以下特殊配置要点：

### 1. 反光柱作为 Landmark 约束

反光柱检测结果会作为 Landmark 加入位姿图优化：

```cpp
// 在 convertTrackedReflectorsToLandmarkList() 中
entry.translation_weight = weight * landmark_translation_weight_;  // 默认 1e5
entry.rotation_weight = 0.0;  // 点状路标不约束旋转
```

**相关参数配置**：

```lua
-- 在 optimization_problem 中配置路标权重
-- 这些权重影响反光柱约束的强度
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
```

### 2. 离线建图流程参数

离线建图使用两次遍历：
1. **第一遍**：正常 Cartographer 建图，生成初始轨迹
2. **第二遍**：回溯处理激光数据，检测反光柱，加入约束

**关键配置**：

```lua
-- 离线建图推荐配置
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 单帧处理
TRAJECTORY_BUILDER_2D.use_imu_data = false            -- 无 IMU 时关闭

-- 优化频率适当提高，确保轨迹质量
POSE_GRAPH.optimize_every_n_nodes = 15  -- 更频繁优化
```

### 3. 反光柱检测参数（ROS 参数）

这些参数通过 ROS 参数服务器配置，不影响 Cartographer 核心，但影响反光柱检测质量：

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `raw_intensity_threshold` | 强度阈值过滤反光柱 | 1000.0 |
| `fixed_dbscan.eps` | DBSCAN 聚类距离 | 0.05 |
| `fixed_dbscan.min_points` | 最小聚类点数 | 5 |
| `global_tracking.match_distance_threshold` | 跟踪匹配距离 | 0.3 |
| `global_tracking.min_detections_in_window` | 确认所需检测次数 | 8 |

### 4. 当前项目配置分析

查看 [`offline_bdy_amr2.lua`](cartographer_ros/configuration_files/offline_bdy_amr2.lua)：

```lua
-- 当前配置特点分析

-- 1. 高精度匹配配置
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2     -- 更密集的点
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400 -- 更多点用于匹配

-- 2. 较小的子图（适合回环检测）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45  -- 比默认 90 小

-- 3. 较严格的回环检测
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 比默认 0.55 更严格

-- 4. 频繁优化
POSE_GRAPH.optimize_every_n_nodes = 15  -- 比默认 90 更频繁
```

**配置评估**：
- ✅ 适合小范围、高精度建图
- ⚠️ 大范围建图可能需要增大 `num_range_data`
- ⚠️ 严格回环阈值可能导致漏检，需根据环境调整

---

## MAP_BUILDER 参数

```lua
MAP_BUILDER = {
  use_trajectory_builder_2d = false,   -- 是否使用 2D 建图
  use_trajectory_builder_3d = false,   -- 是否使用 3D 建图
  num_background_threads = 4,          -- 后台线程数
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}
```

| 参数 | 建议 |
|------|------|
| `use_trajectory_builder_2d` | 2D 建图设为 `true` |
| `num_background_threads` | 根据 CPU 核心数设置（通常 4-8） |

---

## 参数联动调节策略

参数之间相互关联，调节时需要考虑联动效应：

### 策略 1：提高前端匹配精度

```
┌─────────────────────────────────────────────────────────────┐
│ 目标：减少前端漂移                                            │
├─────────────────────────────────────────────────────────────┤
│ 联动调节：                                                    │
│ 1. ↑ occupied_space_weight (1e1 → 2e1)                      │
│ 2. ↑ adaptive_voxel_filter.min_num_points (200 → 400)       │
│ 3. ↓ voxel_filter_size (0.025 → 0.015)                      │
│ 4. ↑ translation_weight (10 → 30)                           │
├─────────────────────────────────────────────────────────────┤
│ 副作用：计算量增加，建图速度下降                               │
└─────────────────────────────────────────────────────────────┘
```

### 策略 2：增强回环检测能力

```
┌─────────────────────────────────────────────────────────────┐
│ 目标：提高回环检测成功率                                      │
├─────────────────────────────────────────────────────────────┤
│ 联动调节：                                                    │
│ 1. ↓ min_score (0.55 → 0.50)                                │
│ 2. ↑ linear_search_window (7 → 10)                          │
│ 3. ↑ global_sampling_ratio (0.003 → 0.01)                   │
│ 4. ↑ branch_and_bound_depth (7 → 8)                         │
├─────────────────────────────────────────────────────────────┤
│ 副作用：可能产生误检，计算时间显著增加                         │
└─────────────────────────────────────────────────────────────┘
```

### 策略 3：大范围建图优化

```
┌─────────────────────────────────────────────────────────────┐
│ 目标：适合大面积环境建图                                      │
├─────────────────────────────────────────────────────────────┤
│ 联动调节：                                                    │
│ 1. ↑ num_range_data (90 → 120)                              │
│ 2. ↑ resolution (0.05 → 0.08)                               │
│ 3. ↑ optimize_every_n_nodes (90 → 120)                      │
│ 4. ↑ max_constraint_distance (15 → 20)                      │
│ 5. ↓ hit_probability (0.55 → 0.52)                          │
├─────────────────────────────────────────────────────────────┤
│ 副作用：地图细节减少，内存占用可能增加                         │
└─────────────────────────────────────────────────────────────┘
```

### 策略 4：动态环境适应

```
┌─────────────────────────────────────────────────────────────┐
│ 目标：减少动态障碍物残留                                      │
├─────────────────────────────────────────────────────────────┤
│ 联动调节：                                                    │
│ 1. ↓ hit_probability (0.55 → 0.50)                          │
│ 2. ↑ miss_probability (0.49 → 0.50)                         │
│ 3. ↓ num_range_data (90 → 60)                               │
│ 4. ↑ max_time_seconds (5 → 3)                               │
├─────────────────────────────────────────────────────────────┤
│ 副作用：静态障碍物确认变慢，地图可能模糊                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 问题诊断与参数调节指南

### 问题 1：地图漂移严重

**症状**：地图与实际环境偏差逐渐增大

**调节方案**：
```lua
-- 增强扫描匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1e1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30

-- 降低回环检测阈值
POSE_GRAPH.constraint_builder.min_score = 0.50

-- 增大搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.

-- 调整submaps.num_range_data
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
```

---

### 问题 2：回环检测失败，地图不闭合

**症状**：回到起点时地图无法正确闭合

**调节方案**：
```lua
-- 降低匹配阈值
POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

-- 增大搜索范围
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)

-- 提高回环约束权重
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e5
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e6
```

---

### 问题 3：地图细节模糊

**症状**：障碍物边缘不清晰

**调节方案**：
```lua
-- 提高分辨率
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03

-- 增加子图数据量
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- 调整概率更新
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.60
```

---

### 问题 4：动态障碍物残留

**症状**：移动物体在地图上留下痕迹

**调节方案**：
```lua
-- 降低击中概率
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.50
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48

-- 减少子图数据量，加快更新
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
```

---

### 问题 5：计算资源不足

**症状**：建图缓慢，CPU 占用过高

**调节方案**：
```lua
-- 降低分辨率
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.08

-- 减少优化频率
POSE_GRAPH.optimize_every_n_nodes = 120

-- 减少采样
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
POSE_GRAPH.global_sampling_ratio = 0.001

-- 减少搜索深度
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 5
```

---

### 问题 6：里程计漂移导致地图错位

**症状**：使用轮式里程计时地图偏移

**调节方案**：
```lua
-- 降低里程计权重
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

-- 增强激光匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2e1
```

---

### 问题 7：转向时地图变形

**症状**：机器人转向时地图出现扭曲

**调节方案**：
```lua
-- 增大旋转约束
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60

-- 启用实时相关扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
```

---

### 问题 8：反光柱约束不生效

**症状**：反光柱检测正常，但地图优化效果不明显

**诊断步骤**：
1. 检查反光柱是否被正确跟踪（查看 `global_tracking` 参数）
2. 检查 Landmark 权重配置
3. 确认反光柱坐标系正确（应为 `map` 坐标系）

**调节方案**：
```lua
-- 增强反光柱约束权重
-- 注意：这些在 C++ 代码中设置，需要修改代码
landmark_translation_weight = 1e6  -- 增大
landmark_rotation_weight = 1e2     -- 点路标不约束旋转

-- 或者在 optimization_problem 中增强局部 SLAM 权重
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e6
```

---

### 问题 9：子图间出现错位

**症状**：相邻子图之间出现明显的偏移或断裂

**原因分析**：
1. 前端匹配质量差
2. 子图边界处数据不足
3. 优化权重配置不当

**调节方案**：
```lua
-- 增强前端匹配
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2e1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 40

-- 调整子图大小
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60  -- 增大子图

-- 增强子图间约束
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e5
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e6

-- 更频繁优化
POSE_GRAPH.optimize_every_n_nodes = 60
```

---

### 问题 10：轨迹末端优化不收敛

**症状**：建图结束时优化无法收敛，地图变形

**调节方案**：
```lua
-- 增加最终优化迭代次数
POSE_GRAPH.max_num_final_iterations = 500

-- 启用求解器日志分析
POSE_GRAPH.optimization_problem.log_solver_summary = true

-- 调整 Huber 损失
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- 增大以更鲁棒

-- Ceres 求解器配置
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true
```

---

## 快速参考表

### 按问题类型查找

| 问题 | 关键参数 | 调节方向 | 优先级 |
|------|----------|----------|--------|
| 地图漂移 | `occupied_space_weight`, `min_score` | 增大 / 降低 | ⭐⭐⭐ |
| 回环失败 | `min_score`, `linear_search_window` | 降低 / 增大 | ⭐⭐⭐ |
| 细节模糊 | `resolution`, `hit_probability` | 减小 / 增大 | ⭐⭐ |
| 动态残留 | `hit_probability`, `miss_probability` | 降低 / 增大 | ⭐⭐ |
| 计算缓慢 | `resolution`, `optimize_every_n_nodes` | 增大 / 增大 | ⭐ |
| 里程漂移 | `odometry_translation_weight` | 减小 | ⭐⭐ |
| 转向变形 | `rotation_weight`, `max_angle_radians` | 增大 / 减小 | ⭐⭐ |
| 子图错位 | `num_range_data`, `occupied_space_weight` | 增大 / 增大 | ⭐⭐⭐ |
| 优化不收敛 | `max_num_final_iterations`, `huber_scale` | 增大 / 增大 | ⭐ |

### 按参数类型查找

| 参数类别 | 主要参数 | 影响范围 |
|----------|----------|----------|
| 分辨率 | `resolution`, `voxel_filter_size` | 精度、内存、速度 |
| 匹配权重 | `occupied_space_weight`, `translation_weight`, `rotation_weight` | 前端跟踪质量 |
| 回环检测 | `min_score`, `linear_search_window`, `global_sampling_ratio` | 全局一致性 |
| 优化权重 | `odometry_*_weight`, `local_slam_*_weight` | 数据融合策略 |
| 子图配置 | `num_range_data`, `hit_probability`, `miss_probability` | 地图质量 |

---

## 相关文件链接

- 主配置文件: [`offline_bdy_amr2.lua`](cartographer_ros/configuration_files/offline_bdy_amr2.lua)
- 位姿图配置: [`pose_graph.lua`](cartographer_ros/configuration_files/pose_graph.lua)
- 2D轨迹配置: [`trajectory_builder_2d.lua`](cartographer_ros/configuration_files/trajectory_builder_2d.lua)
- 地图构建器配置: [`map_builder.lua`](cartographer_ros/configuration_files/map_builder.lua)
- 轨迹构建器配置: [`trajectory_builder.lua`](cartographer_ros/configuration_files/trajectory_builder.lua)

---

> 文档版本: 1.3
> 最后更新: 2026-03-09
> 适用项目: AMR Cartographer 离线反光柱建图系统
