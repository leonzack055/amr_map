<!--
Author: LeonZack055 (Gmail)
ros2-mapping-cli.md (c) 2026
Desc: ROS2建图模块命令行工具使用说明文档
@copyright Copyright (c) <LeonZack055> All rights reserved.
@license BSD 2-Clause License
Created:  2026-03-06T06:48:07.533Z
Modified: 2026-03-06
-->

# ROS2 Mapping CLI 使用说明文档

> 建图模块命令行工具提供了反光柱的交互式离线建图命令行工具和`lifecycle_node`离线建图服务命令行工具。
- 可通过命令行工具进行反光柱建图任务，用于Rviz结合进行实时的地图和反光柱可视化，并支持手动前进、后退的帧播放功能。
- `lifecycle_node`离线建图服务命令行工具，用于在终端中进行建图任务，可根据用户指令进行`lifecycle`状态机运行切换，为前端提供建图服务。

---

## 目录

1. [Lifecycle节点离线建图服务](#1-lifecycle节点离线建图服务)
   - [1.1 状态机架构原理](#11-状态机架构原理)
   - [1.2 使用方法](#12-使用方法)
   - [1.3 服务接口说明](#13-服务接口说明)
   - [1.4 应用实例](#14-应用实例)
2. [交互式建图工具](#2-交互式建图工具)
   - [2.1 功能概述](#21-功能概述)
   - [2.2 使用方法](#22-使用方法)
   - [2.3 键盘控制命令](#23-键盘控制命令)
   - [2.4 注意事项](#24-注意事项)
3. [配置参数说明](#3-配置参数说明)
4. [常见问题与解决方案](#4-常见问题与解决方案)

---

## 1. Lifecycle节点离线建图服务

Lifecycle节点离线建图服务基于ROS2的Lifecycle Node架构实现，提供了标准化的状态管理和生命周期控制能力。

### 1.1 状态机架构原理

#### 1.1.1 ROS2 Lifecycle Node 状态机

ROS2 Lifecycle Node遵循标准的状态机模型，包含以下**主要状态(Primary States)**：

| 状态 | 名称 | 说明 |
|------|------|------|
| `UNCONFIGURED` | 未配置状态 | 节点初始状态，资源未分配 |
| `INACTIVE` | 非活动状态 | 节点已配置但未激活，资源已分配但未运行 |
| `ACTIVE` | 活动状态 | 节点正在运行，执行主要功能 |
| `FINALIZED` | 终止状态 | 节点已关闭，不可恢复 |

#### 1.1.2 状态转换图

```
                    ┌──────────────────────────────────────────────────────┐
                    │                                                      │
                    ▼                                                      │
            ┌───────────────┐                                      ┌───────┴───────┐
            │ UNCONFIGURED  │◄─────────────────────────────────────│   FINALIZED   │
            │  (未配置)      │                                      │    (终止)     │
            └───────┬───────┘                                      └───────────────┘
                    │                                                      ▲
        configure   │                                                      │
         (配置)      │                                                      │
                    ▼                                                      │
            ┌───────────────┐                                      ┌───────┴───────┐
            │   INACTIVE    │◄─────────────────────────────────────│   shutdown    │
            │   (非活动)     │          deactivate (停用)           │               │
            └───────┬───────┘                                      └───────────────┘
                    │                                                      │
         activate   │                                                      │
          (激活)    │                                                      │
                    ▼                                                      │
            ┌───────────────┐                                              │
            │    ACTIVE     │──────────────────────────────────────────────┘
            │    (活动)     │
            └───────────────┘
```

#### 1.1.3 转换事件(Transitions)

| 转换事件 | 触发函数 | 状态变化 | 说明 |
|----------|----------|----------|------|
| `TRANSITION_CONFIGURE` | `on_configure()` | UNCONFIGURED → INACTIVE | 初始化资源配置 |
| `TRANSITION_ACTIVATE` | `on_activate()` | INACTIVE → ACTIVE | 启动节点主功能 |
| `TRANSITION_DEACTIVATE` | `on_deactivate()` | ACTIVE → INACTIVE | 暂停节点功能 |
| `TRANSITION_CLEANUP` | `on_cleanup()` | INACTIVE → UNCONFIGURED | 清理资源，重置状态 |
| `TRANSITION_SHUTDOWN` | `on_shutdown()` | 任意 → FINALIZED | 关闭节点 |

#### 1.1.4 本项目的状态机实现

本项目实现了两个Lifecycle节点：

1. **[`LifecycleOfflineCartoNode`](amr_map/cartographer_ros/src/lifecycle_offline_node.cpp)** - 标准离线建图节点
2. **[`LifecycleOfflineReflectorNode`](amr_map/cartographer_ros/src/lifecycle_offline_reflector_node.cpp)** - 反光柱离线建图节点

**内部状态枚举**：

```cpp
enum class MapBuildStatus {
  STATUS_IDEL = 0,      // 空闲状态
  STATUS_BUILDING,      // 建图中
  STATUS_COMPLETE,      // 建图完成
  STATUS_CANCEL,        // 建图取消
  STATUS_ERROR,         // 建图错误
  STATUS_FAILED         // 建图失败
};
```

**状态机流转序列**：

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         建图任务完整生命周期                                   │
└─────────────────────────────────────────────────────────────────────────────┘

1. 初始状态
   ┌─────────────┐
   │UNCONFIGURED │ ← map_build_status_ = STATUS_IDEL
   └─────────────┘

2. 发送CMD_START命令
   ┌─────────────┐    configure     ┌─────────────┐    activate    ┌─────────────┐
   │UNCONFIGURED │ ───────────────► │  INACTIVE   │ ─────────────► │   ACTIVE    │
   └─────────────┘                  └─────────────┘                └─────────────┘
                                                          ↑
                                                          │
                                           map_build_status_ = STATUS_BUILDING
                                           启动建图线程，处理rosbag数据

3. 建图完成（自动流转）
   ┌─────────────┐   deactivate    ┌─────────────┐    cleanup    ┌─────────────┐
   │   ACTIVE    │ ──────────────► │  INACTIVE   │ ────────────► │UNCONFIGURED │
   └─────────────┘                 └─────────────┘               └─────────────┘
                                          ↑
                                          │
                           map_build_status_ = STATUS_COMPLETE/CANCEL

4. 取消建图（CMD_CANCEL）
   ┌─────────────┐   enable_mapping_=false   ┌─────────────┐
   │   ACTIVE    │ ─────────────────────────► │  INACTIVE   │
   └─────────────┘                            └─────────────┘
                                                      │
                                          map_build_status_ = STATUS_CANCEL
```

### 1.2 使用方法

#### 1.2.1 启动Lifecycle节点

**启动反光柱建图Lifecycle节点**：

```bash
# 方式：直接运行节点
ros2 run cartographer_ros cartographer_lifecycle_offline_reflector_node \
  --ros-args \
  -p rosbag_dir:=/home/admin/map_dir/bag_dir \
  -p cartoConfig_dir:=/home/admin/map_dir/carto_config \
  -p output_dir:=/home/admin/map_dir/output_dir
```
[notice] 参数可不指定，以上为默认值

**启动标准建图Lifecycle节点**：

```bash
ros2 run cartographer_ros cartographer_lifecycle_offline_node \
  --ros-args \
  -p rosbag_dir:=/home/admin/map_dir
```
😢😢😢 [deperacted] 标准建图方式，要求在cartographer的lua配置文件中指定`use_landmarks = false` 不然会受反光柱建图的影响。
后期此工具将被弃用，建议使用`cartographer_lifecycle_offline_reflector_node`进行标准建图。

#### 1.2.2 手动管理Lifecycle
```bash
# 查看当前节点状态
ros2 lifecycle list /lifecycle_cartographer_node

# 手动触发获取状态转换用于状态调式
ros2 lifecycle get /lifecycle_cartographer_node

# 手动触发状态转换用于状态调式, 慎用
ros2 lifecycle set /lifecycle_cartographer_node configure
ros2 lifecycle set /lifecycle_cartographer_node activate
ros2 lifecycle set /lifecycle_cartographer_node deactivate
ros2 lifecycle set /lifecycle_cartographer_node cleanup
```

### 1.3 服务接口说明

Lifecycle建图节点提供`build_map_service`服务，支持以下命令：

#### 1.3.1 服务定义

**服务类型**: `byd_mapbuilder_msgs/srv/MapBuild`

**请求消息**:
```
MapBuildRequest req
  uint8 cmd_id        # 命令ID
  string bagfile      # rosbag文件名
  string filename     # 输出文件名(.pbstream)
  string configfile   # 配置文件名(.lua)
  string reflector_param_file  # 反光柱参数文件(.yaml)
```

**响应消息**:
```
MapBuildStatus status
  uint8 status        # 状态码
  float32 progress    # 进度(0.0-1.0)
string msg            # 状态描述信息
```

#### 1.3.2 命令ID说明

| 命令ID | 常量名 | 说明 | 使用场景 |
|--------|--------|------|----------|
| `0` | `CMD_IDEL` | 查询状态 | 查询当前建图任务状态 |
| `1` | `CMD_START` | 开始建图 | 启动新的建图任务 |
| `2` | `CMD_CANCEL` | 取消建图 | 取消当前正在进行的建图任务 |

#### 1.3.3 服务调用示例

**查询状态**:
```bash
ros2 service call /build_map_service byd_mapbuilder_msgs/srv/MapBuild \
  "{req: {cmd_id: 0}}"
```

**开始建图**:
```bash
ros2 service call /build_map_service byd_mapbuilder_msgs/srv/MapBuild \
  "{req: {cmd_id: 1, bagfile: 'test.bag', filename: 'output.pbstream', configfile: 'offline_bdy_amr2.lua', reflector_param_file: 'reflector_params.yaml'}}"
```

📖: 默认从`/home/admin/map_dir`读取配置文件和rosbag文件，输出文件保存到`/home/admin/map_dir/output_dir`

**取消建图**:
```bash
ros2 service call /build_map_service byd_mapbuilder_msgs/srv/MapBuild \
  "{req: {cmd_id: 2}}"
```

#### 1.3.4 进度监控

建图进度通过`/mapping_process`话题发布：

```bash
# 监听进度话题
ros2 topic echo /mapping_process

# 进度消息格式
# byd_mapbuilder_msgs/msg/MapBuildProcess
# float32 progress  # 进度值 0.0-1.0
```

**进度阶段划分**：
- `0.0 - 0.3`: rosbag数据回放阶段
- `0.3 - 0.7`: 反光柱检测回溯阶段
- `0.7 - 0.85`: 地图优化与保存阶段
- `0.85 - 1.0`: 地图格式转换阶段

### 1.4 应用实例

#### 1.4.1 LifecycleOfflineReflectorNode 反光柱建图节点

参考源码：[`lifecycle_offline_reflector_node.cpp`](amr_map/cartographer_ros/src/lifecycle_offline_reflector_node.cpp)

**核心回调函数**：

| 回调函数 | 功能 |
|----------|------|
| [`on_configure()`](amr_map/cartographer_ros/src/lifecycle_offline_reflector_node.cpp:387) | 创建ROS节点，配置反光柱检测模块 |
| [`on_activate()`](amr_map/cartographer_ros/src/lifecycle_offline_reflector_node.cpp:1065) | 启动建图线程，处理rosbag数据 |
| [`on_deactivate()`](amr_map/cartographer_ros/src/lifecycle_offline_reflector_node.cpp:1701) | 等待建图线程结束，清理执行器 |
| [`on_cleanup()`](amr_map/cartographer_ros/src/lifecycle_offline_reflector_node.cpp:1754) | 重置所有资源和状态变量 |

**关键特性**：
- 支持反光柱检测参数文件配置
- 全局反光柱跟踪器(Glo balReflectorTracker)
- 点云畸变矫正(CublicDistortionCorrector)
- 自动状态转换和资源清理

#### 1.4.2 LifecycleOfflineCartoNode 标准建图节点

参考源码：[`lifecycle_offline_node.cpp`](amr_map/cartographer_ros/src/lifecycle_offline_node.cpp)

**核心回调函数**：

| 回调函数 | 功能 |
|----------|------|
| [`on_configure()`](amr_map/cartographer_ros/src/lifecycle_offline_node.cpp:313) | 创建cartographer节点，初始化参数 |
| [`on_activate()`](amr_map/cartographer_ros/src/lifecycle_offline_node.cpp:334) | 启动建图线程，处理传感器数据 |
| [`on_deactivate()`](amr_map/cartographer_ros/src/lifecycle_offline_node.cpp:942) | 等待线程结束，取消执行器 |
| [`on_cleanup()`](amr_map/cartographer_ros/src/lifecycle_offline_node.cpp:995) | 重置线程和节点资源 |

**关键特性**：
- 实时反光柱Landmark检测
- Landmark位姿优化与更新
- 支持多传感器数据源

---

## 2. 交互式建图工具

交互式建图工具提供基于键盘控制的逐帧反光柱检测和地图构建功能，适用于调试和可视化场景。

### 2.1 功能概述

参考源码：[`offline_reflector_mapping_node.cpp`](amr_map/cartographer_ros/src/offline_reflector_mapping_node.cpp)

**主要功能**：
- ✅ 逐帧激光雷达数据处理
- ✅ 实时反光柱检测与可视化
- ✅ 键盘控制帧前进/后退
- ✅ 自动播放模式
- ✅ Rviz实时可视化
- ✅ 点云畸变矫正
- ✅ 全局反光柱跟踪

### 2.2 使用方法

#### 2.2.1 启动命令

```bash
ros2 run cartographer_ros cartographer_offline_reflector_mapping_node \
  -configuration_directory=$(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/configuration_files \ -configuration_basenames=offline_bdy_amr2.lua \
  -use_bag_transforms=true \
  -keep_running=true \
  -bag_filenames=/path/to/your.bag \
  --ros-args -r /scan:=scan -r /odom_combined:=odom \
  --params-file /path/to/reflector_bag_mapping.yaml
```

#### 2.2.2 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `bag_path` | string | "" | rosbag文件路径 |
| `scan_topic` | string | "/scan" | 激光雷达话题名 |
| `odom_topic` | string | "/odom_combined" | 里程计话题名 |
| `classification_method` | string | "pca"(默认) | 反光柱分类方法 |
| `distort_corrector.method` | string | "Spline" | 点云畸变矫正方法 |
| `raw_intensity_threshold` | double | 1000.0 | 强度阈值 |
| `min_confidence` | double | 0.5 | 最小置信度 |

### 2.3 键盘控制命令

程序启动后，可通过以下键盘命令进行交互：

| 按键 | 功能 | 说明 |
|------|------|------|
| `n` / `N` | 下一帧 | 处理下一帧激光数据 |
| `p` / `P` | 上一帧 | 回退到上一帧数据 |
| `空格` | 切换自动模式 | 开启/关闭自动播放 |
| `s` / `S` | 保存地图 | 保存反光柱到pbstream地图 |
| `q` / `Q` | 退出程序 | 安全关闭节点 |

**控制流程示意**：

```
┌──────────────────────────────────────────────────────────────────┐
│                      键盘控制交互流程                              │
└──────────────────────────────────────────────────────────────────┘

    启动程序
        │
        ▼
   ┌─────────┐
   │ 第一帧  │◄──────────────────────────────────┐
   └────┬────┘                                   │
        │                                        │
        ▼                                        │
   ┌─────────┐    'n'      ┌─────────┐    'n'    │
   │ 当前帧  │────────────►│ 下一帧  │──────────┤
   └────┬────┘             └─────────┘           │
        │                   'p'                 │
        │◄──────────────────────────────────────┤
        │                                        │
        │ ' ' (空格)                             │
        ▼                                        │
   ┌─────────┐                                   │
   │ 自动模式│───────────────────────────────────┤
   │ (100ms) │                                   │
   └────┬────┘                                   │
        │ 最后一帧                                │
        ▼                                        │
   ┌─────────┐                                   │
   │ 保存地图│                                   │
   │  (s)    │                                   │
   └────┬────┘                                   │
        │                                        │
        │ 'q'                                    │
        ▼                                        │
   ┌─────────┐                                   │
   │  退出   │                                   │
   └─────────┘                                   │
```

### 2.4 注意事项

#### 2.4.1 数据要求

1. **Rosbag数据要求**：
   - **必须包含激光雷达数据**(`/scan`)
   - **必须包含里程计数据**(`/odom_combined`)
   - **推荐包含TF变换数据**(`/tf`, `/tf_static`)
   - **时间戳必须连续，无跳变**

2. **TF变换要求**：
   - 必须存在 `base_link` → `laser` 的静态变换
   - 变换时间戳必须与激光数据同步

#### 2.4.2 参数调优建议

**强度阈值调整**：
```yaml
# 根据实际激光雷达调整强度阈值
raw_intensity_threshold: 1000.0  # 默认值
# 高反射率反光柱可适当提高阈值
# raw_intensity_threshold: 1600.0
```

**DBSCAN聚类参数**：
```yaml
fixed_dbscan:
  eps: 0.05           # 邻域半径
  min_points: 13       # 最小点数
  contine_gap: 5      # 连续间隙阈值
  continue_points: 8  # 连续点数阈值
```

**全局跟踪器参数**：
```yaml
global_tracking:
  enable: true
  match_distance_threshold: 0.3    # 匹配距离阈值
  match_distance_inactive: 0.5     # 匹配距离重新回到视野内反光柱阈值
  confirm_time_window: 1.0         # 确认时间窗口
  min_detections_in_window: 8      # 窗口内最小检测次数
```

#### 2.4.3 可视化话题

程序发布以下可视化话题，可在Rviz中订阅：

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/original_scan` | `sensor_msgs/msg/LaserScan` | 原始激光扫描 |
| `/filtered_points` | `sensor_msgs/msg/PointCloud2` | 滤波后点云 |
| `/clustered_points` | `sensor_msgs/msg/PointCloud2` | 聚类后点云 |
| `/reflector_markers` | `visualization_msgs/msg/MarkerArray` | 反光柱标记 |
| `/tracked_reflectors` | `visualization_msgs/msg/MarkerArray` | 跟踪的反光柱 |
| `/laser_pose` | `visualization_msgs/msg/Marker` | 激光位姿 |

#### 2.4.4 常见问题

**问题1**: 无法获取TF变换
```
解决：检查rosbag中是否包含完整的TF数据，或提供正确的URDF文件
```

**问题2**: 反光柱检测数量为0
```
解决：
1. 检查强度阈值是否合适
2. 检查激光雷达是否正确识别反光柱，可通过手动按帧触发检测
3. 调整DBSCAN聚类参数
```

**问题3**: 程序卡住不动
```
解决：
1. 检查rosbag是否损坏
2. 检查是否有足够内存
3. 查看终端日志输出
4. 通过htop命令查看CPU和内存占用
5. 使用promethues+grafana监控节点运行状态
```

---

## 3. 配置参数说明

### 3.1 反光柱检测参数

```yaml
# reflector_bag_mapping.yaml

# 点云矫正器配置
distort_corrector:
  enable: true
  method: "Spline"  # 矫正方法: Spline, Linear

# DBSCAN聚类配置
fixed_dbscan:
  eps: 0.05
  min_points: 5
  contine_gap: 3
  continue_points: 8

# PCA形状分类配置
pca_classification:
  enable: true
  min_points: 13
  max_elongation_post: 9.5
  min_elongation_board: 12.0
  max_linearity_post: 0.97
  min_linearity_board: 0.93
  near_distance: 1.3
  near_min_points: 20
  near_max_linearity_post: 0.89

# 圆拟合配置
circle_fit:
  max_fit_error: 0.03
  min_inlier_ratio: 0.5
  max_fit_error_near: 0.01
  max_fit_error_far: 0.02
  far_distance_threshold: 1.5
  min_radius: 0.02
  max_radius: 0.05
  max_concave_ratio: 0.2
  ransac_iterations: 100
  ransac_inlier_threshold: 0.0015
  ransac_min_points: 13

# 全局跟踪配置
global_tracking:
  enable: true
  match_distance_threshold: 0.3
  match_distance_inactive: 0.5
  confirm_time_window: 1.0
  min_detections_in_window: 8
  inactive_timeout: 5.0
  max_inactive_time: 60.0
  position_filter_alpha: 0.3
  position_filter_beta: 0.2
  min_std_dev: 0.02
  max_std_dev: 0.5
  min_confidence_to_track: 0.3
  confidence_filter_alpha: 0.2
  diameter_filter_alpha: 0.3
```

### 3.2 Cartographer配置

参考配置文件：[`offline_bdy_amr2.lua`](amr_map/cartographer_ros/configuration_files/offline_bdy_amr2.lua)

```lua
-- 关键配置项
include "map_builder.lua"
include "trajectory_builder.lua"

map_builder = {
  use_trajectory_builder_2d = true,
  pose_graph = {
    optimize_every_n_nodes = 90,
    -- 反光柱Landmark约束配置
    landmark_sampling_rate = 0.1,
  }
}

trajectory_builder_2d = {
  min_range = 0.1,
  max_range = 30.0,
  -- 反光柱建图需要开启Landmark
  use_landmarks = true,
}
```

---

## 4. 常见问题与解决方案

### 4.1 Lifecycle相关问题

**Q: 如何查看当前Lifecycle节点状态？**
```bash
ros2 lifecycle get /lifecycle_cartographer_node
ros2 lifecycle list /lifecycle_cartographer_node
```

**Q: 状态转换失败怎么办？**
```bash
# 检查节点日志
ros2 topic echo /rosout

# 手动触发状态转换
ros2 lifecycle set /lifecycle_cartographer_node configure
```

**Q: 建图任务卡在BUILDING状态？**
```
检查：
1. rosbag文件是否存在且可读
2. 配置文件路径是否正确
3. 内存是否充足
4. 查看详细日志输出
```

### 4.2 反光柱检测问题

**Q: 检测到的反光柱数量不稳定？**
```
解决方案：
1. 调整强度阈值 raw_intensity_threshold
2. 调整DBSCAN参数 eps 和 min_points
3. 开启全局跟踪器 global_tracking.enable: true
```

**Q: 反光柱位置偏移较大？**
```
解决方案：
1. 开启点云畸变矫正 distort_corrector.enable: true or false
2. 检查TF变换是否准确
3. 调整里程计数据质量
```

### 4.3 地图输出问题

**Q: 输出的地图文件在哪里？**
```
默认输出路径由参数指定：
- Lifecycle节点: output_dir 参数
- 交互式节点: rosbag同目录

输出文件格式：
- .pbstream: Cartographer状态文件
- .pgm: 栅格地图图像
- .yaml: 地图元数据
- .smap: smap元信息文件
```

**Q: 如何使用查看的地图？**
```bash
# 加载pbstream进行定位
ros2 launch cartographer_ros visualize_pbstream.launch.py \
pbstream_filename:=/path/you/map.pbstream
```


---

## 附录

### A. 相关源码文件

| 文件路径 | 说明 |
|----------|------|
| [`lifecycle_offline_reflector_node.cpp`](amr_map/cartographer_ros/src/lifecycle_offline_reflector_node.cpp) | 反光柱Lifecycle建图节点 |
| [`lifecycle_offline_node.cpp`](amr_map/cartographer_ros/src/lifecycle_offline_node.cpp) | 标准Lifecycle建图节点 |
| [`offline_reflector_mapping_node.cpp`](amr_map/cartographer_ros/src/offline_reflector_mapping_node.cpp) | 交互式建图节点 |
| [`reflector_detector.hpp`](amr_perception/amr_reflector_noise_handling/include/amr_reflector_noise_handling/reflector_detector.hpp) | 反光柱检测器 |
| [`global_reflector_tracker.cpp`](amr_perception/amr_reflector_noise_handling/src/global_reflector_tracker.cpp) | 全局反光柱跟踪器 |

### B. 参考资料

- [ROS2 Lifecycle Node 官方文档](https://design.ros2.org/articles/node_lifecycle.html)
- [Cartographer 官方文档](https://google-cartographer-ros.readthedocs.io/)
- [rclcpp_lifecycle API参考](https://docs.ros2.org/latest/api/rclcpp_lifecycle/)

---

*文档版本: 1.0.0*
*最后更新: 2026-03-06*
*作者: LeonZack055*
