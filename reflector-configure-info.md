# 反光柱检测参数配置说明

## 概述

本文档详细说明反光柱检测系统的参数配置，包括参数原理、重点参数解释以及调参指南。反光柱检测系统用于建图过程中识别环境中的反光柱特征，作为 landmark 约束优化地图质量。

### 相关源码文件

- [`RefelctorDetector`](amr_perception/amr_reflector_noise_handling/include/amr_reflector_noise_handling/reflector_detector.hpp:8) - 反光柱检测主类
- [`PCAShapeClassifier`](amr_perception/amr_reflector_noise_handling/include/amr_reflector_noise_handling/pca_shape_classifier.hpp:60) - PCA形状分类器
- [`CircleFitter`](amr_perception/amr_reflector_noise_handling/include/amr_reflector_noise_handling/circle_fitter.hpp:75) - 圆拟合验证器
- [`GlobalReflectorTracker`](amr_perception/amr_reflector_noise_handling/include/amr_reflector_noise_handling/global_reflector_tracker.hpp:27) - 全局反光柱跟踪器
- [`lifecycle_offline_reflector_node.cpp`](cartographer_ros/src/lifecycle_offline_reflector_node.cpp) - 建图节点参数配置入口

---

## 检测流程架构

```
LaserScan数据
    │
    ▼
┌─────────────────┐
│ 1. 强度过滤     │  ← raw_intensity_threshold
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 2. DBSCAN聚类   │  ← fixed_dbscan.*
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 3. PCA形状分类  │  ← pca_classification.*
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 4. 圆拟合验证   │  ← circle_fit.*
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 5. 全局跟踪     │  ← global_tracking.*
└────────┬────────┘
         │
         ▼
   Landmark输出
```

---

## 一、强度过滤参数

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `raw_intensity_threshold` | double | 1000.0 | 强度过滤阈值，低于此值的点被过滤 |

### 原理解释

激光雷达扫描到的反光柱具有高反射强度特征。强度过滤是检测流程的第一步，通过设定强度阈值筛选可能是反光柱的点云数据。

### 调参指南

| 问题现象 | 调整方向 | 说明 |
|----------|----------|------|
| 反光柱点云过少，漏检 | **降低阈值** | 可能强度阈值过高，过滤掉了有效点 |
| 噪声点过多，误检增加 | **提高阈值** | 阈值过低导致非反光柱点进入后续处理 |
| 不同距离反光柱检测不一致 | 考虑动态阈值 | 远距离反光柱强度衰减，可考虑距离自适应阈值 |

### 注意事项

- 不同品牌/型号的激光雷达强度值范围差异较大，需根据实际设备调整
- 建议先采集反光柱的实际强度数据，统计分布后设定阈值

---

## 二、DBSCAN聚类参数

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `fixed_dbscan.eps` | double | 0.05 | 聚类邻域半径（米） |
| `fixed_dbscan.min_points` | int | 5 | 核心点最小邻居数 |
| `fixed_dbscan.contine_gap` | int | 3 | 连续性间隙阈值（点数） |
| `fixed_dbscan.continue_points` | int | 8 | 连续性判定最小点数 |

### 原理解释

DBSCAN（Density-Based Spatial Clustering of Applications with Noise）是一种基于密度的聚类算法。在反光柱检测中用于将强度过滤后的离散点聚合成簇。

**核心概念**：
- **eps（邻域半径）**：判断两点是否相邻的距离阈值
- **min_points（最小点数）**：一个点成为核心点所需的最小邻居数量
- **连续性检测**：处理激光扫描中因遮挡等原因产生的点云断裂问题

### 重点参数：`eps`

**最重要参数**。直接决定聚类的大小和形状：
- 值过小：反光柱点云被拆分成多个小簇，导致一个反光柱被识别为多个
- 值过大：相邻的反光柱或噪声被合并，导致误检

### 调参指南

| 问题现象 | 调整方向 | 建议值 |
|----------|----------|--------|
| 单个反光柱被拆分成多个簇 | **增大 eps** | 尝试 0.08 ~ 0.12 |
| 相邻反光柱被错误合并 | **减小 eps** | 尝试 0.03 ~ 0.05 |
| 簇的点数过少 | **减小 min_points** | 尝试 3 ~ 4 |
| 噪声点形成簇 | **增大 min_points** | 尝试 6 ~ 8 |
| 点云断裂导致漏检 | **增大 contine_gap** | 尝试 4 ~ 5 |

### 经验公式

```
eps ≈ 反光柱直径 × 1.5 ~ 2.0
```

对于直径 70mm 的反光柱，eps 建议设置为 0.05 ~ 0.08 米。

---

## 三、PCA形状分类参数

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `pca_classification.enable` | bool | true | 是否启用PCA分类 |
| `pca_classification.min_points` | int | 13 | 最小聚类点数 |
| `pca_classification.max_elongation_post` | double | 9.5 | 反光柱最大延伸度 |
| `pca_classification.min_elongation_board` | double | 12.0 | 反光板最小延伸度 |
| `pca_classification.max_linearity_post` | double | 0.97 | 反光柱最大线性度 |
| `pca_classification.min_linearity_board` | double | 0.93 | 反光板最小线性度 |
| `pca_classification.near_distance` | double | 1.3 | 近距离二次判定阈值（米） |
| `pca_classification.near_min_points` | int | 20 | 近距离最小点数 |
| `pca_classification.near_max_linearity_post` | double | 0.89 | 近距离最大线性度 |

### 原理解释

PCA（主成分分析）用于提取点云的形状特征，区分反光柱（圆形）、反光板（线形）和噪声。

**核心特征**：
- **延伸度（Elongation）**：`sqrt(λ1 / λ2)`，其中 λ1、λ2 为协方差矩阵特征值
  - 圆形物体：延伸度接近 1
  - 线形物体：延伸度远大于 1
  
- **线性度（Linearity）**：`(λ1 - λ2) / (λ1 + λ2)`
  - 圆形物体：线性度接近 0
  - 线形物体：线性度接近 1

### 分类规则

```cpp
// 反光柱判定（参见 pca_shape_classifier.hpp:138-154）
if (elongation <= max_elongation_post && linearity <= max_linearity_post) {
    // 近距离二次判定
    if (distance < near_distance) {
        if (point_count >= near_min_points && linearity <= near_max_linearity_post) {
            return REFLECTOR_POST;  // 反光柱
        }
    } else {
        return REFLECTOR_POST;  // 反光柱
    }
}

// 反光板判定
if (elongation >= min_elongation_board && linearity >= min_linearity_board) {
    return REFLECTOR_BOARD;  // 反光板
}
```

### 重点参数

#### `max_elongation_post`（反光柱最大延伸度）

**核心参数**。控制反光柱形状的"圆度"容忍度：
- 值越大：允许更"椭圆"的形状被识别为反光柱
- 值越小：要求更严格的圆形形状

#### `max_linearity_post`（反光柱最大线性度）

**核心参数**。防止将线形物体误判为反光柱：
- 值越大：允许更"线形"的形状
- 值越小：要求更严格的圆形特征

### 调参指南

| 问题现象 | 调整方向 | 说明 |
|----------|----------|------|
| 反光柱漏检（点云稀疏） | **增大 max_elongation_post** | 稀疏点云可能导致延伸度偏高 |
| 反光板误判为反光柱 | **减小 max_linearity_post** | 线形物体的线性度较高 |
| 近距离反光柱漏检 | **减小 near_max_linearity_post** 或 **减小 near_min_points** | 近距离点云密度高，线性度可能偏高 |
| 噪声误判为反光柱 | **增大 min_points** | 过滤点数过少的簇 |

### 典型值参考

```
反光柱（直径70mm）：
  - max_elongation_post: 9.5 ~ 12.0
  - max_linearity_post: 0.7 ~ 0.97

反光板（宽度100mm+）：
  - min_elongation_board: 10.0 ~ 15.0
  - min_linearity_board: 0.85 ~ 0.95
```

---

## 四、圆拟合验证参数

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `circle_fit.max_fit_error` | double | 0.03 | 最大拟合误差（米） |
| `circle_fit.min_inlier_ratio` | double | 0.5 | 最小内点比例 |
| `circle_fit.max_fit_error_near` | double | 0.01 | 近距离最大拟合误差 |
| `circle_fit.max_fit_error_far` | double | 0.02 | 远距离最大拟合误差 |
| `circle_fit.far_distance_threshold` | double | 1.5 | 远距离判定阈值（米） |
| `circle_fit.min_radius` | double | 0.02 | 最小半径（米） |
| `circle_fit.max_radius` | double | 0.05 | 最大半径（米） |
| `circle_fit.max_concave_ratio` | double | 0.2 | 最大凹性度 |
| `circle_fit.ransac_iterations` | int | 100 | RANSAC迭代次数 |
| `circle_fit.ransac_inlier_threshold` | double | 0.0015 | RANSAC内点阈值 |
| `circle_fit.ransac_min_points` | int | 13 | RANSAC最小拟合点数 |

### 原理解释

圆拟合是对PCA分类的二次验证，通过RANSAC算法拟合圆形，验证候选簇是否确实为圆形反光柱。

**RANSAC圆拟合流程**：
1. 随机选择3个点确定一个圆
2. 统计内点数量（到圆心距离接近半径的点）
3. 重复多次，选择内点最多的圆
4. 验证拟合误差、半径范围、凹凸性等条件

### 重点参数

#### `min_radius` / `max_radius`（半径范围）

**关键参数**。直接约束反光柱的物理尺寸：
- 必须根据实际反光柱直径设置
- 半径范围 = 实际半径 ± 允许误差

#### `max_fit_error`（最大拟合误差）

**关键参数**。控制拟合质量：
- 值过大：可能接受非圆形物体
- 值过小：可能拒绝有效反光柱

#### `max_concave_ratio`（最大凹性度）

**重要参数**。区分圆弧和团状噪声：
- 圆弧应该是"凸"的（凹性度低）
- 团状噪声可能呈现"凹"的特征

### 调参指南

| 问题现象 | 调整方向 | 说明 |
|----------|----------|------|
| 反光柱被拒绝（拟合失败） | **增大 max_fit_error** 或 **放宽半径范围** | 点云噪声导致拟合误差偏大 |
| 非圆形物体误判 | **减小 max_fit_error** 或 **减小 max_concave_ratio** | 加强圆形验证 |
| 远距离反光柱漏检 | **增大 max_fit_error_far** | 远距离点云稀疏，拟合误差大 |
| 近距离反光柱漏检 | **增大 max_fit_error_near** | 近距离可能有畸变 |
| RANSAC计算慢 | **减小 ransac_iterations** | 牺牲精度换取速度 |

### 半径设置建议

```cpp
// 假设反光柱直径为 70mm (半径 35mm)
min_radius = 0.02;  // 20mm，允许 -15mm 误差
max_radius = 0.05;  // 50mm，允许 +15mm 误差
```

---

## 五、全局跟踪参数

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `global_tracking.enable` | bool | true | 是否启用全局跟踪 |
| `global_tracking.match_distance_threshold` | double | 0.3 | 匹配距离阈值（米） |
| `global_tracking.match_distance_inactive` | double | 0.5 | 非激活状态匹配阈值 |
| `global_tracking.confirm_time_window` | double | 1.0 | 确认时间窗口（秒） |
| `global_tracking.min_detections_in_window` | int | 8 | 窗口内最小检测次数 |
| `global_tracking.inactive_timeout` | double | 5.0 | 非激活超时时间（秒） |
| `global_tracking.max_inactive_time` | double | 60.0 | 最大非激活时间（秒） |
| `global_tracking.position_filter_alpha` | double | 0.3 | 位置滤波系数 |
| `global_tracking.position_filter_beta` | double | 0.2 | 方差滤波系数 |
| `global_tracking.min_std_dev` | double | 0.02 | 最小位置标准差 |
| `global_tracking.max_std_dev` | double | 0.5 | 最大位置标准差 |
| `global_tracking.min_confidence_to_track` | double | 0.3 | 最小跟踪置信度 |
| `global_tracking.confidence_filter_alpha` | double | 0.2 | 置信度滤波系数 |
| `global_tracking.diameter_filter_alpha` | double | 0.3 | 直径滤波系数 |

### 原理解释

全局跟踪器维护反光柱的全局ID，过滤噪声检测，并通过滑动窗口验证反光柱的真实性。

**状态机**：
```
TENTATIVE (暂定) ──[连续性检测通过]──> CONFIRMED (确认)
      │                                      │
      │                                  [超时未检测]
      │                                      │
      └──[超时未确认]──> 清除             INACTIVE (非激活)
                                             │
                                         [重新检测]
                                             │
                                             ▼
                                         CONFIRMED
```

### 重点参数

#### `confirm_time_window` + `min_detections_in_window`

**核心参数组合**。控制反光柱确认的严格程度：
- 在 `confirm_time_window` 秒内，至少检测到 `min_detections_in_window` 次
- 这是判断反光柱是否为"真实"的关键条件

#### `match_distance_threshold`

**核心参数**。控制数据关联：
- 新检测与已知反光柱的匹配距离
- 值过小：同一反光柱可能被分配多个ID
- 值过大：不同反光柱可能被错误关联

#### `position_filter_alpha`

**重要参数**。EMA滤波系数，控制位置平滑程度：
- 值越大：响应越快，噪声抑制弱
- 值越小：平滑效果强，响应慢

### 调参指南

| 问题现象 | 调整方向 | 说明 |
|----------|----------|------|
| 反光柱无法确认 | **减小 min_detections_in_window** 或 **增大 confirm_time_window** | 检测频率不足 |
| 同一反光柱多个ID | **增大 match_distance_threshold** | 匹配距离过小 |
| 不同反光柱ID混淆 | **减小 match_distance_threshold** | 匹配距离过大 |
| 位置抖动严重 | **减小 position_filter_alpha** | 增强平滑效果 |
| 位置响应慢 | **增大 position_filter_alpha** | 减弱平滑效果 |
| 噪声被确认 | **增大 min_detections_in_window** | 加强确认条件 |

### 参数联动关系

```
确认条件：
  检测频率 ≈ (min_detections_in_window / confirm_time_window)
  
  例如：8次/1秒 = 8Hz 检测频率要求
  
  如果激光频率为 10Hz，则要求 80% 的帧能检测到
```

---

## 六、点云矫正参数

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `distort_corrector.enable` | bool | false | 是否启用点云矫正 |
| `distort_corrector.method` | string | "Spline" | 矫正方法 |

### 原理解释

运动畸变矫正用于修正机器人在激光扫描期间运动导致的点云畸变。对于高速运动的AGV，启用畸变矫正可以提高反光柱位置精度。

### 调参指南

| 场景 | 建议设置 |
|------|----------|
| 低速运动（< 0.5 m/s） | 可不启用 |
| 高速运动（> 1.0 m/s） | 建议启用 |
| 精度要求高 | 建议启用 |

---

## 七、问题诊断与调参速查表

### 常见问题诊断

#### 问题1：反光柱漏检

**症状**：已知存在反光柱，但检测不到或检测率低

**排查步骤**：
1. 检查强度过滤：`raw_intensity_threshold` 是否过高
2. 检查聚类：`fixed_dbscan.eps` 是否过小
3. 检查点数要求：`pca_classification.min_points` 是否过大
4. 检查形状验证：`max_elongation_post` 是否过小
5. 检查圆拟合：`max_fit_error` 是否过小

**推荐调整顺序**：
```
1. 降低 raw_intensity_threshold
2. 增大 fixed_dbscan.eps
3. 减小 pca_classification.min_points
4. 增大 max_elongation_post
5. 增大 circle_fit.max_fit_error
```

#### 问题2：噪声误检

**症状**：检测到虚假的反光柱

**排查步骤**：
1. 检查强度过滤：`raw_intensity_threshold` 是否过低
2. 检查聚类：`fixed_dbscan.min_points` 是否过小
3. 检查形状验证：`max_linearity_post` 是否过大
4. 检查圆拟合：`max_concave_ratio` 是否过大
5. 检查跟踪确认：`min_detections_in_window` 是否过小

**推荐调整顺序**：
```
1. 提高 raw_intensity_threshold
2. 增大 fixed_dbscan.min_points
3. 减小 max_linearity_post
4. 减小 max_concave_ratio
5. 增大 min_detections_in_window
```

#### 问题3：反光柱位置不准

**症状**：反光柱位置抖动或偏移

**排查步骤**：
1. 检查点云矫正：是否需要启用
2. 检查位置滤波：`position_filter_alpha` 设置
3. 检查匹配阈值：`match_distance_threshold` 是否合理

**推荐调整**：
```
1. 启用 distort_corrector.enable = true
2. 调整 position_filter_alpha (减小以增强平滑)
3. 确保激光和里程计时间同步
```

#### 问题4：同一反光柱多个ID

**症状**：同一物理反光柱被分配多个不同的全局ID

**排查步骤**：
1. 检查匹配阈值：`match_distance_threshold` 是否过小
2. 检查位置滤波：`position_filter_alpha` 是否过大导致位置跳变

**推荐调整**：
```
1. 增大 match_distance_threshold
2. 减小 position_filter_alpha
```

#### 问题5：反光柱无法确认

**症状**：反光柱一直处于 TENTATIVE 状态，无法进入 CONFIRMED

**排查步骤**：
1. 检查检测频率：`min_detections_in_window` 是否过高
2. 检查时间窗口：`confirm_time_window` 是否过短
3. 检查是否间歇性检测到

**推荐调整**：
```
1. 减小 min_detections_in_window
2. 增大 confirm_time_window
```

---

### 调参速查表

| 参数类别 | 参数名 | 增大效果 | 减小效果 |
|----------|--------|----------|----------|
| **强度过滤** | raw_intensity_threshold | 漏检增加 | 噪声增加 |
| **聚类** | fixed_dbscan.eps | 簇合并 | 簇分裂 |
| **聚类** | fixed_dbscan.min_points | 噪声减少 | 小簇保留 |
| **PCA** | max_elongation_post | 允许更椭圆 | 要求更圆 |
| **PCA** | max_linearity_post | 允许更线形 | 要求更圆 |
| **PCA** | min_points | 过滤小簇 | 保留小簇 |
| **圆拟合** | max_fit_error | 宽松验证 | 严格验证 |
| **圆拟合** | min_radius | 接受小直径 | 拒绝小直径 |
| **圆拟合** | max_radius | 接受大直径 | 拒绝大直径 |
| **圆拟合** | max_concave_ratio | 允许凹形 | 拒绝凹形 |
| **跟踪** | match_distance_threshold | 易合并 | 易分裂 |
| **跟踪** | min_detections_in_window | 确认严格 | 确认宽松 |
| **跟踪** | confirm_time_window | 确认宽松 | 确认严格 |
| **跟踪** | position_filter_alpha | 响应快 | 平滑强 |

---

## 八、参数配置文件示例

### ROS参数文件示例（YAML格式）

```yaml
# 强度过滤
raw_intensity_threshold: 1000.0
min_confidence: 0.5

# DBSCAN聚类
fixed_dbscan:
  eps: 0.05
  min_points: 5
  contine_gap: 3
  continue_points: 8

# PCA分类
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

# 圆拟合
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

# 全局跟踪
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

# 点云矫正
distort_corrector:
  enable: false
  method: "Spline"
```

---

## 九、参数调试工作流

### 推荐调试流程

1. **数据采集**
   - 采集包含反光柱的典型场景数据
   - 记录反光柱的实际位置、直径、强度分布

2. **强度分析**
   - 统计反光柱点云的强度分布
   - 设置 `raw_intensity_threshold` 过滤非反光柱点

3. **聚类调优**
   - 观察聚类结果，确保一个反光柱对应一个簇
   - 调整 `fixed_dbscan.eps` 和 `min_points`

4. **形状验证调优**
   - 分析反光柱点云的延伸度和线性度分布
   - 调整 PCA 参数使反光柱通过分类

5. **圆拟合验证**
   - 根据反光柱直径设置半径范围
   - 调整误差阈值使有效反光柱通过验证

6. **跟踪参数调优**
   - 根据检测频率调整确认条件
   - 调整匹配阈值避免ID冲突

7. **整体验证**
   - 使用多场景数据验证参数泛化能力
   - 微调参数平衡漏检率和误检率

---

## 参考资料

- [`reflector_detector.cpp`](amr_perception/amr_reflector_noise_handling/src/reflector_detector.cpp) - 检测器实现
- [`global_reflector_tracker.cpp`](amr_perception/amr_reflector_noise_handling/src/global_reflector_tracker.cpp) - 跟踪器实现
- [`lifecycle_offline_reflector_node.cpp`](cartographer_ros/src/lifecycle_offline_reflector_node.cpp:411) - 参数配置入口
