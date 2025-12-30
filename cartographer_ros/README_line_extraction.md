# Cartographer 直线特征提取系统

## 概述

本系统为Cartographer SLAM系统添加了直线特征提取功能，能够从pbstream文件生成的地图图像中提取高质量的直线特征，用于地图优化、SLAM改进和环境理解。

## 主要特性

### 🚀 核心功能
- **多算法融合**: 结合霍夫变换（快速）和LSD（精确）两种算法
- **自适应参数**: 根据地图特征自动调整检测参数
- **智能分类**: 自动识别墙壁、走廊、障碍物等直线类型
- **后处理优化**: 去重、连接、验证等完善处理

### ⚙️ 配置灵活
- **预设模式**: 室内、室外、高精度、快速等预设配置
- **参数调优**: 详细的参数配置系统
- **Lua配置**: 使用Lua脚本进行配置管理

### 📊 多格式输出
- **JSON**: 结构化数据，便于程序处理
- **CSV**: 表格数据，便于分析
- **YAML**: 人类可读配置
- **可视化**: 带直线标注的图像
- **统计信息**: 详细的处理统计

## 系统架构

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   输入图像    │───▶│   图像预处理    │───▶│  直线检测算法  │
│ (PGM/PNG)    │    │ (降噪/边缘)     │    │ (霍夫+LSD)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                      │
┌─────────────────┐    ┌──────────────────┐    │
│   后处理      │◀───│   特征融合      │◀───┘
│ (去重/连接)    │    │ (多算法合并)     │
└─────────────────┘    └──────────────────┘
        │
        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   输出格式    │    │   可视化        │    │   统计信息    │
│ (JSON/CSV)     │    │ (标注图像)       │    │ (性能指标)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 快速开始

### 1. 基本使用

```cpp
#include "cartographer_ros/line_feature_extractor.h"

// 创建配置
cartographer_ros::LineExtractionConfig config;
config.enable_line_extraction = true;
config.enable_adaptive_params = true;

// 创建提取器
cartographer_ros::LineFeatureExtractor extractor(config);

// 提取直线特征
auto lines = extractor.ExtractLineFeatures(image, resolution, origin);
```

### 2. 集成到现有工作流

在`ros_map.cpp`中，直线特征提取已集成到`WritePgm`函数：

```cpp
// 启用直线特征提取
WritePgm(image, resolution, file_writer, origin, pbstream_path, true);

// 直线特征会自动添加到SMAP文件中
```

### 3. 使用配置文件

```lua
-- 加载配置
dofile("line_extraction_config.lua")

-- 应用预设
apply_preset("indoor")

-- 自定义参数
line_extraction.hough_transform.threshold = 60
line_extraction.lsd.density_th = 0.8
```

## 配置详解

### 预处理配置

```lua
preprocessing = {
  use_otsu_threshold = true,        -- 使用Otsu自动阈值
  canny_low_threshold = 50.0,        -- Canny低阈值
  canny_high_threshold = 150.0,       -- Canny高阈值
  enable_morphology = true,           -- 启用形态学操作
  morphology_kernel_size = 3           -- 形态学核大小
}
```

### 霍夫变换配置

```lua
hough_transform = {
  enabled = true,                    -- 启用霍夫变换
  threshold = 50,                    -- 投票阈值
  min_line_length_meters = 0.5,       -- 最小线长(米)
  confidence_threshold = 0.7,          -- 置信度阈值
  max_num_lines = 100                 -- 最大线段数
}
```

### LSD配置

```lua
lsd = {
  enabled = true,                    -- 启用LSD
  density_th = 0.7,                 -- 密度阈值
  min_line_length_meters = 0.3,       -- 最小线长(米)
  confidence_threshold = 0.6,          -- 置信度阈值
  ang_th = 22.5                      -- 角度容差(度)
}
```

## 预设配置

### 🏠 室内环境
- **特点**: 结构化环境，直线特征明显
- **参数**: 较严格的阈值，较小核尺寸
- **适用**: 仓库、办公室、工厂内部

### 🌆 室外环境  
- **特点**: 复杂环境，噪声较多
- **参数**: 较宽松的阈值，较大核尺寸
- **适用**: 户外道路、广场、停车场

### 🎯 高精度模式
- **特点**: 追求最高精度
- **参数**: 严格阈值，多重验证
- **适用**: 精密测量、质检场景

### ⚡ 快速模式
- **特点**: 追求处理速度
- **参数**: 宽松阈值，禁用LSD
- **适用**: 实时应用、大批量处理

## 输出格式

### JSON格式
```json
{
  "lines": [
    {
      "id": 0,
      "start": [10.5, 20.3],
      "end": [50.2, 20.3],
      "length": 39.7,
      "angle": 0.0,
      "confidence": 0.85,
      "type": "WALL",
      "method": "HOUGH+LSD"
    }
  ]
}
```

### CSV格式
```csv
id,start_x,start_y,end_x,end_y,length,angle,confidence,type,method
0,10.5,20.3,50.2,20.3,39.7,0.0,0.85,WALL,HOUGH+LSD
```

## 性能优化

### 内存优化
- **分块处理**: 大图像分块处理，减少内存占用
- **智能缓存**: 缓存中间结果，避免重复计算
- **内存池**: 预分配内存，减少动态分配

### 并行处理
- **多线程**: 支持多线程并行处理
- **任务分割**: 将处理任务分割为独立子任务
- **负载均衡**: 动态调整线程负载

### 算法优化
- **早期终止**: 不满足条件的区域早期终止
- **近似计算**: 使用快速近似算法
- **阈值自适应**: 根据图像特征动态调整

## 测试和验证

### 编译测试程序
```bash
cd amr_map/cartographer_ros
g++ -std=c++17 -I../include -I../../../install/include \
    src/line_feature_test.cpp src/*.cpp \
    -o line_feature_test
```

### 运行测试
```bash
./line_feature_test
```

### 测试内容
- **功能测试**: 验证基本功能正确性
- **性能测试**: 测试不同图像大小的处理时间
- **准确性测试**: 验证检测结果的准确性
- **内存测试**: 监控内存使用情况

## 故障排除

### 常见问题

#### 1. 检测线段过多
**症状**: 检测到大量短小线段
**原因**: 阈值过低，噪声过多
**解决**: 
```lua
line_extraction.hough_transform.threshold = 80
line_extraction.canny_low_threshold = 80
```

#### 2. 检测线段过少
**症状**: 明显直线未被检测
**原因**: 阈值过高，参数过严
**解决**:
```lua
line_extraction.hough_transform.threshold = 30
line_extraction.canny_low_threshold = 30
```

#### 3. 处理速度慢
**症状**: 大图像处理时间过长
**原因**: 算法参数设置不合理
**解决**:
```lua
apply_preset("fast")
line_extraction.performance.enable_parallel_processing = true
```

#### 4. 内存占用大
**症状**: 处理过程中内存不足
**原因**: 图像过大，未启用优化
**解决**:
```lua
line_extraction.performance.enable_block_processing = true
line_extraction.performance.block_size = 512
```

## 扩展开发

### 添加新的检测算法
1. 在`LineFeatureExtractor`中添加新的检测器
2. 实现检测器接口
3. 在配置中添加相应参数
4. 更新融合策略

### 添加新的输出格式
1. 在`OutputResults`中添加新的输出函数
2. 实现格式转换逻辑
3. 在配置中添加输出选项

### 添加新的预设配置
1. 在配置文件中添加新的预设
2. 定义各参数的默认值
3. 添加应用预设的函数

## 版本历史

### v1.0.0 (2024-12-26)
- ✅ 基础直线检测功能
- ✅ 霍夫变换 + LSD双算法
- ✅ 自适应参数调整
- ✅ 多格式输出支持
- ✅ 预设配置系统
- ✅ 性能优化机制

## 贡献指南

1. Fork 项目仓库
2. 创建功能分支
3. 提交代码更改
4. 创建 Pull Request
5. 等待代码审查

## 许可证

本项目采用 Apache License 2.0 许可证。详见 LICENSE 文件。

## 联系方式

- **项目维护者**: Cartographer开发团队
- **问题反馈**: 通过GitHub Issues
- **技术讨论**: 通过GitHub Discussions

---

*本系统旨在为Cartographer用户提供高效、准确、易用的直线特征提取解决方案。*