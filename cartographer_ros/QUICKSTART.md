# 直线特征提取快速开始指南

## 🚀 快速上手

### 1. 系统要求

- ROS Noetic/Foxy
- OpenCV 4.x
- Cartographer
- Lua 5.1+
- CMake 3.10+

### 2. 编译安装

```bash
# 进入工作空间
cd /workspaces/ros-dev/amr_ws

# 编译直线特征提取模块
catkin_make -DCATKIN_ENABLE_TESTING=ON

# 或者使用特定编译文件
catkin_make -f amr_map/cartographer_ros/CMakeLists_line_extraction.txt
```

### 3. 基本使用

#### 方法一：修改现有配置文件

在您的Cartographer配置文件中添加：

```lua
-- 添加到现有的.lua配置文件中
LINE_EXTRACTION = {
  enable = true,
  algorithm = "hybrid",  -- "hough", "lsd", "hybrid"
  preset = "balanced",   -- "fast", "balanced", "accurate", "indoor", "outdoor"
  
  -- 输出配置
  output_formats = {"json", "smap"},
  output_image = true,
  output_directory = "./line_features"
}

-- 在地图构建完成后自动提取
map_builder.options.use_trajectory_builder_2d = true
```

#### 方法二：命令行使用

```bash
# 从pbstream文件提取直线特征
rosrun cartographer_ros line_feature_extractor \
  --pbstream_path /path/to/map.pbstream \
  --config_file /path/to/line_extraction_config.lua \
  --output_dir ./output
```

#### 方法三：编程接口

```cpp
#include "cartographer_ros/line_feature_extractor.h"

// 创建提取器
cartographer_ros::LineFeatureExtractor extractor;

// 加载配置
extractor.LoadConfig("path/to/config.lua");

// 从图像提取直线特征
cartographer::io::Image image = /* 您的图像 */;
double resolution = 0.05;
Eigen::Vector2d origin(0.0, 0.0);

auto features = extractor.ExtractLines(image, resolution, origin);

// 输出结果
extractor.SaveResults(features, "output.json", "json");
```

### 4. 配置预设

#### 快速模式 (Fast)
```lua
preset = "fast"
algorithm = "hough"
image_preprocessing.enable_denoising = false
hough_transform.min_line_length = 50
```

#### 精确模式 (Accurate)
```lua
preset = "accurate"
algorithm = "hybrid"
lsd_detector.precision = 0.01
image_preprocessing.enable_denoising = true
```

#### 室内环境 (Indoor)
```lua
preset = "indoor"
algorithm = "hybrid"
hough_transform.min_line_length = 30
lsd_detector.min_line_length = 20
```

#### 室外环境 (Outdoor)
```lua
preset = "outdoor"
algorithm = "hough"
hough_transform.min_line_length = 100
image_preprocessing.adaptive_threshold_size = 15
```

### 5. 输出格式

#### JSON格式
```json
{
  "header": {
    "map_name": "example_map",
    "resolution": 0.05,
    "extraction_time": "2024-01-01T12:00:00Z"
  },
  "line_features": [
    {
      "start_point": {"x": 1.0, "y": 2.0},
      "end_point": {"x": 3.0, "y": 4.0},
      "length": 2.83,
      "angle": 45.0,
      "confidence": 0.95,
      "type": "wall"
    }
  ]
}
```

#### CSV格式
```csv
start_x,start_y,end_x,end_y,length,angle,confidence,type
1.0,2.0,3.0,4.0,2.83,45.0,0.95,wall
```

#### SMAP集成
直线特征会自动添加到现有的SMAP文件中：
```json
{
  "header": { ... },
  "normalposlist": [ ... ],
  "rssiposlist": [ ... ],
  "linefeaturelist": [
    {
      "startpos": {"x": 1.0, "y": 2.0},
      "endpos": {"x": 3.0, "y": 4.0},
      "length": 2.83,
      "angle": 45.0,
      "confidence": 0.95
    }
  ]
}
```

### 6. 性能优化建议

#### 大地图处理
```lua
-- 对于大地图，使用分块处理
performance.enable_chunking = true
performance.chunk_size = 1024
performance.parallel_processing = true
```

#### 实时处理
```lua
-- 实时应用优化
preset = "fast"
algorithm = "hough"
performance.enable_caching = true
performance.max_memory_usage = 512
```

#### 高精度处理
```lua
-- 高精度要求
preset = "accurate"
algorithm = "hybrid"
image_preprocessing.enable_denoising = true
lsd_detector.precision = 0.005
```

### 7. 常见问题解决

#### 编译错误
```bash
# 确保依赖项已安装
sudo apt-get install libopencv-dev liblua5.1-dev

# 清理重新编译
catkin_make clean
catkin_make
```

#### 运行时错误
```bash
# 检查配置文件路径
rosrun cartographer_ros line_feature_extractor --help

# 验证pbstream文件
ls -la /path/to/map.pbstream
```

#### 性能问题
```lua
-- 降低精度要求
preset = "fast"
algorithm = "hough"

-- 减少内存使用
performance.max_memory_usage = 256
performance.enable_chunking = true
```

### 8. 示例工作流

#### 完整的地图构建和直线提取
```bash
# 1. 启动Cartographer
roslaunch cartographer_ros offline_byd_amr2.launch.py \
  bag_filenames:=/path/to/data.bag \
  pose_graph_filename:=/path/to/map.pbstream

# 2. 提取直线特征
rosrun cartographer_ros line_feature_extractor \
  --pbstream_path /path/to/map.pbstream \
  --config_file line_extraction_config.lua \
  --output_dir ./results

# 3. 查看结果
ls ./results/
# 输出: map.json, map.csv, map_lines.png, map_stats.txt
```

#### 批量处理
```bash
#!/bin/bash
# 批量处理多个pbstream文件

for pbstream in /data/*.pbstream; do
  basename=$(basename "$pbstream" .pbstream)
  output_dir="./results/$basename"
  
  mkdir -p "$output_dir"
  
  rosrun cartographer_ros line_feature_extractor \
    --pbstream_path "$pbstream" \
    --config_file line_extraction_config.lua \
    --output_dir "$output_dir"
    
  echo "Processed: $basename"
done
```

### 9. 高级用法

#### 自定义算法参数
```lua
-- 自定义Hough变换参数
hough_transform = {
  rho_resolution = 1.0,
  theta_resolution = CV_PI/180,
  threshold = 80,
  min_line_length = 50,
  max_line_gap = 10
}

-- 自定义LSD参数
lsd_detector = {
  scale = 0.8,
  sigma_scale = 0.6,
  quant = 2.0,
  ang_th = 22.5,
  log_eps = 0.0,
  density_th = 0.7,
  n_bins = 1024,
  min_line_length = 20,
  precision = 0.01
}
```

#### 自定义后处理
```cpp
// 自定义后处理函数
class CustomPostProcessor : public LinePostProcessor {
public:
  std::vector<LineFeature> Process(
      const std::vector<LineFeature>& lines,
      const ProcessingConfig& config) override {
    // 实现自定义逻辑
    return processed_lines;
  }
};

// 注册自定义处理器
extractor.SetPostProcessor(std::make_unique<CustomPostProcessor>());
```

### 10. 故障排除

#### 调试模式
```lua
-- 启用详细日志
debug.enable_logging = true
debug.log_level = "DEBUG"
debug.save_intermediate_results = true
```

#### 性能分析
```bash
# 运行性能测试
rosrun cartographer_ros line_feature_test \
  --test_type performance \
  --iterations 10
```

#### 内存监控
```bash
# 监控内存使用
htop
# 或
ps aux | grep line_feature_extractor
```

## 📞 获取帮助

- 查看完整文档: `README_line_extraction.md`
- 运行测试: `rosrun cartographer_ros line_feature_test --help`
- 配置示例: `line_extraction_config.lua`
- 问题反馈: 提交Issue到项目仓库

---

🎉 **恭喜！** 您现在已经掌握了直线特征提取系统的基本使用方法。开始探索您的地图中的直线特征吧！