# ROS2直线特征提取快速开始指南

## 🚀 ROS2快速上手

### 1. 系统要求

- ROS2 Humble/Iron/Jazzy
- OpenCV 4.x
- Cartographer (ROS2版本)
- Lua 5.1+
- CMake 3.16+
- Python 3.8+

### 2. 编译安装

```bash
# 进入工作空间
cd /workspaces/ros-dev/amr_ws

# 安装依赖
sudo apt-get install ros-$ROS_DISTRO-opencv python3-opencv

# 编译项目
colcon build --packages-select cartographer_ros

# 或者使用您的build.sh脚本
./build.sh
```

### 3. 基本使用

#### 方法一：使用Launch文件（推荐）

```bash
# 基本使用
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=/path/to/map.pbstream

# 使用预设配置
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=/path/to/map.pbstream \
  preset:=accurate

# 自定义输出目录
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=/path/to/map.pbstream \
  output_dir:=./my_results \
  enable_visualization:=true
```

#### 方法二：直接运行可执行文件

```bash
# C++版本
ros2 run cartographer_ros cartographer_line_feature_extractor \
  --pbstream_path /path/to/map.pbstream \
  --preset balanced \
  --output_dir ./results

# Python包装器
ros2 run cartographer_ros line_feature_extractor.py \
  --pbstream /path/to/map.pbstream \
  --preset accurate \
  --verbose
```

#### 方法三：编程接口

```cpp
#include "cartographer_ros/line_feature_extractor.h"
#include "rclcpp/rclcpp.hpp"

class LineExtractionNode : public rclcpp::Node {
public:
    LineExtractionNode() : Node("line_extraction_node") {
        // 创建提取器
        extractor_ = std::make_unique<cartographer_ros::LineFeatureExtractor>();
        
        // 加载配置
        extractor_->load_config("line_extraction_config.lua");
        extractor_->apply_preset("balanced");
    }
    
    void extract_from_pbstream(const std::string& pbstream_path) {
        // 从pbstream加载图像
        auto image = load_image_from_pbstream(pbstream_path);
        
        // 提取直线特征
        double resolution = 0.05;
        Eigen::Vector2d origin(0.0, 0.0);
        
        auto features = extractor_->extract_lines(*image, resolution, origin);
        
        RCLCPP_INFO(get_logger(), "Extracted %zu line features", features.size());
        
        // 保存结果
        extractor_->save_to_json(features, "output.json", resolution, origin);
    }

private:
    std::unique_ptr<cartographer_ros::LineFeatureExtractor> extractor_;
};
```

### 4. 配置预设

#### Fast模式（速度优先）
```bash
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=map.pbstream \
  preset:=fast
```

#### Accurate模式（精度优先）
```bash
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=map.pbstream \
  preset:=accurate
```

#### Indoor模式（室内环境）
```bash
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=map.pbstream \
  preset:=indoor
```

#### Outdoor模式（室外环境）
```bash
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=map.pbstream \
  preset:=outdoor
```

### 5. 输出格式

#### JSON格式
```json
{
  "header": {
    "map_name": "example_map",
    "resolution": 0.05,
    "extraction_time": "2024-01-01T12:00:00Z",
    "algorithm": "hybrid",
    "total_lines": 156
  },
  "line_features": [
    {
      "id": 1,
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
id,start_x,start_y,end_x,end_y,length,angle,confidence,type
1,1.0,2.0,3.0,4.0,2.83,45.0,0.95,wall
```

#### 可视化图像
- `line_features_visualization.png`：直线特征叠加在原图上

### 6. 与现有工作流集成

#### 修改Cartographer配置

在您的Cartographer Lua配置文件中添加：

```lua
-- 启用直线特征提取
LINE_EXTRACTION = {
  enable = true,
  preset = "balanced",
  output_formats = {"json", "smap"},
  output_directory = "./line_features",
  
  -- 算法选择
  algorithm = "hybrid",  -- "hough", "lsd", "hybrid"
  
  -- 输出选项
  save_visualization = true,
  save_statistics = true,
  
  -- 性能选项
  enable_parallel_processing = true,
  max_memory_usage = 512  -- MB
}

-- 在地图构建完成后自动提取
map_builder.options.post_processing = {
  line_feature_extraction = true
}
```

#### 集成到ros_map.cpp

直线特征提取已自动集成到现有的WritePgm函数中：

```cpp
// 在ros_map.cpp中，WritePgm函数会自动：
// 1. 从pbstream提取地图图像
// 2. 应用直线特征提取
// 3. 将结果添加到SMAP文件
```

### 7. 性能优化

#### 大地图处理
```bash
# 启用分块处理
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=large_map.pbstream \
  preset:=fast \
  verbose:=true
```

#### 并行处理
```lua
-- 在配置文件中启用并行处理
performance = {
  enable_parallel_processing = true,
  num_threads = 4,  -- 使用4个线程
  chunk_size = 1024
}
```

### 8. 故障排除

#### 编译问题
```bash
# 检查依赖
sudo apt-get install ros-$ROS_DISTRO-opencv-cv-dev

# 清理重新编译
rm -rf build/ install/ log/
colcon build --packages-select cartographer_ros
```

#### 运行时问题
```bash
# 检查pbstream文件
ls -la /path/to/map.pbstream

# 查看详细日志
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=map.pbstream \
  verbose:=true
```

#### 性能问题
```bash
# 使用快速模式
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=map.pbstream \
  preset:=fast

# 监控内存使用
htop
```

### 9. 批量处理

#### 创建批量处理脚本
```bash
#!/bin/bash
# batch_extract.sh

PBSTREAM_DIR="/path/to/pbstreams"
OUTPUT_DIR="./batch_results"

for pbstream in "$PBSTREAM_DIR"/*.pbstream; do
  basename=$(basename "$pbstream" .pbstream)
  echo "Processing: $basename"
  
  ros2 launch cartographer_ros line_feature_extractor.launch.py \
    pbstream_file:="$pbstream" \
    output_dir:="$OUTPUT_DIR/$basename" \
    preset:=balanced
done
```

#### 运行批量处理
```bash
chmod +x batch_extract.sh
./batch_extract.sh
```

### 10. 可视化结果

#### 使用Python脚本
```bash
python3 scripts/visualize_line_features.py \
  --json line_features_output/line_features.json \
  --map original_map.pgm \
  --all-charts
```

#### 使用RViz
```bash
# 启动RViz
ros2 run rviz2 rviz2

# 加载直线特征话题（如果发布到ROS）
ros2 topic echo /line_features
```

### 11. 高级配置

#### 自定义算法参数
```lua
-- 自定义Hough变换参数
hough_transform = {
  rho_resolution = 1.0,
  theta_resolution = math.pi/180,
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
  precision = 0.01
}
```

#### 自定义后处理
```cpp
// 实现自定义后处理器
class CustomPostProcessor : public LinePostProcessor {
public:
    std::vector<LineFeature> Process(
        const std::vector<LineFeature>& lines,
        const ProcessingConfig& config) override {
        // 自定义处理逻辑
        return processed_lines;
    }
};
```

### 12. 监控和调试

#### 启用详细日志
```bash
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=map.pbstream \
  verbose:=true
```

#### 性能监控
```bash
# 监控CPU和内存使用
htop
iostat -x 1

# 监控ROS2节点
ros2 node list
ros2 node info /line_feature_extractor
```

## 📞 获取帮助

### 文档资源
- **完整文档**：[`README_line_extraction.md`](README_line_extraction.md)
- **配置示例**：[`line_extraction_config.lua`](configuration_files/line_extraction_config.lua)
- **API参考**：代码内详细注释

### 测试工具
- **功能测试**：已集成到可执行文件中
- **Python脚本**：[`scripts/line_feature_extractor.py`](scripts/line_feature_extractor.py)
- **可视化工具**：[`scripts/visualize_line_features.py`](scripts/visualize_line_features.py)

### 常见问题
1. **编译错误**：检查OpenCV和ROS2依赖
2. **运行时错误**：验证pbstream文件路径
3. **性能问题**：使用fast预设或启用并行处理
4. **内存问题**：启用分块处理

---

🎉 **恭喜！** 您现在已经掌握了ROS2环境下直线特征提取的使用方法。

## 🔄 与build.sh集成

您的现有`build.sh`脚本应该能够直接编译新的直线特征提取功能，因为：

1. ✅ 已添加到现有的`CMakeLists.txt`
2. ✅ 使用ROS2标准的`colcon build`流程
3. ✅ 依赖项已正确配置
4. ✅ 可执行文件已注册

只需运行：
```bash
./build.sh
```

然后就可以使用：
```bash
ros2 launch cartographer_ros line_feature_extractor.launch.py \
  pbstream_file:=2laser/map.pbstream
```

开始您的ROS2直线特征提取之旅吧！🚀