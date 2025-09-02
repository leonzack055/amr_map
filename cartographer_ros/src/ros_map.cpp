#include "cartographer_ros/ros_map.h"
#include "google/protobuf/util/json_util.h"
#include "cartographer_ros/message_map.pb.h"  // 引入protobuf头文件
#include <fstream>
#include <vector>

namespace cartographer_ros {

/**
 * @brief 生成pgm地图和 S-Map地图文件
 * image是左手坐标系，x轴向右，y轴向下；所以要从上向下行扫描了取点
 * @param image 地图图像
 * @param resolution 地图分辨率，单位：m/pixel
 * @param file_writer 文件写入器
 * @param origin 相对于`map_frame`的坐标，图像坐下角坐标
 */
void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer, const Eigen::Vector2d& origin) {
   const std::string header =
      "P5\n# Cartographer map; " + std::to_string(resolution) + " m/pixel\n" +
      std::to_string(image.width()) + " " + std::to_string(image.height()) + "\n255\n";
  file_writer->Write(header.data(), header.size());
  
  // 收集符合条件的像素点
  std::vector<rbk::protocol::Message_MapPos> valid_points;
  const int pixel_threshold = 50;  // 像素阈值，可根据需要调整
  
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const char color = image.GetPixel(x, y)[0];
      file_writer->Write(&color, 1);
      
      // 将char转换为无符号像素值(0-255)
      unsigned char pixel_value = static_cast<unsigned char>(color);
      
      // 检查像素值是否小于阈值
      if (pixel_value < pixel_threshold) {
        // 计算世界坐标
        rbk::protocol::Message_MapPos pos;
        pos.set_x(origin.x() + x * resolution + 0.5*resolution);
        pos.set_y(origin.y() + (image.height()-1 - y) * resolution + 0.5*resolution);
        valid_points.push_back(pos);
      }
    }
  }
  
  // 生成smap文件（假设与pgm文件同路径，仅扩展名不同）
  std::string smap_filename = file_writer->GetFilename();
  size_t ext_pos = smap_filename.find_last_of(".");
  if (ext_pos != std::string::npos) {
    smap_filename = smap_filename.substr(0, ext_pos) + ".smap";
  } else {
    smap_filename += ".smap";
  }
  
  // 调用函数生成smap文件
  PbstreamToSmap(image, resolution, origin, smap_filename, valid_points);
}

void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::cartographer::io::FileWriter* file_writer) {
  const std::string output =
      "image: " + pgm_filename + "\n" + "resolution: " + std::to_string(resolution) + "\n" +
      "origin: [" + std::to_string(origin.x()) + ", " + std::to_string(origin.y()) +
      ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  file_writer->Write(output.data(), output.size());
}

// 实现PbstreamToSmap函数
void PbstreamToSmap(const ::cartographer::io::Image& image, 
                   double resolution, 
                   const Eigen::Vector2d& origin, 
                   const std::string& smap_filename,
                   const std::vector<rbk::protocol::Message_MapPos>& valid_points) {
  // 创建地图消息对象
  rbk::protocol::Message_Map map;

  size_t last_slash_pos = smap_filename.find_last_of("/\\");
  std::string filename;
  if (last_slash_pos != std::string::npos) {
      filename = smap_filename.substr(last_slash_pos + 1);
  } else {
      filename = smap_filename;
  }

  size_t last_dot_pos = filename.find_last_of(".");
  std::string name_without_ext;
  if (last_dot_pos != std::string::npos) {
      name_without_ext = filename.substr(0, last_dot_pos);
  } else {
      name_without_ext = filename; // 如果没有扩展名，直接使用文件名
  }
  
  // 设置地图目录
  map.set_mapdirectory(smap_filename);
  
  // 填充地图头信息
  auto& header = *map.mutable_header();
  header.set_maptype("2D-Map");
  header.set_mapname(name_without_ext);
  header.set_resolution(resolution);
  header.set_version("1.0.0");
  
  // 计算min_pos和max_pos
  if (!valid_points.empty()) {
    auto* min_pos = header.mutable_minpos();
    auto* max_pos = header.mutable_maxpos();
    
    // 初始化min和max为第一个点
    min_pos->set_x(valid_points[0].x());
    min_pos->set_y(valid_points[0].y());
    max_pos->set_x(valid_points[0].x());
    max_pos->set_y(valid_points[0].y());
    
    // 遍历所有点更新min和max
    for (const auto& pos : valid_points) {
      if (pos.x() < min_pos->x()) min_pos->set_x(pos.x());
      if (pos.y() < min_pos->y()) min_pos->set_y(pos.y());
      if (pos.x() > max_pos->x()) max_pos->set_x(pos.x());
      if (pos.y() > max_pos->y()) max_pos->set_y(pos.y());
    }
  }
  
  // 添加所有有效点到normal_pos_list
  for (const auto& pos : valid_points) {
    *map.add_normalposlist() = pos;
  }
  
  // 将protobuf消息转换为JSON
  std::string json_string;
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;  // 保留proto字段名
  options.add_whitespace = true;             // 格式化输出
  
  auto status = google::protobuf::util::MessageToJsonString(map, &json_string, options);
  if (!status.ok()) {
    std::cerr << "Failed to convert protobuf to JSON: " << status.error_message() << std::endl;
    return;
  }
  
  // 写入smap文件
  std::ofstream smap_file(smap_filename);
  if (smap_file.is_open()) {
    smap_file << json_string;
    smap_file.close();
    std::cout << "Successfully generated SMAP file: " << smap_filename << std::endl;
    std::cout << "Number of valid points: " << valid_points.size() << std::endl;
  } else {
    std::cerr << "Failed to open SMAP file for writing: " << smap_filename << std::endl;
  }
}

}
