#include "cartographer_ros/ros_map.h"
#include "google/protobuf/util/json_util.h"
#include "cartographer_ros/message_map.pb.h"  // 引入protobuf头文件
#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/line_feature_extractor.h"
#include <fstream>
#include <vector>

namespace cartographer_ros
{

/**
 * @brief 生成pgm地图和 S-Map地图文件
 * image是左手坐标系，x轴向右，y轴向下；所以要从上向下行扫描了取点
 * @param image 地图图像
 * @param resolution 地图分辨率，单位：m/pixel
 * @param file_writer 文件写入器
 * @param origin 相对于`map_frame`的坐标，图像坐下角坐标
 * @param pbstream_path pbstream文件路径
 * @param enable_line_extraction 是否启用直线特征提取
 */
void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer, const Eigen::Vector2d& origin,
              const std::string& pbstream_path)
{
  const std::string header = "P5\n# Cartographer map; " + std::to_string(resolution) + " m/pixel\n" +
                             std::to_string(image.width()) + " " + std::to_string(image.height()) + "\n255\n";
  file_writer->Write(header.data(), header.size());

  std::string pbstream_file;
  const std::string suffix = ".pbstream";
  if (pbstream_path.size() >= suffix.size())
  {
    // 比较路径末尾与后缀是否相同
    if (std::equal(suffix.rbegin(), suffix.rend(), pbstream_path.rbegin()))
    {
      // 已包含.pbstream后缀，直接赋值
      pbstream_file = pbstream_path;
    }
    else
    {
      // 不包含后缀，添加后赋值
      pbstream_file = pbstream_path + suffix;
    }
  }
  else
  {
    // 路径长度短于后缀，直接添加后缀
    pbstream_file = pbstream_path + suffix;
  }

  // 创建ProtoStreamReader读取pbstream文件
  cartographer::io::ProtoStreamReader reader(pbstream_file);

  // 创建反序列化器
  cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  // 获取pose graph数据
  cartographer::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();

  // 收集反光板数据
  std::vector<rbk::protocol::Message_MapRSSIPos> landmark_rssi_pos_list;
  // 遍历所有的landmark poses
  for (const auto& landmark : pose_graph_proto.landmark_poses())
  {
    rbk::protocol::Message_MapRSSIPos rssi_pos;
    // 提取全局位姿信息
    const auto& global_pose = landmark.global_pose();
    rssi_pos.set_x(global_pose.translation().x());
    rssi_pos.set_y(global_pose.translation().y());
    landmark_rssi_pos_list.push_back(rssi_pos);
  }

  // 收集符合条件的像素点
  std::vector<rbk::protocol::Message_MapPos> valid_points;
  const int pixel_threshold = 50;  // 像素阈值，可根据需要调整
  for (int y = 0; y < image.height(); ++y)
  {
    for (int x = 0; x < image.width(); ++x)
    {
      const char color = image.GetPixel(x, y)[0];
      file_writer->Write(&color, 1);

      // 将char转换为无符号像素值(0-255)
      unsigned char pixel_value = static_cast<unsigned char>(color);

      // 检查像素值是否小于阈值
      if (pixel_value < pixel_threshold)
      {
        // 计算世界坐标
        rbk::protocol::Message_MapPos pos;
        pos.set_x(origin.x() + x * resolution + 0.5 * resolution);
        pos.set_y(origin.y() + (image.height() - 1 - y) * resolution + 0.5 * resolution);
        valid_points.push_back(pos);
      }
    }
  }

  // 生成smap文件（假设与pgm文件同路径，仅扩展名不同）
  std::string smap_filename = file_writer->GetFilename();
  size_t ext_pos = smap_filename.find_last_of(".");
  if (ext_pos != std::string::npos)
  {
    smap_filename = smap_filename.substr(0, ext_pos) + ".smap";
  }
  else
  {
    smap_filename += ".smap";
  }

  // 直线特征提取
  std::vector<cartographer_ros::LineFeature> line_features;
  if (false)
  {  // 默认启用直线特征提取
    // 创建直线特征提取器配置
    cartographer_ros::LineExtractionConfig config;
    config.enable_line_extraction = true;
    config.enable_adaptive_params = true;
    config.enable_debug_output = false;

    // 预处理配置
    config.preprocessing.use_otsu_threshold = true;
    config.preprocessing.enable_canny = true;
    config.preprocessing.canny_low_threshold = 50.0;
    config.preprocessing.canny_high_threshold = 150.0;

    // 霍夫变换配置
    config.hough_transform.enabled = true;
    config.hough_transform.threshold = 50;
    config.hough_transform.min_line_length_meters = 0.5;
    config.hough_transform.confidence_threshold = 0.7;

    // LSD配置
    config.lsd.enabled = false;
    config.lsd.density_th = 0.7;
    config.lsd.min_line_length_meters = 0.3;
    config.lsd.confidence_threshold = 0.6;

    // 输出配置
    config.output.output_json = true;
    config.output.output_csv = true;
    config.output.output_visualization = true;
    config.output.output_directory = "./line_features/";
    config.output.base_filename = smap_filename.substr(0, smap_filename.find_last_of('.')) + "_lines";

    // 创建提取器并提取直线特征
    // 创建cv::Mat对象
    cv::Mat gray_Mat(image.height(), image.width(), CV_8UC1);
    for (int y = 0; y < image.height(); ++y)
    {
      for (int x = 0; x < image.width(); ++x)
      {
        const char color = image.GetPixel(x, y)[0];
        gray_Mat.at<uchar>(y, x) = static_cast<uchar>(color);  // 将char类型转换为uchar类型
      }
    }
    cartographer_ros::LineFeatureExtractor extractor(config);
    line_features = extractor.ExtractLineFeatures(gray_Mat, resolution, origin);

    LOG(INFO) << "直线特征提取完成，共提取 " << line_features.size() << " 条直线";

    // 将直线特征添加到SMAP文件
    AddLineFeaturesToSmap(line_features, smap_filename);
  }

  // 调用函数生成smap文件
  PbstreamToSmap(image, resolution, origin, smap_filename, valid_points, landmark_rssi_pos_list, line_features);
}

void WriteYaml(const double resolution, const Eigen::Vector2d& origin, const std::string& pgm_filename,
               ::cartographer::io::FileWriter* file_writer)
{
  const std::string output = "image: " + pgm_filename + "\n" + "resolution: " + std::to_string(resolution) + "\n" +
                             "origin: [" + std::to_string(origin.x()) + ", " + std::to_string(origin.y()) +
                             ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  file_writer->Write(output.data(), output.size());
}

// 添加直线特征到SMAP文件的函数
void AddLineFeaturesToSmap(const std::vector<cartographer_ros::LineFeature>& line_features,
                           const std::string& smap_filename)
{
  // 读取现有SMAP文件
  std::ifstream smap_file(smap_filename);
  std::string json_content;

  if (smap_file.is_open())
  {
    std::stringstream buffer;
    buffer << smap_file.rdbuf();
    json_content = buffer.str();
    smap_file.close();
  }
  else
  {
    // 如果文件不存在，创建基础结构
    json_content = "{\n  \"header\": {},\n  \"normalposlist\": [],\n  \"rssiposlist\": []\n}";
  }

  // 解析JSON并添加直线特征
  // 简化实现：直接在文件末尾添加直线特征部分
  std::ofstream output_file(smap_filename);
  if (output_file.is_open())
  {
    // 移除最后的 }
    if (!json_content.empty() && json_content.back() == '}')
    {
      json_content.pop_back();
    }

    output_file << json_content << ",\n";
    output_file << "  \"linefeatures\": [\n";

    for (size_t i = 0; i < line_features.size(); ++i)
    {
      const auto& line = line_features[i];
      output_file << "    {\n";
      output_file << "      \"id\": " << i << ",\n";
      output_file << "      \"start\": [" << line.start_point.x() << ", " << line.start_point.y() << "],\n";
      output_file << "      \"end\": [" << line.end_point.x() << ", " << line.end_point.y() << "],\n";
      output_file << "      \"length\": " << line.length << ",\n";
      output_file << "      \"angle\": " << line.angle << ",\n";
      output_file << "      \"confidence\": " << line.confidence << ",\n";
      output_file << "      \"support_points\": " << line.support_points << ",\n";
      output_file << "      \"type\": \"" << cartographer_ros::line_feature_utils::LineTypeToString(line.type)
                  << "\",\n";
      output_file << "      \"method\": \"" << line.detection_method << "\"\n";
      output_file << "    }" << (i < line_features.size() - 1 ? "," : "") << "\n";
    }

    output_file << "  ]\n";
    output_file << "}\n";
    output_file.close();

    LOG(INFO) << "直线特征已添加到SMAP文件: " << smap_filename;
  }
}

// 实现PbstreamToSmap函数
void PbstreamToSmap(const ::cartographer::io::Image& image, double resolution, const Eigen::Vector2d& origin,
                    const std::string& smap_filename, const std::vector<rbk::protocol::Message_MapPos>& valid_points,
                    const std::vector<rbk::protocol::Message_MapRSSIPos>& landmark_rssi_pos_list,
                    const std::vector<cartographer_ros::LineFeature>& line_features)
{
  // 创建地图消息对象
  rbk::protocol::Message_Map map;

  size_t last_slash_pos = smap_filename.find_last_of("/\\");
  std::string filename;
  if (last_slash_pos != std::string::npos)
  {
    filename = smap_filename.substr(last_slash_pos + 1);
  }
  else
  {
    filename = smap_filename;
  }

  size_t last_dot_pos = filename.find_last_of(".");
  std::string name_without_ext;
  if (last_dot_pos != std::string::npos)
  {
    name_without_ext = filename.substr(0, last_dot_pos);
  }
  else
  {
    name_without_ext = filename;  // 如果没有扩展名，直接使用文件名
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
  if (!valid_points.empty())
  {
    auto* min_pos = header.mutable_minpos();
    auto* max_pos = header.mutable_maxpos();

    // 初始化min和max为第一个点
    min_pos->set_x(valid_points[0].x());
    min_pos->set_y(valid_points[0].y());
    max_pos->set_x(valid_points[0].x());
    max_pos->set_y(valid_points[0].y());

    // 遍历所有点更新min和max
    for (const auto& pos : valid_points)
    {
      if (pos.x() < min_pos->x())
        min_pos->set_x(pos.x());
      if (pos.y() < min_pos->y())
        min_pos->set_y(pos.y());
      if (pos.x() > max_pos->x())
        max_pos->set_x(pos.x());
      if (pos.y() > max_pos->y())
        max_pos->set_y(pos.y());
    }
  }

  // 添加所有有效点到normal_pos_list
  for (const auto& pos : valid_points)
  {
    *map.add_normalposlist() = pos;
  }
  for (const auto& rssi_pos : landmark_rssi_pos_list)
  {
    *map.add_rssiposlist() = rssi_pos;
  }
  // 将protobuf消息转换为JSON
  std::string json_string;
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;  // 保留proto字段名
  options.add_whitespace = true;              // 格式化输出

  auto status = google::protobuf::util::MessageToJsonString(map, &json_string, options);
  if (!status.ok())
  {
    std::cerr << "Failed to convert protobuf to JSON: " << status.error_message() << std::endl;
    return;
  }

  // 写入smap文件
  std::ofstream smap_file(smap_filename);
  if (smap_file.is_open())
  {
    smap_file << json_string;
    smap_file.close();
    std::cout << "Successfully generated SMAP file: " << smap_filename << std::endl;
    std::cout << "Number of valid points: " << valid_points.size() << std::endl;
  }
  else
  {
    std::cerr << "Failed to open SMAP file for writing: " << smap_filename << std::endl;
  }
}

}  // namespace cartographer_ros
