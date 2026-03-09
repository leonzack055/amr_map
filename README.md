# 项目名称
cartographer-ros2 用于AMR进行建图和定位； 

## 项目安装

### x86_64编译
需要在本地安装ros2的desktop版，以及cartographer，然后执行以下编译安装
```bash
colcon build --packages-up-to cartographer_ros
colcon build --packages-up-to cartographer_rviz
```
#### 编译安装arm64
需要在arm64的机器上编译，编译指令如下, 需要在 10.4.35.36 USM服务器上,进入docker容器内部
将文件目录拷贝到 byd_app目录下
```bash
sudo bash ../../robot_dev/config/build.sh -p X5 -s cartographer_ros 
sudo bash ../../robot_dev/config/build.sh -p X5 -s cartographer_rviz
```

## 项目使用
建图模式:
- 离线建图:
```bash
ros2 launch ros2 launch cartographer_ros offline_byd_amr.launch.py bag_filenames:=xxx.bag
```
- 在线建图:
```bash
ros2 launch ros2 launch cartographer_ros byd_amr_lds.launch.py
```
**默认** 在线建图的结果是存放在 `install/map.pbstream` 且会持续覆盖。

- 定位模式(板端):
```bash
ros2 launch ros2 launch cartographer_ros online_localization.launch.py load_state_filename:=xxx.bag.pbstream
```
- **lifecycle反光柱建图**:

**第一步**
```bash
ros2 run cartographer_ros cartographer_lifecycle_offline_reflector_node
```
**第二步**
```bash
ros2 service call /build_map_service byd_mapbuilder_msgs/srv/MapBuild "{req: {cmd_id: 1}, filename: "amr_linecargo.pbstream", bagfile: "linesCargo.bag", configfile: "offline_bdy_amr2.lua", reflector_param_file: "reflector_bag_mapping.yaml"}"
```
正常返回状态: 
```bash
response:
byd_mapbuilder_msgs.srv.MapBuild_Response(status=byd_mapbuilder_msgs.msg.MapBuildStatus(status=1, progress=0.0), msg='建图请求成功，正在进行建图工作......')
```
**参数说明**
- `cmd_id`
     - '1': 开始建图命令 
     - '2': 取消正在建图的命令
     - '0': 查询状态
- `filename`: 指定生成地图的名称，以`.pbstream`为结尾
- `bagfile`: 指定使用的`rosbag`包名称
- `configfile`: 指定使用的`lua`文件
- `reflector_param_file`: 指定使用反光柱配置的`yaml`文件
**第三步**
监听建图进程Topic
```bash
ros2 topic echo /mapping_process
```
监听`topic`消息，打印建图过程的百分比，生成完地图后返回100%。


## 项目说明文档
1. `docker-ubuntu22.04` 建图服务配置与使用目录`amr_map/deployment`, 参考配置文件：[`docker-configure.md`](docker-configure.md)
2. `ros2-mapping-cli` 使用说明, 参考配置文件：[`ros2-mapping-cli.md`](ros2-mapping-cli.md)
3.  建图参数配置说明, 参考配置文件：[`mapping-configure-info.md`](mapping-configure-info)
4.  反光柱检测跟踪算法说明与参数配置, 参考配置文件：[`reflector-configure-info.md`](reflector-configure-info.md)


## 项目维护

#### 2025.07.21
- 修复了在线建图的问题，现在可以正常显示。
#### 2025.07.22
- 继续LOG日志输出，消息调度和定位匹配耗时，以及消息发布耗时
  
#### 2025.08.22
- ｀leon/dev` 进行工厂建图功能
- 1. 新增`urdf/byd_sim_amr.urdf`为和`bcr_bot`仿真环境urdf; 实车使用`urdf/byd_amr.urdf`
- 2. `byd_amr_lds.launch.py` 进行在线建图，新增`robot_state_publisher`，不使用`rosbag`的`tf`
- 3. 新增`offline_byd_amr2.lua` 进行`frame_id`映射
- 4. 新增`offline_byd_amr2.launch.py` 不打印建图过程中的`INFO`日志，需要进行手动退出
- 5. 建图指令如下 `ros2 launch cartographer_ros offline_byd_amr2.launch.py use_bag_transforms:=False bag_filenames:='xxx'`,
完成后，会在`rosbag`所在目录生成`pbstream`地图
- 6. 在线建图 `ros2 launch cartographer_ros offline_byd_amr.launch.py` 并在起一个窗口播放`rosbag`可以显示在线建图过程，
可用于比较激光建图`trajectory`与里程计`odom`在整个建图过程中的偏差。

#### 2026.03.6
- ｀feature/mapping` 进行工厂建图功能
- 1. 新增反光柱回溯建图功能: 当前版本，要求`rosbag`包中必须包含标定后的静态TF，录制消息名必须为`\scan`,`\odom_combined`
且雷达消息的`frame_id`为`laser_link`，里程计消息的`frame_id`为`base_link`
- 2. 新增lifecyle离线建图服务默认参数文件目录配置；此文件目录为 `map_dir`，其子目录包含 `bag_dir`  `carto_config`  `output_dir`  `reflector_config`
分别用于存放录制的包，`cartographer`配置文件，输出地图文件，反光柱检测算法配置文件;
- 3. 新增反光柱手动离线建图，开发测试用例，用于测试反光柱检测算法，在离线建图过程中，手动触发反光柱检测算法，用于测试反光柱检测算法的性能对比
- 4. 修复`bag_process`消息在没完成地图本地回写，就返回完成建图100%的BUG；保证收到建图消息为`100%`时，完成所需全部文件的生成
- 5. 关闭直线检测算法服务，并修复生成的`yaml`文件中指定`map_file`的路径问题
- 6. 新增参数说明文档，以及用户可配置文档说明，具体见: **项目说明文档**(二次开发人员使用时必读)。

#### 2026.03.10
- `amr_map:leon/dev` 反光柱离线建图功能; `amr_perception:feature/leon/dev` 反光柱检测算法功能
- 1. 新增docker-file部署功能
- 2. 新增`docker-*-.sh` 脚本启动文件
- 3. 新增网页端快速部署功能，具体见: **项目说明文档**(二次开发人员使用时必读)，[网页端快速部署](docker-configure.md)。

## 项目贡献

## 项目协议