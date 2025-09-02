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

## 项目维护

#### 2025.07.21
- 修复了在线建图的问题，现在可以正常显示。
#### 2025.07.22
- 继续LOG日志输出，消息调度和定位匹配耗时，以及消息发布耗时
  
#### 2025.08.22
- ｀feature/mapping` 进行工厂建图功能
- 1. 新增`urdf/byd_sim_amr.urdf`为和`bcr_bot`仿真环境urdf; 实车使用`urdf/byd_amr.urdf`
- 2. `byd_amr_lds.launch.py` 进行在线建图，新增`robot_state_publisher`，不使用`rosbag`的`tf`
- 3. 新增`offline_byd_amr2.lua` 进行`frame_id`映射
- 4. 新增`offline_byd_amr2.launch.py` 不打印建图过程中的`INFO`日志，需要进行手动退出
- 5. 建图指令如下 `ros2 launch cartographer_ros offline_byd_amr2.launch.py use_bag_transforms:=False bag_filenames:='xxx'`,
完成后，会在`rosbag`所在目录生成`pbstream`地图
- 6. 在线建图 `ros2 launch cartographer_ros offline_byd_amr.launch.py` 并在起一个窗口播放`rosbag`可以显示在线建图过程，
可用于比较激光建图`trajectory`与里程计`odom`在整个建图过程中的偏差。

## 项目贡献

## 项目协议