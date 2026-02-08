"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare,FindPackagePrefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
  ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    pkg_prefix = os.path.dirname(FindPackagePrefix('cartographer_ros').find('cartographer_ros'))

    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'byd_amr.urdf')
    
    declare_load_state = DeclareLaunchArgument('load_state_filename', default_value='', description='Filename of the state file to load.')
    declare_load_fronze = DeclareLaunchArgument('load_frozen_state', default_value='false', description='Whether to load the state as frozen.')
    load_state_filename = LaunchConfiguration('load_state_filename')
    load_frozen_state = LaunchConfiguration('load_frozen_state')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    # 是否使用 robot_state_publisher 节点
    use_urdf = LaunchConfiguration('use_urdf')
    declear_use_urdf = DeclareLaunchArgument(
        'use_urdf',
        default_value='True',
        description='Bag包运行时使用自定义开启自定义urdf')
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}],
        condition=IfCondition(use_urdf),
        output = 'log'
        )
    ## ***** Nodes *****
    # 建图结束时将地图保存到install/map.pbstream中
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'online_byd_landmark.lua',
            '-save_state_filename', pkg_prefix+'/map.pbstream',
            '-load_state_filename', load_state_filename,
            '-load_frozen_state', load_frozen_state,
            '--ros-args', '--log-level', 'info'],
        remappings = [
            ('odom', '/odom_combined'),],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_docker2d.rviz'],
        parameters = [{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_load_state,
        declare_load_fronze,
        # Launch arguments
        declear_use_urdf,
        LogInfo(msg=[LaunchConfiguration('use_urdf')]),
        robot_state_publisher_node,
        # Nodes
        cartographer_node,
        cartographer_occupancy_grid_node,
        # rviz_node,
    ])
