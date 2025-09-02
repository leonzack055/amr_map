"""
  Copyright 2024 [Your Name]
  修复了所有问题的纯定位模式启动文件

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
from launch.actions import DeclareLaunchArgument, Shutdown, ExecuteProcess, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
import os

def generate_launch_description():
    # 定位模式，需要制定load_state_filename 默认在install/map.pbstream中
    pkg_prefix = os.path.dirname(FindPackagePrefix('cartographer_ros').find('cartographer_ros'))
    # ***** Launch arguments *****
    bag_filename_arg = DeclareLaunchArgument('bag_filename', description='数据包')
    no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value='false')
    use_bag_transforms_arg = DeclareLaunchArgument(
        'use_bag_transforms', default_value='true')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=FindPackageShare(
        'cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz')
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value=os.path.join(
        FindPackageShare('cartographer_ros').find('cartographer_ros'), 
        'configuration_files'
    ))
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename', default_value='online_localization.lua')
    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename', description='Path to .pbstream map file to load', 
        default_value=os.path.join(pkg_prefix, 'map.pbstream'))    
    
    # ***** Nodes *****
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
    #    parameters = [{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', LaunchConfiguration('load_state_filename'),
            '-load_frozen_state=true',  # 使用load_state_frozen冻结地图
        ],
       remappings=[
           ('scan', '/scan'),
           ('imu', '/imu'),
           ('odom', '/odom_combined')
       ],
        output='log'
    )
    
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
     #       {'use_sim_time': True},
            {'resolution': 0.05}],
        )
    initializer_node = Node(
        package='cartographer_initializer',
        executable='cartographer_initializer',
        name='cartographer_initializer',
        output='log',
      #  parameters=[{'use_sim_time': True}],
    )
    
    pose_interpolator_node = Node(
        package='pose_interpolator',
        executable='pose_interpolator_node',
        name='pose_interpolator',
        output='screen',
        parameters=[{
            'output_frequency': 50.0,
            'max_interpolation_time': 0.5,
            # 'use_sim_time': True
        }],
        remappings=[
            ('/tracked_pose', '/tracked_pose'),
            ('/odom', '/odom_combined'),
            ('/global_pose', '/global_pose')
        ]
    )
    
    return LaunchDescription([
        no_rviz_arg,
        use_bag_transforms_arg,
        configuration_directory_arg,
        configuration_basename_arg,
        load_state_filename_arg,
        initializer_node,
        pose_interpolator_node,   
        # 主节点立即启动
        cartographer_node,
        cartographer_occupancy_grid_node
        
    ])
