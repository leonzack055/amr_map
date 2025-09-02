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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames')
    use_bag_transforms_arg = DeclareLaunchArgument(
        'use_bag_transforms', default_value='true')
    keep_running_arg = DeclareLaunchArgument('keep_running', default_value='true')
    no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value='false')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=FindPackageShare(
        'cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz')
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value=FindPackageShare(
        'cartographer_ros').find('cartographer_ros') + '/configuration_files')
    configuration_basenames_arg = DeclareLaunchArgument(
        'configuration_basenames', default_value='offline_bdy_amr2.lua')
    urdf_filenames_arg = DeclareLaunchArgument('urdf_filenames', default_value=FindPackageShare(
        'cartographer_ros').find('cartographer_ros') + '/urdf/byd_amr.urdf')

    # ***** Nodes *****
    # rviz_node = Node(
    #     package = 'rviz2',
    #     executable = 'rviz2',
    #     on_exit = Shutdown(),
    #     # arguments = ['-d', LaunchConfiguration('rviz_config')],
    #     parameters = [{'use_sim_time': True}],
    #     condition = UnlessCondition(LaunchConfiguration('no_rviz'))
    # )
    
    cartographer_offline_node_node = Node(
        package='cartographer_ros',
        executable='cartographer_offline_node',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basenames', LaunchConfiguration('configuration_basenames'),
            '-urdf_filenames', LaunchConfiguration('urdf_filenames'),
            '-use_bag_transforms', LaunchConfiguration('use_bag_transforms'),
            '-keep_running', LaunchConfiguration('keep_running'),
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '--ros-args', '--log-level', 'warn'],
        remappings=[
            ('/odom_combined', 'odom'),
            ],
        output='screen'
    )


    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        use_bag_transforms_arg,
        keep_running_arg,
        no_rviz_arg,
        rviz_config_arg,
        configuration_directory_arg,
        configuration_basenames_arg,
        urdf_filenames_arg,
        # Nodes
        # rviz_node,
        cartographer_offline_node_node
    ])
