#!/usr/bin/env python3

# Copyright 2024 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch file for Cartographer Line Feature Extractor

This launch file provides a convenient way to run the line feature extractor
with various configuration options.

Usage:
    ros2 launch cartographer_ros line_feature_extractor.launch.py pbstream_file:=map.pbstream
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for line feature extractor"""
    
    # Declare launch arguments
    pbstream_file_arg = DeclareLaunchArgument(
        'pbstream_file',
        description='Path to the pbstream file to process'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to the configuration file (optional)'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='./line_features_output',
        description='Output directory for results'
    )
    
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='balanced',
        choices=['fast', 'balanced', 'accurate', 'indoor', 'outdoor'],
        description='Preset configuration'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization output'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging'
    )
    
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='false',
        description='Use Python wrapper instead of C++ executable'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('cartographer_ros')
    
    # Default config file path
    default_config_file = PathJoinSubstitution([
        pkg_share, 'configuration_files', 'line_extraction_config.lua'
    ])
    
    # Log configuration
    log_config = LogInfo(
        msg=[
            'Line Feature Extractor Configuration:',
            '  PBStream File: ', LaunchConfiguration('pbstream_file'),
            '  Config File: ', LaunchConfiguration('config_file'),
            '  Output Dir: ', LaunchConfiguration('output_dir'),
            '  Preset: ', LaunchConfiguration('preset'),
            '  Visualization: ', LaunchConfiguration('enable_visualization'),
            '  Verbose: ', LaunchConfiguration('verbose'),
            '  Use Python: ', LaunchConfiguration('use_python')
        ]
    )
    
    # C++ executable node
    cpp_node = Node(
        package='cartographer_ros',
        executable='cartographer_line_feature_extractor',
        # name='line_feature_extractor',
        output='screen',
        arguments=[
            '-pbstream_path', LaunchConfiguration('pbstream_file'),
            # '-config_file', LaunchConfiguration('config_file'),
            '-output_dir', LaunchConfiguration('output_dir'),
            '-preset', LaunchConfiguration('preset'),
            '-verbose', LaunchConfiguration('verbose')],
        condition=UnlessCondition(LaunchConfiguration('use_python'))
    )
    
    # Python wrapper node
    # python_node = Node(
    #     package='cartographer_ros',
    #     executable='line_feature_extractor.py',
    #     name='line_feature_extractor_python',
    #     output='screen',
    #     arguments=[
    #         '--pbstream', LaunchConfiguration('pbstream_file'),
    #         ['--config', LaunchConfiguration('config_file')],
    #         ['--output-dir', LaunchConfiguration('output_dir')],
    #         ['--preset', LaunchConfiguration('preset')],
    #     ] + (['--no-visualization'] if LaunchConfiguration('enable_visualization').perform(None) == 'false' else []) +
    #        (['--verbose'] if LaunchConfiguration('verbose').perform(None) == 'true' else []),
    #     condition=IfCondition(LaunchConfiguration('use_python'))
    # )
    
    return LaunchDescription([
        # Launch arguments
        pbstream_file_arg,
        config_file_arg,
        output_dir_arg,
        preset_arg,
        enable_visualization_arg,
        verbose_arg,
        use_python_arg,
        
        # Log configuration
        log_config,
        
        # Nodes
        cpp_node,
        # python_node,
    ])


if __name__ == '__main__':
    generate_launch_description()