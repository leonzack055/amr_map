#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2直线特征提取脚本

这个脚本提供了一个ROS2兼容的接口来运行直线特征提取工具。

使用方法:
    ros2 run cartographer_ros line_feature_extractor.py --pbstream map.pbstream

作者: Cartographer Line Feature Extraction Team
日期: 2024
"""

import os
import sys
import argparse
import subprocess
import json
import logging
from pathlib import Path
import rclpy
from rclpy.node import Node

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class LineFeatureExtractorNode(Node):
    """直线特征提取ROS2节点"""
    
    def __init__(self):
        super().__init__('line_feature_extractor_node')
        self.get_logger().info('Line Feature Extractor Node initialized')
        
    def extract_features(self, pbstream_path, config_file=None, output_dir=None, 
                      preset=None, enable_visualization=True, verbose=False):
        """
        执行直线特征提取
        
        Args:
            pbstream_path (str): pbstream文件路径
            config_file (str): 配置文件路径
            output_dir (str): 输出目录
            preset (str): 预设配置
            enable_visualization (bool): 是否启用可视化
            verbose (bool): 是否启用详细日志
            
        Returns:
            bool: 是否成功
        """
        try:
            # 验证输入参数
            if not pbstream_path:
                self.get_logger().error('pbstream_path is required')
                return False
                
            if not os.path.exists(pbstream_path):
                self.get_logger().error(f'pbstream file does not exist: {pbstream_path}')
                return False
                
            # 创建输出目录
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
                
            # 构建命令
            cmd = ['ros2', 'run', 'cartographer_ros', 'cartographer_line_feature_extractor']
            
            # 添加参数
            cmd.extend(['--pbstream_path', pbstream_path])
            
            if config_file:
                cmd.extend(['--config_file', config_file])
                
            if output_dir:
                cmd.extend(['--output_dir', output_dir])
                
            if preset:
                cmd.extend(['--preset', preset])
                
            if not enable_visualization:
                cmd.extend(['--enable_visualization=false'])
                
            if verbose:
                cmd.extend(['--verbose'])
                
            self.get_logger().info(f'Running command: {" ".join(cmd)}')
            
            # 执行命令
            result = subprocess.run(
                cmd,
                check=True,
                capture_output=True,
                text=True,
                timeout=300  # 5分钟超时
            )
            
            self.get_logger().info('Line feature extraction completed successfully')
            self.get_logger().info(f'Output: {result.stdout}')
            
            # 分析结果
            return self._analyze_results(output_dir or './line_features_output')
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Line feature extraction failed: {e}')
            self.get_logger().error(f'Error output: {e.stderr}')
            return False
        except subprocess.TimeoutExpired:
            self.get_logger().error('Line feature extraction timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            return False
            
    def _analyze_results(self, output_dir):
        """分析提取结果"""
        try:
            output_path = Path(output_dir)
            
            # 检查输出文件
            json_file = output_path / 'line_features.json'
            if json_file.exists():
                with open(json_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                line_count = len(data.get('line_features', []))
                self.get_logger().info(f'Extracted {line_count} line features')
                
                # 检查可视化文件
                viz_file = output_path / 'line_features_visualization.png'
                if viz_file.exists():
                    self.get_logger().info(f'Visualization saved to: {viz_file}')
                    
                return True
            else:
                self.get_logger().warning('No output files found')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error analyzing results: {e}')
            return False


def main():
    """主函数"""
    rclpy.init()
    
    # 创建节点
    node = LineFeatureExtractorNode()
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='ROS2 Line Feature Extractor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 基本使用
  python line_feature_extractor.py --pbstream map.pbstream
  
  # 指定配置和输出目录
  python line_feature_extractor.py --pbstream map.pbstream --config config.lua --output-dir ./results
  
  # 使用预设配置
  python line_feature_extractor.py --pbstream map.pbstream --preset accurate --verbose
        """
    )
    
    parser.add_argument(
        '--pbstream',
        type=str,
        required=True,
        help='Path to pbstream file'
    )
    
    parser.add_argument(
        '--config',
        type=str,
        help='Path to configuration file'
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default='./line_features_output',
        help='Output directory (default: ./line_features_output)'
    )
    
    parser.add_argument(
        '--preset',
        type=str,
        choices=['fast', 'balanced', 'accurate', 'indoor', 'outdoor'],
        help='Preset configuration'
    )
    
    parser.add_argument(
        '--no-visualization',
        action='store_true',
        help='Disable visualization output'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose logging'
    )
    
    args = parser.parse_args()
    
    try:
        # 执行特征提取
        success = node.extract_features(
            pbstream_path=args.pbstream,
            config_file=args.config,
            output_dir=args.output_dir,
            preset=args.preset,
            enable_visualization=not args.no_visualization,
            verbose=args.verbose
        )
        
        if success:
            node.get_logger().info('✅ Line feature extraction completed successfully!')
        else:
            node.get_logger().error('❌ Line feature extraction failed!')
            
        return 0 if success else 1
        
    except KeyboardInterrupt:
        node.get_logger().info('⚠️  User interrupted operation')
        return 1
    except Exception as e:
        node.get_logger().error(f'❌ Unexpected error: {e}')
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())