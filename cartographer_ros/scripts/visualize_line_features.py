#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
直线特征可视化脚本

这个脚本用于可视化从Cartographer地图中提取的直线特征，
支持多种输出格式和交互式显示。

使用方法:
    python visualize_line_features.py --json line_features.json --map map.pgm

作者: Cartographer Line Feature Extraction Team
日期: 2024
"""

import os
import sys
import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import cv2
from pathlib import Path
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class LineFeatureVisualizer:
    """直线特征可视化器"""
    
    def __init__(self):
        """初始化可视化器"""
        self.fig = None
        self.ax = None
        self.map_image = None
        self.line_features = []
        self.resolution = 0.05  # 默认分辨率
        self.origin = (0.0, 0.0)  # 默认原点
        
    def load_line_features(self, json_file):
        """
        从JSON文件加载直线特征
        
        Args:
            json_file (str): JSON文件路径
        """
        if not os.path.exists(json_file):
            raise FileNotFoundError(f"JSON文件不存在: {json_file}")
            
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
            
        self.line_features = data.get("line_features", [])
        
        # 提取地图信息
        if "header" in data:
            header = data["header"]
            self.resolution = header.get("resolution", 0.05)
            origin = header.get("origin", {})
            self.origin = (origin.get("x", 0.0), origin.get("y", 0.0))
            
        logger.info(f"加载了 {len(self.line_features)} 条直线特征")
        
    def load_map_image(self, map_file):
        """
        加载地图图像
        
        Args:
            map_file (str): 地图文件路径（PGM格式）
        """
        if not os.path.exists(map_file):
            logger.warning(f"地图文件不存在: {map_file}")
            return
            
        # 读取PGM文件
        self.map_image = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        
        if self.map_image is None:
            logger.error(f"无法读取地图文件: {map_file}")
            return
            
        logger.info(f"加载地图图像: {self.map_image.shape}")
        
    def load_yaml_config(self, yaml_file):
        """
        从YAML文件加载地图配置
        
        Args:
            yaml_file (str): YAML文件路径
        """
        if not os.path.exists(yaml_file):
            logger.warning(f"YAML文件不存在: {yaml_file}")
            return
            
        try:
            import yaml
            with open(yaml_file, 'r') as f:
                config = yaml.safe_load(f)
                
            self.resolution = config.get("resolution", 0.05)
            origin = config.get("origin", [0.0, 0.0, 0.0])
            self.origin = (origin[0], origin[1])
            
            logger.info(f"从YAML加载配置: resolution={self.resolution}, origin={self.origin}")
            
        except ImportError:
            logger.warning("未安装PyYAML，无法解析YAML文件")
        except Exception as e:
            logger.warning(f"解析YAML文件失败: {e}")
            
    def create_visualization(self, figsize=(12, 10), show_map=True, show_grid=True):
        """
        创建可视化图像
        
        Args:
            figsize (tuple): 图像大小
            show_map (bool): 是否显示地图背景
            show_grid (bool): 是否显示网格
        """
        self.fig, self.ax = plt.subplots(figsize=figsize)
        
        if show_map and self.map_image is not None:
            # 显示地图背景
            extent = self._calculate_extent()
            self.ax.imshow(self.map_image, cmap='gray', extent=extent, origin='lower')
            
        # 绘制直线特征
        self._draw_line_features()
        
        if show_grid:
            self.ax.grid(True, alpha=0.3)
            
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Line Features Visualization')
        self.ax.set_aspect('equal')
        
    def _calculate_extent(self):
        """计算地图显示范围"""
        if self.map_image is None:
            return None
            
        height, width = self.map_image.shape
        
        # 计算世界坐标范围
        x_min = self.origin[0]
        y_min = self.origin[1]
        x_max = x_min + width * self.resolution
        y_max = y_min + height * self.resolution
        
        return [x_min, x_max, y_min, y_max]
        
    def _draw_line_features(self):
        """绘制直线特征"""
        if not self.line_features:
            logger.warning("没有直线特征可绘制")
            return
            
        # 准备直线数据
        lines = []
        colors = []
        
        for feature in self.line_features:
            start = feature.get("start_point", {})
            end = feature.get("end_point", {})
            
            x1, y1 = start.get("x", 0), start.get("y", 0)
            x2, y2 = end.get("x", 0), end.get("y", 0)
            
            lines.append([(x1, y1), (x2, y2)])
            
            # 根据置信度设置颜色
            confidence = feature.get("confidence", 0.5)
            if confidence > 0.8:
                colors.append('red')  # 高置信度
            elif confidence > 0.6:
                colors.append('orange')  # 中等置信度
            else:
                colors.append('yellow')  # 低置信度
                
        # 创建直线集合
        line_collection = LineCollection(lines, colors=colors, linewidths=2, alpha=0.8)
        self.ax.add_collection(line_collection)
        
        # 添加图例
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='red', lw=2, label='High Confidence (>0.8)'),
            Line2D([0], [0], color='orange', lw=2, label='Medium Confidence (0.6-0.8)'),
            Line2D([0], [0], color='yellow', lw=2, label='Low Confidence (<0.6)')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right')
        
        logger.info(f"绘制了 {len(lines)} 条直线特征")
        
    def add_statistics(self):
        """添加统计信息"""
        if not self.line_features:
            return
            
        # 计算统计信息
        total_lines = len(self.line_features)
        avg_confidence = np.mean([f.get("confidence", 0) for f in self.line_features])
        avg_length = np.mean([f.get("length", 0) for f in self.line_features])
        
        # 统计角度分布
        angles = [f.get("angle", 0) for f in self.line_features]
        horizontal_lines = len([a for a in angles if abs(a) < 15 or abs(a - 180) < 15])
        vertical_lines = len([a for a in angles if 75 < a < 105 or 255 < a < 285])
        
        # 添加文本信息
        stats_text = (
            f"Total Lines: {total_lines}\n"
            f"Avg Confidence: {avg_confidence:.3f}\n"
            f"Avg Length: {avg_length:.2f}m\n"
            f"Horizontal: {horizontal_lines}\n"
            f"Vertical: {vertical_lines}"
        )
        
        self.ax.text(0.02, 0.98, stats_text, transform=self.ax.transAxes,
                    verticalalignment='top', bbox=dict(boxstyle='round', 
                    facecolor='wheat', alpha=0.8))
        
    def save_visualization(self, output_file, dpi=300):
        """
        保存可视化图像
        
        Args:
            output_file (str): 输出文件路径
            dpi (int): 图像分辨率
        """
        if self.fig is None:
            logger.error("请先创建可视化图像")
            return
            
        self.fig.savefig(output_file, dpi=dpi, bbox_inches='tight')
        logger.info(f"可视化图像已保存到: {output_file}")
        
    def show_interactive(self):
        """显示交互式图像"""
        if self.fig is None:
            logger.error("请先创建可视化图像")
            return
            
        plt.show()
        
    def create_angle_histogram(self, output_file=None):
        """
        创建角度分布直方图
        
        Args:
            output_file (str): 输出文件路径（可选）
        """
        if not self.line_features:
            logger.warning("没有直线特征可分析")
            return
            
        angles = [f.get("angle", 0) for f in self.line_features]
        
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.hist(angles, bins=36, range=(0, 180), alpha=0.7, color='skyblue', edgecolor='black')
        ax.set_xlabel('Angle (degrees)')
        ax.set_ylabel('Frequency')
        ax.set_title('Line Angle Distribution')
        ax.grid(True, alpha=0.3)
        
        # 添加主要角度标记
        for angle in [0, 45, 90, 135]:
            ax.axvline(x=angle, color='red', linestyle='--', alpha=0.5, label=f'{angle}°')
            
        ax.legend()
        
        if output_file:
            fig.savefig(output_file, dpi=300, bbox_inches='tight')
            logger.info(f"角度分布图已保存到: {output_file}")
        else:
            plt.show()
            
    def create_length_distribution(self, output_file=None):
        """
        创建长度分布图
        
        Args:
            output_file (str): 输出文件路径（可选）
        """
        if not self.line_features:
            logger.warning("没有直线特征可分析")
            return
            
        lengths = [f.get("length", 0) for f in self.line_features]
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 直方图
        ax1.hist(lengths, bins=20, alpha=0.7, color='lightgreen', edgecolor='black')
        ax1.set_xlabel('Length (meters)')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Line Length Distribution')
        ax1.grid(True, alpha=0.3)
        
        # 累积分布
        sorted_lengths = np.sort(lengths)
        cumulative = np.arange(1, len(sorted_lengths) + 1) / len(sorted_lengths)
        ax2.plot(sorted_lengths, cumulative, color='blue', linewidth=2)
        ax2.set_xlabel('Length (meters)')
        ax2.set_ylabel('Cumulative Probability')
        ax2.set_title('Cumulative Length Distribution')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if output_file:
            fig.savefig(output_file, dpi=300, bbox_inches='tight')
            logger.info(f"长度分布图已保存到: {output_file}")
        else:
            plt.show()
            
    def export_to_cad(self, output_file):
        """
        导出为CAD格式（DXF）
        
        Args:
            output_file (str): 输出DXF文件路径
        """
        try:
            import ezdxf
            
            doc = ezdxf.new('R2010')
            msp = doc.modelspace()
            
            for i, feature in enumerate(self.line_features):
                start = feature.get("start_point", {})
                end = feature.get("end_point", {})
                
                x1, y1 = start.get("x", 0), start.get("y", 0)
                x2, y2 = end.get("x", 0), end.get("y", 0)
                
                # 添加直线
                line = msp.add_line((x1, y1), (x2, y2))
                
                # 设置图层（基于置信度）
                confidence = feature.get("confidence", 0.5)
                if confidence > 0.8:
                    layer_name = "HIGH_CONFIDENCE"
                elif confidence > 0.6:
                    layer_name = "MEDIUM_CONFIDENCE"
                else:
                    layer_name = "LOW_CONFIDENCE"
                    
                if layer_name not in doc.layers:
                    doc.layers.new(layer_name, color=2 if confidence > 0.8 else 3)
                    
                line.dxf.layer = layer_name
                
            doc.saveas(output_file)
            logger.info(f"DXF文件已保存到: {output_file}")
            
        except ImportError:
            logger.warning("未安装ezdxf，无法导出DXF格式")
        except Exception as e:
            logger.error(f"导出DXF失败: {e}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="可视化Cartographer直线特征",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 基本可视化
  python visualize_line_features.py --json line_features.json
  
  # 包含地图背景
  python visualize_line_features.py --json line_features.json --map map.pgm --yaml map.yaml
  
  # 生成所有分析图表
  python visualize_line_features.py --json line_features.json --all-charts --output-dir ./analysis
  
  # 导出CAD格式
  python visualize_line_features.py --json line_features.json --export-cad lines.dxf
        """
    )
    
    parser.add_argument(
        "--json",
        type=str,
        required=True,
        help="直线特征JSON文件路径"
    )
    
    parser.add_argument(
        "--map",
        type=str,
        help="地图PGM文件路径"
    )
    
    parser.add_argument(
        "--yaml",
        type=str,
        help="地图YAML配置文件路径"
    )
    
    parser.add_argument(
        "--output",
        type=str,
        default="line_features_visualization.png",
        help="输出图像文件路径 (默认: line_features_visualization.png)"
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default="./visualization_output",
        help="输出目录 (默认: ./visualization_output)"
    )
    
    parser.add_argument(
        "--all-charts",
        action="store_true",
        help="生成所有分析图表"
    )
    
    parser.add_argument(
        "--export-cad",
        type=str,
        help="导出为DXF格式"
    )
    
    parser.add_argument(
        "--show-stats",
        action="store_true",
        default=True,
        help="显示统计信息"
    )
    
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="显示交互式图像"
    )
    
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="详细输出"
    )
    
    args = parser.parse_args()
    
    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        
    # 创建输出目录
    os.makedirs(args.output_dir, exist_ok=True)
    
    # 创建可视化器
    visualizer = LineFeatureVisualizer()
    
    try:
        # 加载数据
        logger.info("加载直线特征数据...")
        visualizer.load_line_features(args.json)
        
        if args.map:
            logger.info("加载地图图像...")
            visualizer.load_map_image(args.map)
            
        if args.yaml:
            logger.info("加载地图配置...")
            visualizer.load_yaml_config(args.yaml)
            
        # 创建主可视化
        logger.info("创建可视化图像...")
        visualizer.create_visualization(show_map=bool(args.map))
        
        if args.show_stats:
            visualizer.add_statistics()
            
        # 保存主图像
        output_path = os.path.join(args.output_dir, args.output)
        visualizer.save_visualization(output_path)
        
        # 生成额外图表
        if args.all_charts:
            logger.info("生成分析图表...")
            
            # 角度分布
            angle_output = os.path.join(args.output_dir, "angle_distribution.png")
            visualizer.create_angle_histogram(angle_output)
            
            # 长度分布
            length_output = os.path.join(args.output_dir, "length_distribution.png")
            visualizer.create_length_distribution(length_output)
            
        # 导出CAD格式
        if args.export_cad:
            logger.info("导出CAD格式...")
            visualizer.export_to_cad(args.export_cad)
            
        # 显示交互式图像
        if args.interactive:
            visualizer.show_interactive()
            
        logger.info("✅ 可视化完成!")
        
    except Exception as e:
        logger.error(f"❌ 错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()