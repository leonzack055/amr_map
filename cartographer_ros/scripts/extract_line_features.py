#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
直线特征提取示例脚本

这个脚本展示了如何使用Cartographer的直线特征提取功能
从pbstream文件中提取直线特征。

使用方法:
    python extract_line_features.py --pbstream map.pbstream --config config.lua

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

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class LineFeatureExtractor:
    """直线特征提取器包装类"""
    
    def __init__(self, config_file=None):
        """
        初始化提取器
        
        Args:
            config_file (str): 配置文件路径
        """
        self.config_file = config_file or "line_extraction_config.lua"
        self.output_dir = "./line_features_output"
        
    def extract_from_pbstream(self, pbstream_path, output_dir=None):
        """
        从pbstream文件提取直线特征
        
        Args:
            pbstream_path (str): pbstream文件路径
            output_dir (str): 输出目录
            
        Returns:
            dict: 提取结果信息
        """
        if not os.path.exists(pbstream_path):
            raise FileNotFoundError(f"pbstream文件不存在: {pbstream_path}")
            
        if output_dir:
            self.output_dir = output_dir
            
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 构建命令
        cmd = [
            "rosrun", "cartographer_ros", "line_feature_extractor",
            "--pbstream_path", pbstream_path,
            "--config_file", self.config_file,
            "--output_dir", self.output_dir
        ]
        
        logger.info(f"开始提取直线特征: {pbstream_path}")
        logger.info(f"输出目录: {self.output_dir}")
        
        try:
            # 执行提取命令
            result = subprocess.run(
                cmd,
                check=True,
                capture_output=True,
                text=True,
                timeout=300  # 5分钟超时
            )
            
            logger.info("直线特征提取完成")
            logger.info(f"输出: {result.stdout}")
            
            # 分析输出结果
            return self._analyze_output()
            
        except subprocess.CalledProcessError as e:
            logger.error(f"提取失败: {e}")
            logger.error(f"错误输出: {e.stderr}")
            raise
        except subprocess.TimeoutExpired:
            logger.error("提取超时")
            raise
            
    def _analyze_output(self):
        """分析输出结果"""
        results = {
            "output_dir": self.output_dir,
            "files": [],
            "line_count": 0,
            "processing_time": 0.0
        }
        
        # 检查输出文件
        output_path = Path(self.output_dir)
        for file_path in output_path.glob("*"):
            results["files"].append(file_path.name)
            
        # 尝试读取JSON结果
        json_file = output_path / "line_features.json"
        if json_file.exists():
            try:
                with open(json_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                if "line_features" in data:
                    results["line_count"] = len(data["line_features"])
                    
                if "header" in data and "processing_time" in data["header"]:
                    results["processing_time"] = data["header"]["processing_time"]
                    
            except Exception as e:
                logger.warning(f"无法解析JSON结果文件: {e}")
                
        return results
        
    def batch_extract(self, pbstream_dir, output_base_dir=None):
        """
        批量提取直线特征
        
        Args:
            pbstream_dir (str): pbstream文件目录
            output_base_dir (str): 输出基础目录
            
        Returns:
            list: 所有提取结果
        """
        pbstream_dir = Path(pbstream_dir)
        if not pbstream_dir.exists():
            raise FileNotFoundError(f"目录不存在: {pbstream_dir}")
            
        if output_base_dir:
            base_output_dir = output_base_dir
        else:
            base_output_dir = "./batch_line_features"
            
        all_results = []
        
        # 查找所有pbstream文件
        pbstream_files = list(pbstream_dir.glob("*.pbstream"))
        
        if not pbstream_files:
            logger.warning(f"在目录 {pbstream_dir} 中未找到pbstream文件")
            return all_results
            
        logger.info(f"找到 {len(pbstream_files)} 个pbstream文件")
        
        for i, pbstream_file in enumerate(pbstream_files, 1):
            logger.info(f"处理文件 {i}/{len(pbstream_files)}: {pbstream_file.name}")
            
            # 为每个文件创建输出目录
            file_output_dir = os.path.join(
                base_output_dir, 
                pbstream_file.stem
            )
            
            try:
                result = self.extract_from_pbstream(
                    str(pbstream_file), 
                    file_output_dir
                )
                result["input_file"] = str(pbstream_file)
                all_results.append(result)
                
                logger.info(f"完成: {pbstream_file.name} - {result['line_count']} 条直线")
                
            except Exception as e:
                logger.error(f"处理文件失败 {pbstream_file.name}: {e}")
                all_results.append({
                    "input_file": str(pbstream_file),
                    "error": str(e),
                    "line_count": 0
                })
                
        return all_results
        
    def generate_report(self, results, report_file="extraction_report.json"):
        """
        生成提取报告
        
        Args:
            results (list): 提取结果列表
            report_file (str): 报告文件路径
        """
        if not results:
            logger.warning("没有结果数据可生成报告")
            return
            
        # 统计信息
        total_files = len(results)
        successful_files = len([r for r in results if "error" not in r])
        total_lines = sum(r.get("line_count", 0) for r in results)
        avg_lines = total_lines / successful_files if successful_files > 0 else 0
        total_time = sum(r.get("processing_time", 0) for r in results)
        
        report = {
            "summary": {
                "total_files": total_files,
                "successful_files": successful_files,
                "failed_files": total_files - successful_files,
                "total_lines_extracted": total_lines,
                "average_lines_per_file": avg_lines,
                "total_processing_time": total_time
            },
            "details": results
        }
        
        # 保存报告
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
            
        logger.info(f"报告已保存到: {report_file}")
        logger.info(f"总计: {total_files} 个文件, {total_lines} 条直线")
        
        return report


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="从Cartographer pbstream文件提取直线特征",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 单文件提取
  python extract_line_features.py --pbstream map.pbstream
  
  # 指定配置文件
  python extract_line_features.py --pbstream map.pbstream --config custom_config.lua
  
  # 批量提取
  python extract_line_features.py --batch-dir /path/to/pbstreams --output-dir ./results
  
  # 生成报告
  python extract_line_features.py --pbstream map.pbstream --generate-report
        """
    )
    
    parser.add_argument(
        "--pbstream", 
        type=str,
        help="pbstream文件路径"
    )
    
    parser.add_argument(
        "--batch-dir",
        type=str,
        help="批量处理目录（包含多个pbstream文件）"
    )
    
    parser.add_argument(
        "--config",
        type=str,
        default="line_extraction_config.lua",
        help="配置文件路径 (默认: line_extraction_config.lua)"
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default="./line_features_output",
        help="输出目录 (默认: ./line_features_output)"
    )
    
    parser.add_argument(
        "--generate-report",
        action="store_true",
        help="生成提取报告"
    )
    
    parser.add_argument(
        "--report-file",
        type=str,
        default="extraction_report.json",
        help="报告文件路径 (默认: extraction_report.json)"
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
        
    # 验证参数
    if not args.pbstream and not args.batch_dir:
        parser.error("必须指定 --pbstream 或 --batch-dir 参数")
        
    if args.pbstream and args.batch_dir:
        parser.error("--pbstream 和 --batch-dir 参数不能同时使用")
        
    # 创建提取器
    extractor = LineFeatureExtractor(args.config)
    
    try:
        if args.pbstream:
            # 单文件提取
            result = extractor.extract_from_pbstream(
                args.pbstream, 
                args.output_dir
            )
            
            print(f"\n✅ 提取完成!")
            print(f"📁 输出目录: {result['output_dir']}")
            print(f"📊 直线数量: {result['line_count']}")
            print(f"⏱️  处理时间: {result['processing_time']:.2f}s")
            
            if args.generate_report:
                extractor.generate_report([result], args.report_file)
                
        else:
            # 批量提取
            results = extractor.batch_extract(
                args.batch_dir,
                args.output_dir
            )
            
            if results:
                print(f"\n✅ 批量提取完成!")
                successful = len([r for r in results if "error" not in r])
                total_lines = sum(r.get("line_count", 0) for r in results)
                
                print(f"📁 处理文件: {len(results)}")
                print(f"✅ 成功: {successful}")
                print(f"❌ 失败: {len(results) - successful}")
                print(f"📊 总直线数: {total_lines}")
                
                if args.generate_report:
                    extractor.generate_report(results, args.report_file)
            else:
                print("❌ 没有找到可处理的文件")
                
    except KeyboardInterrupt:
        print("\n⚠️  用户中断操作")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()