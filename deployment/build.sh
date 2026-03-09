#!/bin/bash

# 默认配置
DEFAULT_TARGET_PACKAGE="your_target_package"  # 默认目标包
DEFAULT_BUILD_DIR="build"                     # 构建目录
DEFAULT_INSTALL_DIR="install"                 # 安装目录（符号链接指向此处）
DEFAULT_LOG_DIR="log"                         # 日志目录
DEFAULT_JOBS=$(nproc)                         # 并行线程数（默认CPU核心数）
DEFAULT_CMAKE_BUILD_TYPE="Release"            # 并行线程数（默认CPU核心数）
SYMLINK_ENABLED=true                          # 默认启用symlink模式

# 显示帮助信息
show_help() {
    echo "ROS 2 colcon build 符号链接模式构建脚本（默认启用 --symlink-install）"
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  -t, --target <包名>   指定目标包（--packages-up-to 的参数，默认: $DEFAULT_TARGET_PACKAGE）"
    echo "  -b, --build <目录>    构建目录（默认: $DEFAULT_BUILD_DIR）"
    echo "  -i, --install <目录>  安装目录（默认: $DEFAULT_INSTALL_DIR）"
    echo "  -l, --log <目录>      日志目录（默认: $DEFAULT_LOG_DIR）"
    echo "  -j, --jobs <数量>     并行编译数（默认: $DEFAULT_JOBS）"
    echo "  -c, --cmake <类型>    cmake_build_type（默认: $DEFAULT_CMAKE_BUILD_TYPE）"
    echo "  --no-symlink          禁用符号链接模式（使用常规安装）"
    echo "  -h, --help            显示帮助信息"
    echo
    echo "示例:"
    echo "  $0 -t my_package        # 用symlink模式编译my_package及其依赖"
    echo "  $0 -t my_package --no-symlink  # 禁用symlink，常规安装"
    echo "  $0 -t my_package --no-symlink -c Debug # 禁用Release,常规调试安装"
}

# 解析命令行参数
TARGET_PACKAGE=$DEFAULT_TARGET_PACKAGE
BUILD_DIR=$DEFAULT_BUILD_DIR
INSTALL_DIR=$DEFAULT_INSTALL_DIR
LOG_DIR=$DEFAULT_LOG_DIR
JOBS=$DEFAULT_JOBS
CMAKE_BUILD_TYPE=$DEFAULT_CMAKE_BUILD_TYPE           #默认Release
SYMLINK_FLAG="--symlink-install"  # 默认启用

while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--target)
            TARGET_PACKAGE="$2"
            shift 2
            ;;
        -b|--build)
            BUILD_DIR="$2"
            shift 2
            ;;
        -i|--install)
            INSTALL_DIR="$2"
            shift 2
            ;;
        -l|--log)
            LOG_DIR="$2"
            shift 2
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
	-c|--cmake)
            CMAKE_BUILD_TYPE="$2"
            shift 2
            ;;

        --no-symlink)
            SYMLINK_FLAG=""  # 禁用symlink
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "错误：未知选项 $1"
            show_help
            exit 1
            ;;
    esac
done

# 检查目标包是否为空
if [[ -z "$TARGET_PACKAGE" ]]; then
    echo "错误：目标包名不能为空！"
    exit 1
fi

# 构建命令（核心：包含--symlink-install，可通过--no-symlink禁用）
echo "===== 开始构建: 目标包=$TARGET_PACKAGE，并行数=$JOBS ====="
echo "符号链接模式: $( [[ -n "$SYMLINK_FLAG" ]] && echo "启用" || echo "禁用" )"

colcon build \
    --packages-up-to "$TARGET_PACKAGE" \
    --build-base "$BUILD_DIR" \
    --install-base "$INSTALL_DIR" \
    --parallel-workers "$JOBS" \
    $SYMLINK_FLAG \
    --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE -DCMAKE_CXX_FLAGS="-O2 -g -Wall"

# 检查构建结果
if [[ $? -eq 0 ]]; then
    echo "===== 构建成功！安装目录: $INSTALL_DIR ====="
    echo "提示：修改源码后无需重新build，直接运行节点即可（symlink模式下自动生效）"
    # 自动加载环境变量（推荐启用，否则需手动source）
    echo "加载环境变量..."
    source "$INSTALL_DIR/setup.bash"
    echo "完成当前install环境目录加载..."
else
    echo "===== 构建失败！ ====="
    exit 1
fi
