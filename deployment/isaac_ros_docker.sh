#!/bin/bash
set -e  # 脚本出错时立即退出

# -------------------------- 配置参数（仅需修改这里） --------------------------
# 镜像名称（必填，替换为你的实际镜像名，如 byod-isaac-ros:latest）
IMAGE_NAME="10.4.0.233:5443/amr/byd-image/software/x86/ros2/ubuntu22.04:latest"
ISAAC_ROS_DEV_DIR="/home/test/leon/ros_ws"
# 容器内工作目录（固定）
WORK_DIR="/workspaces/ros-dev"
# 容器内入口脚本路径（固定）
ENTRYPOINT_SCRIPT="/usr/local/bin/scripts/byd-ws-entrypoint.sh"
# 容器内用户名（固定为 admin）
CONTAINER_USER="admin"

# 配置代理
HTTP_PROXY=""
HTTPS_PROXY=""
NO_PROXY=""

# ----- 打印日志
function print_color {
    tput setaf $1
    echo "$2"
    tput sgr0
}

function print_error {
    print_color 1 "$1"
}

function print_warning {
    print_color 3 "$1"
}

function print_info {
    print_color 2 "$1"
}

# -------------------------- 自动生成容器名（镜像名+当前用户名，去除特殊字符） --------------------------
# 提取镜像名（去除标签）并清理特殊字符
IMAGE_BASE_NAME=$(echo "$IMAGE_NAME" | cut -d: -f1 | tr '/:.' '-' )
# 当前宿主用户名
HOST_USER=$(id -un)
# 容器名 = 镜像名_base + 用户名（确保唯一）
CONTAINER_NAME="${IMAGE_BASE_NAME}-${HOST_USER}"

# -------------------------- 检查依赖 --------------------------
# 检查 Docker 是否安装
if ! command -v docker &> /dev/null; then
    echo "错误：未安装 Docker，请先安装 Docker。"
    exit 1
fi

# 检查 Docker 服务是否运行
if ! docker info &> /dev/null; then
    echo "错误：Docker 服务未运行，请启动 Docker 服务。"
    exit 1
fi

# 检查 X11 服务（可视化依赖）
if [ -z "$DISPLAY" ]; then
    echo "错误：未检测到 DISPLAY 环境变量，可视化应用需要 X11 支持。"
    exit 1
fi

# -------------------------- 容器操作逻辑 --------------------------
# 检查容器是否已存在
# 检查容器是否已存在
if docker inspect "$CONTAINER_NAME" &> /dev/null; then
    echo "容器 $CONTAINER_NAME 已存在，准备 attach 进入..."
    # 若容器未运行，先启动
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" = "false" ]; then
        echo "容器未运行，启动中..."
        docker start "$CONTAINER_NAME"
    fi
    # attach 到容器（--sig-proxy=false 确保 Ctrl+C 不会终止容器）
    docker exec -it -u admin "$CONTAINER_NAME" bash
    exit 0
fi

# -------------------------- 容器不存在时，创建并启动 --------------------------
echo "容器 $CONTAINER_NAME 不存在，创建并启动中..."

# 配置 X11 转发（允许容器访问宿主 X 服务）
xhost +local:"$CONTAINER_NAME" > /dev/null 2>&1

# 获取当前用户 UID/GID（用于用户映射）
HOST_UID=$(id -u)
HOST_GID=$(id -g)

# 启动容器（后台运行，--detach）
DOCKER_ARGS+=("-e USER")
DOCKER_ARGS+=("-e ISAAC_ROS_WS=/workspaces/ros_ws")
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
#DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
docker run -itd \
    --name "$CONTAINER_NAME" \
    --network host \
    --ipc=host \
    --privileged \
    -e UID=$HOST_UID \
    -e GID=$HOST_GID \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e HTTP_PROXY=$HTTP_PROXY \
    -e HTTPS_PROXY=$HTTPS_PROXY \
    -e NO_PROXY=$NO_PROXY \
    ${DOCKER_ARGS[@]} \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    -v $ISAAC_ROS_DEV_DIR:/workspaces/ros-dev \
    -w "$WORK_DIR" \
    --entrypoint $ENTRYPOINT_SCRIPT \
    "$IMAGE_NAME"

# 等待容器启动（给入口脚本预留初始化时间）
sleep 2

# 首次启动后自动 attach 到容器
echo "容器启动成功，进入终端..."
docker exec -it -u admin "$CONTAINER_NAME" /usr/local/bin/start-web-terminal.sh
# -------------------------- 清理 X11 权限（退出容器后执行） --------------------------
xhost -local:"$CONTAINER_NAME" > /dev/null 2>&1
echo "已退出容器，容器仍在后台运行（名称：$CONTAINER_NAME）"
