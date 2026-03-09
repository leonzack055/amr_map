#!/bin/bash                                                                                                                                    
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency

#!/bin/bash
set -xe  # 出错时立即退出，便于排查

# 1. 使用正确的环境变量 USER_UID 和 USER_GID（假设 GID 对应变量是 USER_GID）
# 检查变量是否存在，避免为空
if [ -z "$UID" ] || [ -z "$GID" ]; then
  echo "错误：环境变量 UID 或 GID 未设置"
  exit 1
fi

# 2. 修改 jtop 组的 GID（若 jtop 组存在，否则可忽略或添加创建组的命令）
#sudo groupmod -g "$USER_GID" jtop || echo "警告：jtop 组不存在，跳过组 GID 修改"

# 3. 修改 admin 用户的 UID 和 GID（确保与宿主一致）
sudo usermod -u "$UID" admin || echo "修改 admin UID"
sudo groupmod -g "$GID" admin || echo "修改 admin GID 失败"
# 同时修改 admin 主组的 GID（关键）

# 4. 修复因 UID/GID 变更导致的权限问题（如工作目录）
# 假设工作目录为 /workspace，根据实际情况修改
echo "chown /workspaces 到admin用户空间"
sudo chown -R admin:admin /workspaces

# 5. 切换到 admin 用户，并执行传递的参数（$@）
# 使用 exec 替换当前进程，确保后续命令在 admin 身份下执行
echo "切换到 admin 用户并执行命令..."
#exec su -l admin -c "$*"
#exec su -l admin
su -l admin -c "$*"


