# Docker 容器内 Nginx + GoTTY Web终端部署方案

## 概述

本文档描述如何在已有的 Docker 容器（Ubuntu 22.04）内安装和配置 Nginx 与 GoTTY，实现通过 Web 浏览器访问后台终端。

**特别注意**：本文档针对使用 UID/GID 模式启动的容器，通过 `entry_point.sh` 切换到非 root 用户（admin）运行。

### 方案选择建议

| 方案 | 推荐度 | 适用场景 | 优点 | 缺点 |
|------|--------|----------|------|------|
| **脚本启动** | ⭐⭐⭐⭐⭐ | 通用 Docker 容器 | 简单可靠，无额外依赖 | 需手动管理进程 |
| **Supervisord** | ⭐⭐⭐⭐ | 需要进程守护 | 自动重启，日志管理 | 额外安装 supervisor |
| **Systemd** | ⭐⭐ | 特殊需求场景 | 功能完整，原生服务管理 | 需特权模式，配置复杂 |

**结论**：对于普通 Docker 容器，**强烈推荐使用脚本启动方式**（第五章方案一或方案二）。Systemd 方案仅在以下特殊情况考虑：
- 容器本身就是为 systemd 设计的专用镜像
- 需要完整的 init 系统管理多个服务
- 已有基础设施依赖 systemd

**为什么 Systemd 在 Docker 中不推荐**：
1. Docker 设计理念是"一个容器一个进程"，systemd 违背了这一原则
2. 需要特权模式运行，存在安全风险
3. 需要挂载 cgroup 文件系统，增加配置复杂度
4. UID/GID 动态设置与 systemd 服务用户配置冲突
5. 容器重启后 systemd 状态可能不一致

### 架构说明

```
┌─────────────────────────────────────────────────────────────┐
│                        用户浏览器                            │
│                   http://your-ip:8080/terminal              │
│                   http://your-ip:8080/files (文件服务)       │
└─────────────────────────┬───────────────────────────────────┘
                          │ HTTP
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    Docker 容器 (Ubuntu 22.04)               │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                    Nginx (:8080)                     │   │
│  │  - 以 admin 用户运行 (无 user 指令)                  │   │
│  │  - 静态页面服务                                      │   │
│  │  - 反向代理 /terminal → GoTTY                       │   │
│  │  - 文件服务 /files → /home/admin/map_dir            │   │
│  │  - WebSocket 支持                                    │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │ HTTP (localhost:9090)             │
│                         ▼                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                    GoTTY (:9090)                     │   │
│  │  - 以 admin 用户运行                                 │   │
│  │  - WebSocket 终端服务                                │   │
│  │  - Bash/Zsh Shell                                   │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              文件目录 (/home/admin/map_dir)          │   │
│  │  - 地图文件存储                                      │   │
│  │  - WebDAV 支持上传/下载/修改                         │   │
│  │  - 文件所有者为 admin                                │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 一、UID/GID 模式说明

### 1.1 entry_point.sh 工作原理

```bash
#!/bin/bash
# entry_point.sh - 容器入口脚本
sudo groupmod -g $GID jtop
sudo usermod -u $UID admin
su admin -s /bin/bash "$@"
```

**执行流程**：
1. 容器以 root 启动
2. 修改 `jtop` 组的 GID 为环境变量 `$GID`
3. 修改 `admin` 用户的 UID 为环境变量 `$UID`
4. 使用 `su` 切换到 `admin` 用户执行后续命令

### 1.2 服务启动策略

由于容器启动后以 `admin` 用户运行，Nginx 和 GoTTY 也需要以 `admin` 用户身份运行：

| 服务 | 运行用户 | 端口 | 注意事项 |
|------|----------|------|----------|
| Nginx | admin | 8080 | **不能使用 `user` 指令**，必须使用非特权端口(≥1024) |
| GoTTY | admin | 9090 | 正常运行，避免与 Nginx 端口冲突 |
| 文件服务 | admin | - | 通过 Nginx 提供，文件所有者为 admin |

### 1.3 端口分配说明

| 服务 | 容器内端口 | 宿主机端口 | 说明 |
|------|-----------|-----------|------|
| Nginx | 8080 | 8080 | 主入口（Web 界面 + 文件服务） |
| GoTTY | 9090 | - | 仅本地监听，由 Nginx 代理 |

**重要**：非 root 用户无法绑定 1024 以下的端口，因此：
- Nginx 使用 8080 端口（而非默认的 80）
- GoTTY 使用 9090 端口（避免与 Nginx 冲突）

---

## 二、安装步骤

### 2.1 安装 Nginx（以 root 执行）

```bash
# 更新包列表
apt-get update

# 安装 Nginx
apt-get install -y nginx

# 安装额外工具
apt-get install -y wget sudo

# 验证安装
nginx -v
# 输出: nginx version: nginx/1.18.0 (Ubuntu)
```

### 2.2 安装 GoTTY（以 root 执行）

```bash
# 下载 GoTTY
GOTTY_VERSION="1.0.1"
wget -q https://github.com/yudai/gotty/releases/download/v${GOTTY_VERSION}/gotty_linux_amd64.tar.gz

# 解压并安装
tar -xzf gotty_linux_amd64.tar.gz
mv gotty /usr/local/bin/
chmod +x /usr/local/bin/gotty

# 验证安装
gotty --version
# 输出: gotty version 1.0.1

# 清理
rm gotty_linux_amd64.tar.gz
```

### 2.3 创建必要目录（以 root 执行）

```bash
# GoTTY 配置和日志目录
mkdir -p /etc/gotty
mkdir -p /var/log/gotty

# Nginx 目录
mkdir -p /var/www/html
mkdir -p /var/lib/nginx/body
mkdir -p /var/lib/nginx/proxy
mkdir -p /var/lib/nginx/fastcgi
mkdir -p /var/lib/nginx/uwsgi
mkdir -p /var/lib/nginx/scgi

# 文件服务目录 (地图文件存储)
mkdir -p /home/admin/map_dir

# 设置目录权限 - 让 admin 用户可以访问
chown -R admin:admin /var/log/gotty
chown -R admin:admin /var/www/html
chown -R admin:admin /var/lib/nginx
chown -R admin:admin /var/log/nginx
chown -R admin:admin /run/nginx 2>/dev/null || mkdir -p /run/nginx && chown -R admin:admin /run/nginx
chown -R admin:admin /home/admin/map_dir
```

或者在docker内执行

```bash
cd ./amr_map/deployment
sudo ./create_dirs.sh
```

---

## 三、GoTTY 配置

### 3.1 创建配置文件

**文件：`/etc/gotty/config`**

```bash
cat > /etc/gotty/config << 'EOF'
# GoTTY 配置文件

# 监听地址 (0.0.0.0 允许外部访问，通过 Nginx 代理)
# 注意：如果只允许容器内访问，可改为 "127.0.0.1"
address = "0.0.0.0"

# 监听端口 (避免与 Nginx 8080 冲突)
port = 9090

# 允许远程写入 (允许执行命令)
permit_write = true

# 允许远程重新连接
reconnect = true

# 重新连接延迟 (秒)
reconnect_timeout = 10

# 终端类型
term = "xterm-256color"

# 窗口标题格式
title_format = "AMR Terminal"

# 自动重连
once = false

# 关闭客户端连接时退出命令
close_signal = 1  # SIGHUP

# 首选 WebSocket
prefer_websocket = true

# 宽度和高度 (0 = 自动)
width = 0
height = 0

# 滚动回溯行数
scrollback = 10000

# 字体大小 (像素)
font-size = 14

# 字体家族
font-family = "Consolas, Monaco, 'Courier New', monospace"
EOF
```

或者在docker内执行

```bash
sudo ./create_gotty_config.sh
```

### 3.2 创建启动脚本

**文件：`/usr/local/bin/start-gotty.sh`**

```bash
cat > /usr/local/bin/start-gotty.sh << 'EOF'
#!/bin/bash
# GoTTY 启动脚本

CONFIG_FILE="/etc/gotty/config"
SHELL="/bin/bash"

echo "Starting GoTTY on 127.0.0.1:9090..."
exec gotty --config "$CONFIG_FILE" "$SHELL"
EOF

chmod +x /usr/local/bin/start-gotty.sh
```

或者在docker内执行

```bash
sudo ./create_gotty_start.sh
```

---

## 四、Nginx 配置（admin 用户运行）

### 4.1 修改主配置文件

**文件：`/etc/nginx/nginx.conf`**

```bash
cat > /etc/nginx/nginx.conf << 'EOF'
# Nginx 主配置 - 以 admin 用户运行
# 注意：当以非 root 用户运行时，不能使用 "user" 指令
# 如果添加 "user admin;" 会导致警告并被忽略

worker_processes auto;
pid /run/nginx/nginx.pid;
error_log /var/log/nginx/error.log notice;

events {
    worker_connections 1024;
}

http {
    include /etc/nginx/mime.types;
    default_type application/octet-stream;

    log_format main '$remote_addr - $remote_user [$time_local] "$request" '
                    '$status $body_bytes_sent "$http_referer" '
                    '"$http_user_agent"';

    access_log /var/log/nginx/access.log main;

    sendfile on;
    tcp_nopush on;
    tcp_nodelay on;
    keepalive_timeout 65;
    types_hash_max_size 2048;

    # 临时文件路径 (admin 可写)
    client_body_temp_path /var/lib/nginx/body;
    proxy_temp_path /var/lib/nginx/proxy;
    fastcgi_temp_path /var/lib/nginx/fastcgi;
    uwsgi_temp_path /var/lib/nginx/uwsgi;
    scgi_temp_path /var/lib/nginx/scgi;

    # Gzip 压缩
    gzip on;
    gzip_vary on;
    gzip_proxied any;
    gzip_comp_level 6;
    gzip_types text/plain text/css text/xml application/json application/javascript;

    # 包含站点配置
    include /etc/nginx/sites-enabled/*;
}
EOF
```

或者在docker内执行

```bash
sudo ./create_nginx_conf.sh
```

### 4.2 创建首页

**文件：`/var/www/html/index.html`**

```bash
cat > /var/www/html/index.html << 'EOF'
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AMR 建图系统 - Web终端</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            color: #fff;
        }
        .container { text-align: center; padding: 40px; }
        h1 {
            font-size: 2.5rem;
            margin-bottom: 20px;
            background: linear-gradient(90deg, #00d2ff, #3a7bd5);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        p { color: #a0a0a0; margin-bottom: 30px; font-size: 1.1rem; }
        .btn {
            display: inline-block;
            padding: 15px 40px;
            background: linear-gradient(90deg, #00d2ff, #3a7bd5);
            color: #fff;
            text-decoration: none;
            border-radius: 30px;
            font-weight: bold;
            font-size: 1.1rem;
            transition: transform 0.3s, box-shadow 0.3s;
            box-shadow: 0 4px 15px rgba(0, 210, 255, 0.3);
        }
        .btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 6px 20px rgba(0, 210, 255, 0.5);
        }
        .info {
            margin-top: 40px;
            padding: 20px;
            background: rgba(255,255,255,0.05);
            border-radius: 10px;
            font-size: 0.9rem;
            color: #888;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 AMR 建图系统</h1>
        <p>通过 Web 终端远程管理和监控您的建图任务</p>
        <a href="/terminal/" class="btn">打开终端</a>
        <div class="info">
            <p>终端连接需要 WebSocket 支持</p>
        </div>
    </div>
</body>
</html>
EOF

chown admin:admin /var/www/html/index.html
```

或者在docker内执行

```bash
sudo ./create_gotty_html.sh
```


### 4.3 创建站点配置

**文件：`/etc/nginx/sites-available/web-terminal`**

```bash
cat > /etc/nginx/sites-available/web-terminal << 'EOF'
# GoTTY 上游服务
upstream gotty {
    server 127.0.0.1:9090;
}

server {
    # 非 root 用户必须使用 >= 1024 的端口
    listen 8080;
    server_name localhost;

    # 根目录
    root /var/www/html;
    index index.html;

    # 安全头
    add_header X-Frame-Options "SAMEORIGIN" always;
    add_header X-Content-Type-Options "nosniff" always;
    add_header X-XSS-Protection "1; mode=block" always;

    # 首页
    location / {
        try_files $uri $uri/ =404;
    }

    # GoTTY 终端代理 (关键配置)
    location /terminal/ {
        # 反向代理到 GoTTY
        proxy_pass http://gotty/;
        # 必须禁止 Origin 头 (必须)
        proxy_set_header Origin "";
        proxy_redirect   off;

        # WebSocket 支持 (必须)
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        
        # 代理头设置
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        # 超时设置 (长时间运行的命令需要)
        proxy_read_timeout 86400;
        proxy_send_timeout 86400;
        
        # 缓冲设置
        proxy_buffering off;
    }
  
    # GoTTY 静态资源
    location /terminal/js/ {
        proxy_pass http://gotty/js/;
    }

    location /terminal/css/ {
        proxy_pass http://gotty/css/;
    }

    # ========================================
    # 文件服务 - 地图文件管理
    # ========================================
    # 访问地址: c
    
    location /files/ {
        alias /home/admin/map_dir/;
        
        # 目录浏览
        autoindex on;
        autoindex_exact_size off;
        autoindex_localtime on;
        
        # 允许所有 HTTP 方法 (用于 WebDAV)
        dav_methods PUT DELETE MKCOL COPY MOVE;
        
        # 允许文件上传
        dav_access user:rw group:rw all:r;
        
        # 创建临时文件目录 (用于上传)
        client_body_temp_path /var/lib/nginx/body;
        
        # 上传文件大小限制 (0 = 无限制)
        client_max_body_size 0;
        
        # 允许覆盖已存在的文件
        create_full_put_path on;
        
        # 认证 (可选，建议启用)
        # auth_basic "File Server";
        # auth_basic_user_file /etc/nginx/.htpasswd;
    }

    # 健康检查
    location /health {
        access_log off;
        return 200 "OK\n";
        add_header Content-Type text/plain;
    }
}
EOF
```

### 4.4 文件服务配置说明

**关于文件所有者问题**：

当 Nginx 以 `admin` 用户运行时，通过 WebDAV 上传或修改的文件会自动以 `admin` 用户身份创建，文件所有者就是 `admin`。这是因为：

1. Nginx 工作进程以 `admin` 用户运行
2. WebDAV 操作（PUT、DELETE、MKCOL 等）由工作进程执行
3. 新创建的文件继承进程的用户身份

**验证文件所有者**：
```bash
# 在容器内检查 Nginx 进程用户
ps aux | grep nginx
# 输出应显示: admin ... nginx: worker process

# 上传文件后检查所有者
ls -la /home/admin/map_dir/
# 输出应显示: -rw-r--r-- 1 admin admin ... filename
```

**可选：添加 HTTP 基本认证**：

```bash
# 安装 apache2-utils (包含 htpasswd)
apt-get install -y apache2-utils

# 创建密码文件
htpasswd -cb /etc/nginx/.htpasswd admin your_password

# 设置权限
chown admin:admin /etc/nginx/.htpasswd
chmod 600 /etc/nginx/.htpasswd
```

然后取消站点配置中 `auth_basic` 相关行的注释。

或者在docker内执行

```bash
sudo ./create_nginx_site.sh
```

### 4.5 启用站点配置

```bash
# 删除默认站点
rm -f /etc/nginx/sites-enabled/default

# 启用新站点
ln -sf /etc/nginx/sites-available/web-terminal /etc/nginx/sites-enabled/

# 测试配置 (应该显示成功，无警告)
nginx -t
# 预期输出:
# nginx: the configuration file /etc/nginx/nginx.conf syntax is ok
# nginx: configuration file /etc/nginx/nginx.conf test is successful
```

---

### 4.6 容器内执行方式
容器内可直接用脚本启动，默认当前用户组admin与主机端docker启动用户绑定。无执行限制
```bash
start-web-terminal.sh

Starting Web Terminal services...
Starting Nginx...
Starting GoTTY...
✓ Nginx is running (port 8080)
✓ GoTTY is running (port 9090)

==========================================
  Web Terminal Ready!
==========================================
  Homepage:  http://localhost:8080/
  Terminal:  http://localhost:8080/terminal/
  Files:     http://localhost:8080/files/
==========================================
```

## 五、集成到 entry_point.sh

### 5.1 修改 entry_point.sh

**方案一：在 entry_point.sh 中启动服务**

```bash
#!/bin/bash
# entry_point.sh - 修改后的容器入口脚本

# 修改用户 UID/GID
sudo groupmod -g $GID jtop
sudo usermod -u $UID admin

# 确保目录权限正确
sudo chown -R admin:admin /var/log/gotty
sudo chown -R admin:admin /var/log/nginx
sudo chown -R admin:admin /run/nginx 2>/dev/null || sudo mkdir -p /run/nginx && sudo chown -R admin:admin /run/nginx

# 切换到 admin 用户并启动服务
su admin -s /bin/bash -c "
    # 启动 Nginx (后台)
    nginx
    
    # 启动 GoTTY (后台)
    nohup /usr/local/bin/start-gotty.sh > /var/log/gotty/gotty.log 2>&1 &
    
    echo 'Web Terminal services started'
    echo 'Access: http://localhost:8080/'
    echo 'Terminal: http://localhost:8080/terminal/'
    echo 'Files: http://localhost:8080/files/'
"

# 执行用户传入的命令
su admin -s /bin/bash "$@"
```

**方案二：创建独立启动脚本**

```bash
#!/bin/bash
# entry_point.sh - 保持原样，通过 CMD 启动服务

sudo groupmod -g $GID jtop
sudo usermod -u $UID admin
su admin -s /bin/bash "$@"
```

创建服务启动脚本 `/usr/local/bin/start-web-terminal.sh`：

```bash
cat > /usr/local/bin/start-web-terminal.sh << 'EOF'
#!/bin/bash
# Web Terminal 服务启动脚本

echo "Starting Web Terminal services..."

# 启动 Nginx
echo "Starting Nginx..."
nginx
if [ $? -ne 0 ]; then
    echo "Failed to start Nginx"
    exit 1
fi

# 启动 GoTTY
echo "Starting GoTTY..."
nohup /usr/local/bin/start-gotty.sh > /var/log/gotty/gotty.log 2>&1 &
sleep 1

# 检查服务状态
if pgrep -x "nginx" > /dev/null; then
    echo "✓ Nginx is running (port 8080)"
else
    echo "✗ Nginx failed to start"
fi

if pgrep -x "gotty" > /dev/null; then
    echo "✓ GoTTY is running (port 8081)"
else
    echo "✗ GoTTY failed to start"
    cat /var/log/gotty/gotty.log
fi

echo ""
echo "=========================================="
echo "  Web Terminal Ready!"
echo "=========================================="
echo "  Homepage:  http://localhost:8080/"
echo "  Terminal:  http://localhost:8080/terminal/"
echo "  Files:     http://localhost:8080/files/"
echo "=========================================="

# 保持容器运行 (如果作为主命令)
if [ "$1" = "--daemon" ]; then
    tail -f /dev/null
fi
EOF

chmod +x /usr/local/bin/start-web-terminal.sh
```

### 5.2 Docker 运行方式

**方式一：通过 entry_point.sh 启动服务**

```bash
# Dockerfile
ENTRYPOINT ["/entry_point.sh"]
CMD ["/usr/local/bin/start-web-terminal.sh", "--daemon"]

# 运行容器 (注意：容器内端口改为 8080)
docker run -d \
    -e UID=1000 \
    -e GID=1000 \
    -p 8080:8080 \
    -v /path/to/maps:/home/admin/map_dir \
    --name web-terminal \
    your-image:tag
```

**方式二：启动后手动启动服务**

```bash
# 运行容器
docker run -d \
    -e UID=1000 \
    -e GID=1000 \
    -p 8080:8080 \
    -v /path/to/maps:/home/admin/map_dir \
    --name web-terminal \
    your-image:tag \
    tail -f /dev/null

# 进入容器启动服务
docker exec -it web-terminal bash
# 在容器内执行:
/usr/local/bin/start-web-terminal.sh
```

**方式三：使用 supervisord 管理服务**

```bash
# 安装 supervisord
apt-get install -y supervisor

# 创建配置文件
cat > /etc/supervisor/conf.d/web-terminal.conf << 'EOF'
[program:nginx]
command=/usr/sbin/nginx -g "daemon off;"
autostart=true
autorestart=true
user=admin
stdout_logfile=/var/log/nginx/supervisor.log
stderr_logfile=/var/log/nginx/supervisor_err.log

[program:gotty]
command=/usr/local/bin/start-gotty.sh
autostart=true
autorestart=true
user=admin
stdout_logfile=/var/log/gotty/supervisor.log
stderr_logfile=/var/log/gotty/supervisor_err.log
EOF

# 修改 entry_point.sh
cat > /entry_point.sh << 'EOF'
#!/bin/bash
sudo groupmod -g $GID jtop
sudo usermod -u $UID admin
sudo chown -R admin:admin /var/log/gotty /var/log/nginx /run/nginx
exec /usr/bin/supervisord -n -c /etc/supervisor/supervisord.conf
EOF
```

---

## 六、Docker 端口映射

### 6.1 运行时端口映射

```bash
# 将容器内 8080 端口映射到宿主机 8080 端口
docker run -d \
    -e UID=1000 \
    -e GID=1000 \
    -p 8080:8080 \
    -v /path/to/maps:/home/admin/map_dir \
    --name web-terminal \
    your-image:tag \
    /usr/local/bin/start-web-terminal.sh --daemon

# 访问地址
# 首页: http://your-host-ip:8080/
# 终端: http://your-host-ip:8080/terminal/
# 文件: http://your-host-ip:8080/files/
```

### 6.2 使用 host 网络模式

```bash
# 使用主机网络 (无需端口映射)
docker run -d \
    -e UID=1000 \
    -e GID=1000 \
    --network host \
    -v /path/to/maps:/home/admin/map_dir \
    --name web-terminal \
    your-image:tag \
    /usr/local/bin/start-web-terminal.sh --daemon

# 访问地址
# 首页: http://your-host-ip:8080/
# 终端: http://your-host-ip:8080/terminal/
# 文件: http://your-host-ip:8080/files/
```

### 6.3 Docker Compose 方式

```yaml
version: '3.8'
services:
  web-terminal:
    image: your-image:tag
    container_name: web-terminal
    environment:
      - UID=1000
      - GID=1000
    ports:
      - "8080:8080"
    volumes:
      - /path/to/maps:/home/admin/map_dir
    command: /usr/local/bin/start-web-terminal.sh --daemon
    restart: unless-stopped
```

---

## 七、验证与测试

### 7.1 服务状态检查

```bash
# 检查 Nginx 状态
pgrep -a nginx
# 输出: nginx: master process /usr/sbin/nginx

# 检查 GoTTY 状态
pgrep -a gotty
# 输出: gotty --config /etc/gotty/config /bin/bash

# 检查端口监听
ss -tlnp | grep -E "(8080|9090)"
# 输出:
# LISTEN  0  128  0.0.0.0:8080    0.0.0.0:*  users:(("nginx",pid=...))
# LISTEN  0  128  127.0.0.1:9090  0.0.0.0:*  users:(("gotty",pid=...))
```

### 7.2 功能测试

```bash
# 测试首页
curl -s http://localhost:8080/ | head -5

# 测试健康检查
curl http://localhost:8080/health
# 输出: OK

# 测试文件服务
curl -s http://localhost:8080/files/
# 输出目录列表 (如果有文件)

# 上传文件测试 (WebDAV)
echo "test content" > /tmp/test.txt
curl -T /tmp/test.txt http://localhost:8080/files/test.txt

# 验证文件所有者
ls -la /home/admin/map_dir/test.txt
# 输出: -rw-r--r-- 1 admin admin ... test.txt

# 查看 GoTTY 日志
tail -f /var/log/gotty/gotty.log

# 查看 Nginx 日志
tail -f /var/log/nginx/access.log
```

---

## 八、使用 Systemd 管理服务（不推荐）

> ⚠️ **警告**：此方案仅作为技术参考，**不推荐在普通 Docker 容器中使用**。
>
> 对于大多数场景，请使用第五章的脚本启动方式或 Supervisord 方案。

### 8.1 Docker 容器中使用 Systemd 的前提条件

**重要说明**：默认 Docker 容器不运行 systemd init 系统。要在容器中使用 systemd 管理服务，需要满足以下条件：

| 条件 | 说明 |
|------|------|
| 基础镜像 | 需要使用支持 systemd 的镜像，或在 Ubuntu 22.04 基础上安装 systemd |
| 容器权限 | 需要 `--privileged` 或 `--cap-add SYS_ADMIN` |
| Cgroup 挂载 | 需要挂载 `/sys/fs/cgroup` |
| 入口点 | 使用 `/sbin/init` 或 `/lib/systemd/systemd` 作为 PID 1 |

### 8.2 创建支持 Systemd 的 Docker 镜像

**Dockerfile 示例**：

```dockerfile
FROM ubuntu:22.04

# 避免 apt 交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 安装 systemd 和必要工具
RUN apt-get update && apt-get install -y \
    systemd \
    systemd-sysv \
    nginx \
    wget \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 安装 GoTTY
RUN wget -q https://github.com/yudai/gotty/releases/download/v1.0.1/gotty_linux_amd64.tar.gz && \
    tar -xzf gotty_linux_amd64.tar.gz && \
    mv gotty /usr/local/bin/ && \
    chmod +x /usr/local/bin/gotty && \
    rm gotty_linux_amd64.tar.gz

# 创建必要目录
RUN mkdir -p /etc/gotty /var/log/gotty /run/nginx

# 复制配置文件
COPY gotty-config /etc/gotty/config
COPY nginx.conf /etc/nginx/nginx.conf
COPY web-terminal /etc/nginx/sites-available/web-terminal
COPY gotty.service /etc/systemd/system/gotty.service
COPY entry_point.sh /entry_point.sh

# 启用服务
RUN systemctl enable nginx gotty

# 暴露端口
EXPOSE 80

# 使用 systemd 作为入口点
ENTRYPOINT ["/sbin/init"]
```

### 8.3 创建 Systemd 服务文件

**GoTTY 服务文件：`/etc/systemd/system/gotty.service`**

```ini
[Unit]
Description=GoTTY Web Terminal
Documentation=https://github.com/yudai/gotty
After=network.target

[Service]
Type=simple
# 以 admin 用户运行（需要 UID/GID 动态设置，见下方说明）
User=admin
Group=admin
ExecStart=/usr/local/bin/gotty --config /etc/gotty/config /bin/bash
Restart=always
RestartSec=3

# 日志
StandardOutput=journal
StandardError=journal

# 资源限制
LimitNOFILE=65536

[Install]
WantedBy=multi-user.target
```

**Nginx 服务文件（Ubuntu 默认已有，可自定义）：`/etc/systemd/system/nginx.service`**

```ini
[Unit]
Description=The NGINX HTTP and reverse proxy server
After=network.target remote-fs.target nss-lookup.target

[Service]
Type=forking
PIDFile=/run/nginx/nginx.pid
# 以 admin 用户运行
User=admin
Group=admin
ExecStartPre=/usr/sbin/nginx -t
ExecStart=/usr/sbin/nginx
ExecReload=/bin/kill -s HUP $MAINPID
ExecStop=/bin/kill -s QUIT $MAINPID
PrivateTmp=true

[Install]
WantedBy=multi-user.target
```

### 8.4 UID/GID 模式下的 Systemd 配置

由于 UID/GID 是动态设置的，需要修改 `entry_point.sh` 在 systemd 启动前设置用户：

**方案一：使用 systemd override drop-in**

```bash
#!/bin/bash
# entry_point.sh - systemd 模式

# 设置用户 UID/GID
groupmod -g $GID jtop
usermod -u $UID admin

# 创建 systemd override 目录
mkdir -p /etc/systemd/system/nginx.service.d
mkdir -p /etc/systemd/system/gotty.service.d

# 动态设置服务用户
cat > /etc/systemd/system/nginx.service.d/override.conf << EOF
[Service]
User=admin
Group=admin
EOF

cat > /etc/systemd/system/gotty.service.d/override.conf << EOF
[Service]
User=admin
Group=admin
EOF

# 重新加载 systemd
systemctl daemon-reload

# 设置目录权限
chown -R admin:admin /var/log/gotty
chown -R admin:admin /var/log/nginx
chown -R admin:admin /var/lib/nginx
chown -R admin:admin /run/nginx

# 启动服务
systemctl start nginx
systemctl start gotty

# 保持容器运行
exec tail -f /dev/null
```

**方案二：使用 systemd tmpfiles.d**

```bash
# /etc/tmpfiles.d/web-terminal.conf
# 确保运行时目录存在并设置权限
d /run/nginx 0755 admin admin -
d /var/log/gotty 0755 admin admin -
```

### 8.5 运行支持 Systemd 的容器

```bash
# 构建镜像
docker build -t web-terminal-systemd .

# 运行容器（需要特权模式）
docker run -d \
    --name web-terminal \
    --privileged \
    -v /sys/fs/cgroup:/sys/fs/cgroup:ro \
    -e UID=1000 \
    -e GID=1000 \
    -p 8080:80 \
    web-terminal-systemd

# 或者使用最小权限
docker run -d \
    --name web-terminal \
    --cap-add SYS_ADMIN \
    -v /sys/fs/cgroup:/sys/fs/cgroup:ro \
    -e UID=1000 \
    -e GID=1000 \
    -p 8080:80 \
    web-terminal-systemd
```

### 8.6 Systemd 服务管理命令

```bash
# 进入容器
docker exec -it web-terminal bash

# 查看服务状态
systemctl status nginx
systemctl status gotty

# 启动/停止/重启服务
systemctl start nginx
systemctl stop nginx
systemctl restart nginx

# 查看日志
journalctl -u nginx -f
journalctl -u gotty -f

# 启用/禁用开机自启
systemctl enable nginx gotty
systemctl disable nginx gotty

# 查看所有服务状态
systemctl list-units --type=service
```

### 8.7 Systemd vs Supervisord 对比

| 特性 | Systemd | Supervisord |
|------|---------|-------------|
| 容器兼容性 | 需要特权模式，配置复杂 | 完美适配容器环境 |
| 资源占用 | 较高（完整 init 系统） | 轻量级 |
| 日志管理 | journald，功能强大 | 简单文件日志 |
| 依赖管理 | 原生支持 After/Wants | 需要手动管理 |
| 用户权限 | 支持 User/Group | 支持 user 配置 |
| 开机自启 | systemctl enable | autostart=true |
| **推荐场景** | 宿主机、VM、专用容器 | 通用 Docker 容器 |

### 8.8 最终推荐方案总结

**推荐优先级**：

1. **脚本启动（首选）** - 第五章方案一或方案二
   ```bash
   # 最简单的方式
   /usr/local/bin/start-web-terminal.sh --daemon
   ```
   - 无需额外依赖
   - 配置简单
   - 完全符合容器设计原则

2. **Supervisord（次选）** - 第五章方案三
   ```bash
   # 需要进程守护时使用
   apt-get install -y supervisor
   ```
   - 自动重启崩溃的服务
   - 统一的日志管理

3. **Systemd（不推荐）** - 仅在特殊场景使用
   - 容器必须以特权模式运行
   - 需要完整 init 系统
   - 配置复杂，维护成本高

**对于 UID/GID 模式的容器**：
- 脚本方式最灵活，可以在 `entry_point.sh` 中正确设置用户权限后再启动服务
- Supervisord 也支持通过 `user=admin` 配置以指定用户运行
- Systemd 与动态 UID/GID 设置存在冲突，需要额外的 override 配置

**Supervisord 完整配置示例**：

```bash
# 安装
apt-get install -y supervisor

# Nginx 服务配置
cat > /etc/supervisor/conf.d/nginx.conf << 'EOF'
[program:nginx]
command=/usr/sbin/nginx -g "daemon off;"
autostart=true
autorestart=true
user=admin
stdout_logfile=/var/log/nginx/supervisor.log
stderr_logfile=/var/log/nginx/supervisor_err.log
priority=10
EOF

# GoTTY 服务配置
cat > /etc/supervisor/conf.d/gotty.conf << 'EOF'
[program:gotty]
command=/usr/local/bin/gotty --config /etc/gotty/config /bin/bash
autostart=true
autorestart=true
user=admin
stdout_logfile=/var/log/gotty/supervisor.log
stderr_logfile=/var/log/gotty/supervisor_err.log
priority=20
EOF

# 修改 entry_point.sh
cat > /entry_point.sh << 'EOF'
#!/bin/bash
groupmod -g $GID jtop
usermod -u $UID admin
chown -R admin:admin /var/log/gotty /var/log/nginx /run/nginx /var/lib/nginx
exec /usr/bin/supervisord -n -c /etc/supervisor/supervisord.conf
EOF
```

---

## 九、常见问题排查

### 问题1：Nginx 启动失败 - "user" 指令警告

**症状**：
```
nginx: [warn] the "user" directive makes sense only if the master process runs with super-user privileges, ignored in /etc/nginx/nginx.conf:4
```

**原因**：当以非 root 用户运行 Nginx 时，`user` 指令无效且会产生警告

**解决方案**：从 `nginx.conf` 中删除 `user admin;` 行

```bash
# 检查配置文件
grep "^user" /etc/nginx/nginx.conf

# 如果存在，删除该行
sed -i '/^user admin;/d' /etc/nginx/nginx.conf

# 重新测试
nginx -t
```

### 问题2：Nginx 启动失败 - 端口权限问题

**症状**：
```
nginx: [emerg] bind() to 0.0.0.0:80 failed (13: Permission denied)
```

**原因**：非 root 用户无法绑定 1024 以下的端口

**解决方案**：

方案一：使用非特权端口（推荐）

```bash
# 修改 Nginx 配置使用 8080 端口
sed -i 's/listen 80;/listen 8080;/' /etc/nginx/sites-available/web-terminal

# 重启 Nginx
nginx -s reload
```

方案二：授予绑定特权端口的能力

```bash
# 在容器启动时添加能力
docker run --cap-add=NET_BIND_SERVICE ...
```

方案三：使用 setcap（需要 root 权限设置一次）

```bash
# 在 Dockerfile 或容器构建时执行
setcap 'cap_net_bind_service=+ep' /usr/sbin/nginx
```

### 问题3：文件上传后所有者不是 admin

**症状**：通过 WebDAV 上传的文件所有者显示为其他用户（如 root 或 www-data）

**原因**：Nginx 工作进程未以 admin 用户运行

**排查**：

```bash
# 检查 Nginx 工作进程用户
ps aux | grep nginx
# 正确输出应为:
# root      ... nginx: master process ...
# admin     ... nginx: worker process ...

# 如果 worker 进程不是 admin，检查配置
```

**解决**：

```bash
# 确保没有 user 指令（非 root 运行时由当前用户运行）
grep "user" /etc/nginx/nginx.conf

# 确保通过 start-web-terminal.sh 启动（以 admin 用户执行）
whoami  # 应该显示 admin
```

### 问题4：文件服务 403 Forbidden

**症状**：访问 `/files/` 返回 403 错误

**排查**：

```bash
# 检查目录权限
ls -la /home/admin/
ls -la /home/admin/map_dir/

# 检查 Nginx 错误日志
tail -f /var/log/nginx/error.log
```

**解决**：

```bash
# 确保目录存在且有正确权限
mkdir -p /home/admin/map_dir
chown -R admin:admin /home/admin/map_dir
chmod 755 /home/admin/map_dir
```

### 问题2：目录权限问题

**症状**：
```
nginx: [emerg] mkdir() "/var/lib/nginx/body" failed (13: Permission denied)
```

**解决**：

```bash
# 检查目录权限
ls -la /var/lib/nginx/
ls -la /var/log/nginx/
ls -la /run/nginx/

# 修复权限
sudo chown -R admin:admin /var/lib/nginx
sudo chown -R admin:admin /var/log/nginx
sudo chown -R admin:admin /run/nginx
```

### 问题3：WebSocket 连接失败

**症状**：终端页面空白或连接错误，浏览器控制台显示：
```
Firefox 无法建立到 ws://127.0.0.1:8080/terminal/ws 服务器的连接
```

**原因分析**：

当从容器外用浏览器访问时，GoTTY 返回的页面中 WebSocket URL 可能包含 `127.0.0.1`，但：
- 容器外的浏览器访问 `127.0.0.1` 指向用户本机，而非容器
- GoTTY 监听 `127.0.0.1` 时，只接受容器内本地连接

**排查**：

```bash
# 检查 GoTTY 是否监听
ss -tlnp | grep 9090

# 检查 Nginx 配置
grep -A10 "location /terminal/" /etc/nginx/sites-enabled/*

# 查看浏览器控制台错误 (F12 → 控制台)
# 查看网络请求 (F12 → 网络 → WS)
```

**解决方案**：

**方案一：修改 GoTTY 监听地址（推荐）**

修改 `/etc/gotty/config`：

```bash
# 将 address 从 "127.0.0.1" 改为 "0.0.0.0"
sed -i 's/address = "127.0.0.1"/address = "0.0.0.0"/' /etc/gotty/config

# 重启 GoTTY
pkill gotty
nohup /usr/local/bin/start-gotty.sh > /var/log/gotty/gotty.log 2>&1 &
```

**方案二：覆盖 Origin 头（解决 "origin not allowed" 错误）**

错误日志显示 `websocket: origin not allowed` 是因为 GoTTY 验证 WebSocket 握手的 Origin 头。当浏览器从 `http://<external-ip>:8080/terminal/` 访问时，Origin 是 `http://<external-ip>:8080`，但 GoTTY 期望的是 `http://127.0.0.1:9090`。

**解决方法**：在 Nginx 配置中覆盖 Origin 头：

```nginx
location /terminal/ {
    proxy_pass http://gotty/;
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
    proxy_set_header Host $http_host;
    proxy_set_header X-Real-IP $remote_addr;
    proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    proxy_set_header X-Forwarded-Proto $scheme;
    
    # 关键：覆盖 Origin 头以绕过 GoTTY 的 Origin 检查
    proxy_set_header Origin "http://127.0.0.1:9090";
    
    proxy_read_timeout 86400;
}
```

**快速修复命令**：

```bash
# 在容器内执行，添加 Origin 头覆盖
sed -i '/proxy_set_header X-Forwarded-Proto/a\        \# 关键：覆盖 Origin 头以绕过 GoTTY 的 Origin 检查\n        proxy_set_header Origin "http://127.0.0.1:9090";' /etc/nginx/sites-available/web-terminal

# 重启 Nginx
nginx -s reload
```

**方案三：使用完整配置脚本**

创建更新脚本 `update-gotty-external.sh`：

```bash
#!/bin/bash
# 更新 GoTTY 配置以支持外部访问

# 1. 修改 GoTTY 配置
cat > /etc/gotty/config << 'EOF'
# 监听所有接口 (允许 Nginx 代理外部请求)
address = "0.0.0.0"

# 监听端口
port = 9090

# 允许远程写入
permit_write = true

# 允许重新连接
reconnect = true
reconnect_timeout = 10

# 终端类型
term = "xterm-256color"

# 窗口标题
title_format = "AMR Terminal"

# 首选 WebSocket
prefer_websocket = true

# 滚动回溯
scrollback = 10000
EOF

# 2. 更新 Nginx 站点配置
cat > /etc/nginx/sites-available/web-terminal << 'EOF'
upstream gotty {
    server 127.0.0.1:9090;
}

server {
    listen 8080;
    server_name localhost;
    root /var/www/html;
    index index.html;

    location / {
        try_files $uri $uri/ =404;
    }

    location /terminal/ {
        proxy_pass http://gotty/;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $http_host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        # 关键：覆盖 Origin 头以绕过 GoTTY 的 Origin 检查
        proxy_set_header Origin "http://127.0.0.1:9090";
        proxy_read_timeout 86400;
        proxy_send_timeout 86400;
        proxy_buffering off;
    }

    location /terminal/js/ {
        proxy_pass http://gotty/js/;
    }

    location /terminal/css/ {
        proxy_pass http://gotty/css/;
    }

    location /files/ {
        alias /home/admin/map_dir/;
        autoindex on;
        autoindex_exact_size off;
        autoindex_localtime on;
        dav_methods PUT DELETE MKCOL COPY MOVE;
        dav_access user:rw group:rw all:r;
        client_max_body_size 0;
        create_full_put_path on;
    }

    location /health {
        access_log off;
        return 200 "OK\n";
        add_header Content-Type text/plain;
    }
}
EOF

# 3. 重启服务
echo "Restarting services..."
nginx -s reload
pkill gotty
sleep 1
nohup /usr/local/bin/start-gotty.sh > /var/log/gotty/gotty.log 2>&1 &

# 4. 验证
sleep 2
echo ""
echo "=== Service Status ==="
ss -tlnp | grep -E "(8080|9090)"
echo ""
echo "=== GoTTY Log (last 5 lines) ==="
tail -5 /var/log/gotty/gotty.log
echo ""
echo "Configuration updated. Try accessing from external browser:"
echo "  http://<container-ip>:8080/terminal/"
EOF

chmod +x /usr/local/bin/update-gotty-external.sh
```

**验证步骤**：

```bash
# 1. 检查 GoTTY 监听地址 (应该是 0.0.0.0:9090)
ss -tlnp | grep 9090
# 期望输出: LISTEN 0 128 0.0.0.0:9090 0.0.0.0:*

# 2. 测试 WebSocket 连接 (在容器内)
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" \
    -H "Sec-WebSocket-Key: test" -H "Sec-WebSocket-Version: 13" \
    http://127.0.0.1:9090/ws
# 期望: HTTP/1.1 101 Switching Protocols

# 3. 从容器外浏览器访问
# 打开 http://<your-container-ip>:8080/terminal/
# 按 F12 查看网络 → WS 标签，确认 WebSocket 连接成功
```

### 问题4：服务在容器重启后未启动

**解决**：确保服务启动命令在 entry_point.sh 或 CMD 中正确配置

```bash
# 检查容器启动命令
docker inspect web-terminal | grep -A5 "Cmd"

# 手动启动服务
docker exec -it web-terminal /usr/local/bin/start-web-terminal.sh
```

---

## 十、完整安装脚本

将以下脚本保存为 `/root/install-web-terminal.sh`（以 root 执行）：

```bash
#!/bin/bash
set -e

echo "=========================================="
echo "  Web Terminal 一键安装脚本 (UID/GID 模式)"
echo "=========================================="

# 1. 安装依赖
echo "[1/8] 安装依赖..."
apt-get update
apt-get install -y nginx wget sudo

# 2. 安装 GoTTY
echo "[2/8] 安装 GoTTY..."
GOTTY_VERSION="1.0.1"
wget -q https://github.com/yudai/gotty/releases/download/v${GOTTY_VERSION}/gotty_linux_amd64.tar.gz
tar -xzf gotty_linux_amd64.tar.gz
mv gotty /usr/local/bin/
chmod +x /usr/local/bin/gotty
rm gotty_linux_amd64.tar.gz

# 3. 创建目录并设置权限
echo "[3/8] 创建目录..."
mkdir -p /etc/gotty /var/log/gotty
mkdir -p /var/www/html
mkdir -p /var/lib/nginx/{body,proxy,fastcgi,uwsgi,scgi}
mkdir -p /run/nginx
mkdir -p /home/admin/map_dir

# 4. 配置 GoTTY
echo "[4/8] 配置 GoTTY..."
cat > /etc/gotty/config << 'EOF'
# 监听所有接口 (允许外部浏览器通过 Nginx 代理访问)
address = "0.0.0.0"
port = 9090
permit_write = true
reconnect = true
reconnect_timeout = 10
term = "xterm-256color"
title_format = "AMR Terminal"
once = false
close_signal = 1
prefer_websocket = true
scrollback = 10000
EOF

cat > /usr/local/bin/start-gotty.sh << 'EOF'
#!/bin/bash
exec gotty --config /etc/gotty/config /bin/bash
EOF
chmod +x /usr/local/bin/start-gotty.sh

# 5. 配置 Nginx (以 admin 用户运行，无 user 指令)
echo "[5/8] 配置 Nginx..."
cat > /etc/nginx/nginx.conf << 'EOF'
# 注意：非 root 用户运行时不能使用 "user" 指令
worker_processes auto;
pid /run/nginx/nginx.pid;
error_log /var/log/nginx/error.log notice;

events { worker_connections 1024; }

http {
    include /etc/nginx/mime.types;
    default_type application/octet-stream;
    log_format main '$remote_addr - $remote_user [$time_local] "$request" $status';
    access_log /var/log/nginx/access.log main;
    sendfile on;
    keepalive_timeout 65;
    
    client_body_temp_path /var/lib/nginx/body;
    proxy_temp_path /var/lib/nginx/proxy;
    fastcgi_temp_path /var/lib/nginx/fastcgi;
    uwsgi_temp_path /var/lib/nginx/uwsgi;
    scgi_temp_path /var/lib/nginx/scgi;
    
    include /etc/nginx/sites-enabled/*;
}
EOF

# 6. 创建首页
echo "[6/8] 创建首页..."
cat > /var/www/html/index.html << 'EOF'
<!DOCTYPE html>
<html><head><meta charset="UTF-8"><title>Web Terminal</title>
<style>
body{background:linear-gradient(135deg,#1a1a2e,#16213e);min-height:100vh;display:flex;justify-content:center;align-items:center;color:#fff;font-family:sans-serif;}
.c{text-align:center;padding:40px;}
h1{font-size:2.5rem;background:linear-gradient(90deg,#00d2ff,#3a7bd5);-webkit-background-clip:text;-webkit-text-fill-color:transparent;}
p{color:#a0a0a0;margin:20px 0;}
a{display:inline-block;padding:15px 40px;background:linear-gradient(90deg,#00d2ff,#3a7bd5);color:#fff;text-decoration:none;border-radius:30px;font-weight:bold;margin:10px;}
</style></head>
<body><div class="c">
<h1>🚀 Web Terminal</h1>
<p>AMR 建图系统管理界面</p>
<a href="/terminal/">打开终端</a>
<a href="/files/">文件服务</a>
</div></body></html>
EOF

# 7. 创建站点配置
echo "[7/8] 创建站点配置..."
cat > /etc/nginx/sites-available/web-terminal << 'EOF'
upstream gotty { server 127.0.0.1:9090; }
server {
    listen 8080;
    root /var/www/html;
    index index.html;
    
    location / { try_files $uri $uri/ =404; }
    
    location /terminal/ {
        proxy_pass http://gotty/;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        # 关键：覆盖 Origin 头以绕过 GoTTY 的 Origin 检查
        proxy_set_header Origin "http://127.0.0.1:9090";
        proxy_read_timeout 86400;
    }
    
    location /terminal/js/ { proxy_pass http://gotty/js/; }
    location /terminal/css/ { proxy_pass http://gotty/css/; }
    
    location /files/ {
        alias /home/admin/map_dir/;
        autoindex on;
        autoindex_exact_size off;
        autoindex_localtime on;
        dav_methods PUT DELETE MKCOL COPY MOVE;
        dav_access user:rw group:rw all:r;
        client_max_body_size 0;
        create_full_put_path on;
    }
    
    location /health {
        access_log off;
        return 200 "OK\n";
        add_header Content-Type text/plain;
    }
}
EOF

rm -f /etc/nginx/sites-enabled/default
ln -sf /etc/nginx/sites-available/web-terminal /etc/nginx/sites-enabled/

# 8. 设置权限
echo "[8/8] 设置权限..."
chown -R admin:admin /var/log/gotty
chown -R admin:admin /var/www/html
chown -R admin:admin /var/lib/nginx
chown -R admin:admin /var/log/nginx
chown -R admin:admin /run/nginx
chown -R admin:admin /home/admin/map_dir

# 创建启动脚本
cat > /usr/local/bin/start-web-terminal.sh << 'EOF'
#!/bin/bash
echo "Starting Web Terminal..."
nginx
nohup /usr/local/bin/start-gotty.sh > /var/log/gotty/gotty.log 2>&1 &
sleep 1
echo "✓ Web Terminal started"
echo "  Homepage:  http://localhost:8080/"
echo "  Terminal:  http://localhost:8080/terminal/"
echo "  Files:     http://localhost:8080/files/"
[ "$1" = "--daemon" ] && tail -f /dev/null
EOF
chmod +x /usr/local/bin/start-web-terminal.sh

# 测试 Nginx 配置
nginx -t

echo ""
echo "=========================================="
echo "  安装完成!"
echo "=========================================="
echo ""
echo "启动服务: /usr/local/bin/start-web-terminal.sh"
echo "后台运行: /usr/local/bin/start-web-terminal.sh --daemon"
echo ""
echo "Docker 运行示例:"
echo "  docker run -d -e UID=1000 -e GID=1000 -p 8080:8080 \\"
echo "    -v /path/to/maps:/home/admin/map_dir your-image \\"
echo "    /usr/local/bin/start-web-terminal.sh --daemon"
```

运行安装：

```bash
chmod +x /root/install-web-terminal.sh
/root/install-web-terminal.sh
```

---

## 十一、服务管理命令速查

```bash
# ========== 启动服务 ==========
# 前台启动 (调试)
/usr/local/bin/start-web-terminal.sh

# 后台启动
/usr/local/bin/start-web-terminal.sh --daemon

# ========== 停止服务 ==========
# 停止 Nginx
nginx -s stop

# 停止 GoTTY
pkill gotty

# 停止所有
pkill gotty; nginx -s stop

# ========== 重启服务 ==========
# 重启 Nginx
nginx -s reload

# 重启 GoTTY
pkill gotty; nohup /usr/local/bin/start-gotty.sh > /var/log/gotty/gotty.log 2>&1 &

# ========== 查看状态 ==========
pgrep -a nginx
pgrep -a gotty
ss -tlnp | grep -E "(8080|9090)"

# ========== 查看日志 ==========
tail -f /var/log/nginx/access.log
tail -f /var/log/nginx/error.log
tail -f /var/log/gotty/gotty.log

# ========== 文件服务操作 ==========
# 上传文件 (WebDAV)
curl -T local_file.txt http://localhost:8080/files/remote_file.txt

# 下载文件
curl -O http://localhost:8080/files/remote_file.txt

# 删除文件
curl -X DELETE http://localhost:8080/files/remote_file.txt

# 创建目录
curl -X MKCOL http://localhost:8080/files/new_directory/

# 列出目录内容
curl -s http://localhost:8080/files/

# 检查文件所有者
ls -la /home/admin/map_dir/
```

---

## 十二、参考资料

- [GoTTY GitHub](https://github.com/yudai/gotty)
- [Nginx WebSocket Proxying](https://www.nginx.com/blog/websocket-nginx-proxy-guide/)
- [Nginx WebDAV Module](https://nginx.org/en/docs/http/ngx_http_dav_module.html)
- [Nginx Documentation](https://nginx.org/en/docs/)
- [Linux Capabilities](https://man7.org/linux/man-pages/man7/capabilities.7.html)

---

## 十三、快速参考卡片

### 访问地址

| 服务 | 地址 | 说明 |
|------|------|------|
| 首页 | `http://your-ip:8080/` | 导航页面 |
| 终端 | `http://your-ip:8080/terminal/` | Web 终端 |
| 文件 | `http://your-ip:8080/files/` | 文件浏览/上传 |
| 健康检查 | `http://your-ip:8080/health` | 服务状态 |

### 端口说明

| 服务 | 容器内端口 | 说明 |
|------|-----------|------|
| Nginx | 8080 | 主入口，非特权端口 |
| GoTTY | 9090 | 仅本地监听 |

### 关键配置要点

1. **删除 `user` 指令**：非 root 运行时不能使用
2. **使用端口 8080+**：非 root 无法绑定 < 1024 端口
3. **GoTTY 端口 9090**：避免与 Nginx 8080 冲突
4. **文件所有者**：Nginx 以 admin 运行，新文件自动属于 admin
