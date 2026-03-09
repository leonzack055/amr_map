cat > /etc/gotty/config << 'EOF'
# GoTTY 配置文件

# 监听地址 (本地，由 Nginx 代理)
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