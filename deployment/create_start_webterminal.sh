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
    echo "✓ GoTTY is running (port 9090)"
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