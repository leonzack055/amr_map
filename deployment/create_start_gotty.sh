cat > /usr/local/bin/start-gotty.sh << 'EOF'
#!/bin/bash
# GoTTY 启动脚本

CONFIG_FILE="/etc/gotty/config"
SHELL="/bin/bash"

echo "Starting GoTTY on 127.0.0.1:9090..."
exec gotty --config "$CONFIG_FILE" "$SHELL"
EOF

chmod +x /usr/local/bin/start-gotty.sh