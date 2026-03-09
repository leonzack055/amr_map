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