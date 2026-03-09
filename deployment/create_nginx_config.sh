cat > /etc/nginx/nginx.conf << 'EOF'
# Nginx 主配置 - 以 admin 用户运行
# 注意：当以非 root 用户运行时，不能使用 "user" 指令
# 如果添加 "user admin;" 会导致警告并被忽略

worker_processes auto;
pid /run/nginx/nginx.pid;
error_log /var/log/nginx/error.log notice;

user root;
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