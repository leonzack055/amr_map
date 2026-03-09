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