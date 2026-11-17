#!/usr/bin/env bash
# 启动课程网站（本地静态服务器）
# 用法: ./serve.sh [端口]   默认 8080
PORT="${1:-8080}"
cd "$(dirname "$0")"
echo "==============================================="
echo "  SLAM·MPC 导航课堂"
echo "  http://localhost:${PORT}"
echo "  按 Ctrl+C 停止"
echo "==============================================="
python3 -m http.server "${PORT}" --bind 127.0.0.1
