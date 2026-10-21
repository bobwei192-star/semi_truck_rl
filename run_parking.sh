#!/usr/bin/env bash
# 一键运行半挂车倒车入库三线可视化
# 用法: ./run_parking.sh          交互 pygame 窗口
#       ./run_parking.sh headless 离屏截图 another_solve/parking_three_lines.png
set -e
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
    echo "首次运行: 创建虚拟环境并安装依赖..."
    python3 -m venv .venv
    .venv/bin/pip install -q numpy pygame-ce
fi

.venv/bin/python3 another_solve/parking_three_lines.py "$@"
