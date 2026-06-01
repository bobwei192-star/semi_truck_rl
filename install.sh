#!/bin/bash
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$PROJECT_DIR/venv"

echo "========================================="
echo "  半挂车倒车入库 RL 环境安装"
echo "  项目目录: $PROJECT_DIR"
echo "========================================="

if [ -d "$VENV_DIR" ]; then
    echo "[1/4] 虚拟环境已存在: $VENV_DIR"
else
    echo "[1/4] 创建 Python 虚拟环境..."
    python3 -m venv "$VENV_DIR"
    echo "      虚拟环境创建完成: $VENV_DIR"
fi

echo "[2/4] 激活虚拟环境..."
source "$VENV_DIR/bin/activate"

echo "[3/4] 升级 pip..."
pip install --upgrade pip -q

echo "[4/4] 安装依赖..."
pip install -r "$PROJECT_DIR/requrements.txt"

echo ""
echo "========================================="
echo "  安装完成!"
echo "========================================="
echo ""
echo "使用方法:"
echo "  source $VENV_DIR/bin/activate"
echo "  cd $PROJECT_DIR"
echo ""
echo "  # 训练"
echo "  python3 rl/train.py"
echo ""
echo "  # 查看训练曲线"
echo "  tensorboard --logdir rl_ckpt/tensorboard"
echo ""
echo "  # 推理（需要有训练好的模型）"
echo "  python3 rl/inference.py"
echo ""
echo "  # 手动控制测试"
echo "  python3 manual_test.py"
echo ""
