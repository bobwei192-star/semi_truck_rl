#!/bin/bash
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$PROJECT_DIR/.venv"

echo "========================================="
echo "  半挂车倒车入库 RL 环境安装"
echo "  项目目录: $PROJECT_DIR"
echo "========================================="

# ================================
# 多源测速函数
# ================================
test_pip_source() {
    local url=$1
    local timeout=5
    local start_time=$(date +%s%3N)
    
    if curl -s -o /dev/null -m $timeout "$url/simple/pip/" > /dev/null 2>&1; then
        local end_time=$(date +%s%3N)
        local latency=$((end_time - start_time))
        echo "$latency $url"
    else
        echo "99999 $url"
    fi
}

find_fastest_source() {
    echo "[0/4] 正在测试各PyPI源速度..." >&2
    
    # 定义多个PyPI源（包含国内源）
    local sources=(
        "https://pypi.org/simple"
        "https://mirrors.aliyun.com/pypi/simple"
        "https://pypi.tuna.tsinghua.edu.cn/simple"
        "https://pypi.douban.com/simple"
        "https://mirror.baidu.com/pypi/simple"
        "https://mirrors.cloud.tencent.com/pypi/simple"
    )
    
    # 并行测试所有源并获取最快的
    local fastest=""
    local min_latency=99999
    
    for src in "${sources[@]}"; do
        local result=$(test_pip_source "$src")
        local latency=$(echo "$result" | awk '{print $1}')
        local url=$(echo "$result" | awk '{print $2}')
        
        if [ "$latency" -lt "$min_latency" ]; then
            min_latency="$latency"
            fastest="$url"
        fi
        
        if [ "$latency" -lt 99999 ]; then
            echo "  $url: ${latency}ms" >&2
        else
            echo "  $url: 超时" >&2
        fi
    done
    
    echo "  最快源: $fastest (响应时间: ${min_latency}ms)" >&2
    echo "$fastest"
}

# ================================
# 检测ROCm GPU
# ================================
detect_rocm() {
    if command -v rocminfo &>/dev/null; then
        if rocminfo 2>/dev/null | grep -q -i "GPU"; then
            echo "true"
            return
        fi
    fi
    echo "false"
}

# ================================
# 主安装流程
# ================================

# 查找最快的PyPI源
FASTEST_PIP_SOURCE=$(find_fastest_source)
echo ""

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
pip install --upgrade pip -q -i "$FASTEST_PIP_SOURCE"

echo "[4/4] 安装依赖..."
echo "  使用源: $FASTEST_PIP_SOURCE"

# 先安装基础依赖
pip install -r "$PROJECT_DIR/requrements.txt" -q -i "$FASTEST_PIP_SOURCE"

# 检测是否有ROCm GPU并安装相应的PyTorch
echo ""
echo "========================================="
echo "  检测GPU环境..."
echo "========================================="

HAS_ROCM=$(detect_rocm)
if [ "$HAS_ROCM" = "true" ]; then
    echo "  检测到ROCm GPU，安装ROCm版本PyTorch..."
    # 使用 uv 安装以获得更好的性能（参考 AIMax395 项目）
    if ! command -v uv &>/dev/null; then
        echo "  安装 uv..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.local/bin:$PATH"
    fi
    uv pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/rocm7.0
    echo "  ROCm PyTorch安装完成"
else
    echo "  未检测到ROCm GPU，使用已安装的PyTorch"
fi

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