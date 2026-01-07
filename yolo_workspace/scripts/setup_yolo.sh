#!/bin/bash
set -e

echo "=========================================="
echo "YOLO 환경 설정 시작"
echo "=========================================="

# 작업 디렉토리 설정
WORKSPACE_DIR="/workspace/ros2_ws/yolo_workspace"

# 1. 디렉토리 구조 확인
echo "[1/5] 디렉토리 구조 확인..."
if [ -d "$WORKSPACE_DIR" ]; then
    echo "✓ 작업 디렉토리 존재: $WORKSPACE_DIR"
else
    echo "✗ 작업 디렉토리가 없습니다"
    exit 1
fi

# 2. PyTorch 설치
echo "[2/5] PyTorch 설치 (CUDA 12.1 빌드)..."
echo "  설치 중... (수 분 소요될 수 있습니다)"
pip3 install --no-cache-dir torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu121

echo "✓ PyTorch 설치 완료"

# 3. Ultralytics YOLO 및 Roboflow 설치
echo "[3/5] Ultralytics YOLO 및 Roboflow 설치..."
pip3 install --no-cache-dir ultralytics roboflow

echo "✓ YOLO 및 Roboflow 설치 완료"

# 4. GPU 검증
echo "[4/5] GPU 환경 검증..."
python3 "$WORKSPACE_DIR/scripts/verify_gpu.py"

# 5. YOLOv8 기본 모델 다운로드
echo "[5/5] YOLOv8 기본 모델 다운로드..."
cd "$WORKSPACE_DIR/models/pretrained"
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); print('✓ YOLOv8n 모델 다운로드 완료')"

echo ""
echo "=========================================="
echo "설치 완료!"
echo "=========================================="
echo "작업 디렉토리: $WORKSPACE_DIR"
echo ""
echo "다음 단계:"
echo "  1. 데이터셋 다운로드: python3 scripts/download_dataset.py"
echo "  2. 학습 시작: python3 scripts/train_example.py"
echo "=========================================="
