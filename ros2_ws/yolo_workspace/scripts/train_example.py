#!/usr/bin/env python3
"""
YOLOv8 학습 스크립트
Roboflow에서 다운로드한 데이터셋으로 YOLOv8 모델을 학습합니다.
"""
import os
import sys
from pathlib import Path

def train_yolo():
    print("=" * 60)
    print("YOLOv8 학습 시작")
    print("=" * 60)

    try:
        from ultralytics import YOLO
        import torch
    except ImportError as e:
        print(f"✗ 필요한 패키지를 임포트할 수 없습니다: {e}")
        sys.exit(1)

    # GPU 확인
    if not torch.cuda.is_available():
        print("⚠ GPU를 사용할 수 없습니다. CPU로 학습합니다.")
        device = 'cpu'
    else:
        print(f"✓ GPU 사용: {torch.cuda.get_device_name(0)}")
        device = 0

    # 경로 설정
    workspace = Path("/workspace/ros2_ws/yolo_workspace")
    datasets_dir = workspace / "datasets"

    # Roboflow 데이터셋 찾기 (Drone-Bombard-Simulation-1)
    dataset_name = "Drone-Bombard-Simulation-1"
    data_yaml = datasets_dir / dataset_name / "data.yaml"

    if not data_yaml.exists():
        print(f"✗ 데이터셋을 찾을 수 없습니다: {data_yaml}")
        print()
        print("먼저 데이터셋을 다운로드하세요:")
        print("  python3 download_dataset.py")
        sys.exit(1)

    print(f"✓ 데이터셋 경로: {data_yaml}")

    # 모델 경로
    model_path = workspace / "models" / "pretrained" / "yolov8n.pt"

    if not model_path.exists():
        print(f"⚠ 사전 학습 모델을 찾을 수 없습니다: {model_path}")
        print("  자동으로 다운로드합니다...")
        model = YOLO('yolov8n.pt')
    else:
        print(f"✓ 모델 경로: {model_path}")
        model = YOLO(str(model_path))

    # 학습 설정
    print()
    print("학습 설정:")
    print(f"  - 모델: YOLOv8n")
    print(f"  - 데이터셋: {dataset_name}")
    print(f"  - Epochs: 100")
    print(f"  - 이미지 크기: 640x640")
    print(f"  - 배치 크기: 16")
    print(f"  - 디바이스: {device}")
    print(f"  - Workers: 4")
    print()

    # 학습 시작
    print("=" * 60)
    print("학습 시작... (시간이 오래 걸릴 수 있습니다)")
    print("=" * 60)
    print()

    try:
        results = model.train(
            data=str(data_yaml),
            epochs=100,
            imgsz=640,
            batch=16,
            name="drone_bombard_train",
            project=str(workspace / "runs" / "train"),
            device=device,
            workers=0,  # 공유 메모리 부족 방지 (Docker 환경)
            patience=50,
            save=True,
            save_period=10,
            cache=False,
            plots=True,
            verbose=True,
            # 혼합 정밀도 학습 (L4 GPU에 최적화)
            amp=True,
        )

        print()
        print("=" * 60)
        print("학습 완료!")
        print("=" * 60)
        print(f"✓ 결과 디렉토리: {workspace}/runs/train/drone_bombard_train")
        print(f"✓ 최종 모델: {workspace}/runs/train/drone_bombard_train/weights/best.pt")
        print()
        print("추론 테스트:")
        print("  yolo predict model=runs/train/drone_bombard_train/weights/best.pt source=<이미지경로>")
        print("=" * 60)

    except KeyboardInterrupt:
        print()
        print("=" * 60)
        print("학습이 중단되었습니다")
        print("=" * 60)
        print()
        print("학습을 재개하려면:")
        print("  yolo train resume model=runs/train/drone_bombard_train/weights/last.pt")
        sys.exit(1)

    except Exception as e:
        print()
        print("=" * 60)
        print(f"✗ 학습 실패: {e}")
        print("=" * 60)
        print()
        print("문제 해결:")
        print("  1. GPU 메모리 부족 시 배치 크기 감소: batch=8")
        print("  2. data.yaml 파일 확인")
        print("  3. 데이터셋 이미지 확인")
        sys.exit(1)

if __name__ == "__main__":
    train_yolo()
