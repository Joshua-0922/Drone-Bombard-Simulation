#!/usr/bin/env python3
"""
GPU 환경 검증 스크립트
PyTorch, CUDA, GPU 상태를 확인합니다.
"""
import sys

def verify_environment():
    print("=" * 60)
    print("GPU 환경 검증")
    print("=" * 60)

    # 1. PyTorch 검증
    try:
        import torch
        print(f"✓ PyTorch 버전: {torch.__version__}")
        print(f"✓ CUDA 빌드 버전: {torch.version.cuda}")
    except ImportError as e:
        print(f"✗ PyTorch 임포트 실패: {e}")
        print("  pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121")
        return False

    # 2. CUDA 사용 가능 여부
    if not torch.cuda.is_available():
        print("✗ CUDA를 사용할 수 없습니다")
        print("  nvidia-smi로 GPU 상태를 확인하세요")
        return False

    print(f"✓ CUDA 사용 가능")
    print(f"✓ GPU 개수: {torch.cuda.device_count()}")
    print(f"✓ 현재 GPU: {torch.cuda.get_device_name(0)}")

    # Compute Capability 출력
    capability = torch.cuda.get_device_capability(0)
    print(f"✓ Compute Capability: {capability[0]}.{capability[1]}")

    # 3. 메모리 테스트
    try:
        # 간단한 행렬 곱셈 테스트
        x = torch.rand(1000, 1000).cuda()
        y = torch.rand(1000, 1000).cuda()
        z = x @ y
        torch.cuda.synchronize()

        print(f"✓ GPU 연산 테스트 성공")
        print(f"✓ 할당된 메모리: {torch.cuda.memory_allocated(0) / 1024**2:.2f} MB")
        print(f"✓ 예약된 메모리: {torch.cuda.memory_reserved(0) / 1024**2:.2f} MB")

        # 메모리 정리
        del x, y, z
        torch.cuda.empty_cache()

    except Exception as e:
        print(f"✗ GPU 연산 실패: {e}")
        return False

    # 4. Ultralytics 검증
    try:
        from ultralytics import YOLO
        import ultralytics
        print(f"✓ Ultralytics 버전: {ultralytics.__version__}")
    except ImportError as e:
        print(f"✗ Ultralytics 임포트 실패: {e}")
        print("  pip3 install ultralytics")
        return False

    # 5. Roboflow 검증
    try:
        import roboflow
        print(f"✓ Roboflow 설치 확인")
    except ImportError as e:
        print(f"✗ Roboflow 임포트 실패: {e}")
        print("  pip3 install roboflow")
        return False

    print("=" * 60)
    print("모든 검증 통과!")
    print("=" * 60)
    return True

if __name__ == "__main__":
    success = verify_environment()
    sys.exit(0 if success else 1)
