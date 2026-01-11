#!/usr/bin/env python3
"""
Roboflow 데이터셋 다운로드 스크립트
drone-bombard-simulation 프로젝트의 데이터셋을 YOLOv8 형식으로 다운로드합니다.
"""
import os
import sys

def download_dataset():
    print("=" * 60)
    print("Roboflow 데이터셋 다운로드")
    print("=" * 60)

    try:
        from roboflow import Roboflow
    except ImportError:
        print("✗ Roboflow가 설치되지 않았습니다")
        print("  pip3 install roboflow")
        sys.exit(1)

    # Roboflow 설정
    API_KEY = "WiZh74WrGpQMsfSoWCce"
    WORKSPACE = "test-lchnm"
    PROJECT = "drone-bombard-simulation"
    VERSION = 1

    # 작업 디렉토리
    workspace_dir = "/workspace/ros2_ws/yolo_workspace/datasets"
    os.chdir(workspace_dir)

    print(f"✓ API 키: {API_KEY[:10]}...")
    print(f"✓ 프로젝트: {WORKSPACE}/{PROJECT}")
    print(f"✓ 버전: {VERSION}")
    print(f"✓ 저장 위치: {workspace_dir}")
    print()

    try:
        # Roboflow 초기화
        print("Roboflow 초기화 중...")
        rf = Roboflow(api_key=API_KEY)

        # 프로젝트 및 버전 로드
        print(f"프로젝트 로드 중: {WORKSPACE}/{PROJECT}...")
        project = rf.workspace(WORKSPACE).project(PROJECT)
        version = project.version(VERSION)

        # 데이터셋 다운로드 (YOLOv8 형식)
        print(f"데이터셋 다운로드 중 (YOLOv8 형식)...")
        dataset = version.download("yolov8")

        print()
        print("=" * 60)
        print("다운로드 완료!")
        print("=" * 60)
        print(f"✓ 데이터셋 위치: {dataset.location}")
        print(f"✓ data.yaml 경로: {dataset.location}/data.yaml")
        print()
        print("다음 단계:")
        print("  학습 시작: python3 ../scripts/train_example.py")
        print("=" * 60)

        return dataset.location

    except Exception as e:
        print()
        print("=" * 60)
        print(f"✗ 다운로드 실패: {e}")
        print("=" * 60)
        print()
        print("문제 해결:")
        print("  1. API 키가 유효한지 확인")
        print("  2. 인터넷 연결 확인")
        print("  3. Roboflow 프로젝트 접근 권한 확인")
        sys.exit(1)

if __name__ == "__main__":
    download_dataset()
