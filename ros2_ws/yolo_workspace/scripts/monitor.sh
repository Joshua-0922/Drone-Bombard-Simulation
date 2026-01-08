#!/bin/bash
# YOLO 학습 모니터링 스크립트
CSV="/workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/results.csv"

while true; do
    clear
    echo "=== YOLO Training Monitor ==="
    echo ""
    tail -10 "$CSV" | awk -F',' 'NR>1 {printf "Epoch %3s | mAP50: %5.1f%% | mAP50-95: %5.1f%% | Precision: %5.1f%%\n", $1, $8*100, $9*100, $6*100}'
    echo ""
    echo "(Ctrl+C to exit)"
    sleep 5
done
