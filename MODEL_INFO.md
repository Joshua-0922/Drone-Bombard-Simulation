# Drone Bombard Trained Model

## Model Information
- **Model File**: `drone_bombard_best.pt`
- **Model Size**: 6.0 MB
- **Training Date**: 2026-01-03
- **Total Epochs**: 100
- **Best Epoch**: 96

## Performance Metrics

### Best Epoch Performance (Epoch 96)
- **mAP@0.5**: 0.995 (99.5%)
- **mAP@0.5:0.95**: 0.9937 (99.37%)
- **Precision**: 0.9996 (99.96%)
- **Recall**: 1.0000 (100%)

### Final Epoch Performance (Epoch 100)
- **mAP@0.5**: 0.995 (99.5%)
- **mAP@0.5:0.95**: 0.9911 (99.11%)
- **Precision**: 0.9996 (99.96%)
- **Recall**: 1.0000 (100%)

### Loss Reduction
- **Box Loss**: 0.890 → 0.235 (-73.6%)
- **Class Loss**: 1.673 → 0.133 (-92.1%)
- **DFL Loss**: 1.169 → 0.809 (-30.8%)

### Validation Metrics
- **Val Box Loss**: 0.152
- **Val Class Loss**: 0.097
- **Val DFL Loss**: 0.606

### Training Stability (Last 10 Epochs)
- **Average mAP@0.5:0.95**: 0.9914
- **Standard Deviation**: 0.0011
- **Range**: 0.9901 ~ 0.9937

## Training Details
- **Framework**: YOLOv8/YOLO11
- **Training Time**: ~66 minutes
- **Full Training Results**: See `training_results.csv`

## Model Location in Training Directory
Original location: `/workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/`

## Usage
Load this model with:
```python
from ultralytics import YOLO

model = YOLO('drone_bombard_best.pt')
results = model.predict(source='your_image.jpg')
```

## Conclusion
This model demonstrates excellent performance for drone bombard target detection with near-perfect accuracy and minimal false positives. The training converged stably without signs of overfitting, making it suitable for deployment in production environments.
