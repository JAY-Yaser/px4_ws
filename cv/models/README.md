# 模型文件

存放 YOLO 模型权重文件。

## 下载模型

```bash
conda activate px4
cd cv/models

# YOLOv8n (nano, 最快)
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.pt

# YOLOv8s (small)
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s.pt

# YOLOv8m (medium)
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8m.pt
```

或通过 Python 自动下载：

```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')  # 自动下载到当前目录
```
