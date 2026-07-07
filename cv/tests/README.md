# YOLO 测试

## yolo_test.py

YOLOv8 基础推理测试，使用 ultralytics 官方模型。

```bash
conda activate px4

# 官方示例图片测试（默认 yolov8n.pt）
python3 cv/tests/yolo_test.py

# 指定图片
python3 cv/tests/yolo_test.py --source path/to/image.jpg

# 摄像头实时检测
python3 cv/tests/yolo_test.py --source 0

# 使用不同模型
python3 cv/tests/yolo_test.py --model ../models/yolov8n.pt
python3 cv/tests/yolo_test.py --model yolov8s.pt
```

模型文件存放于 `cv/models/`，脚本默认从 ultralytics 自动下载。
