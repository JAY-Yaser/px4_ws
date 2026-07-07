#!/usr/bin/env python3
"""
YOLOv8 basic test — runs inference on a sample image.

Usage:
    python3 cv/yolo_test.py                          # download & test on sample image
    python3 cv/yolo_test.py --source your_image.jpg  # test on custom image
    python3 cv/yolo_test.py --source 0               # test on webcam
    python3 cv/yolo_test.py --model yolov8n.pt       # use specific model
"""

import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='YOLOv8 test')
    parser.add_argument('--source', default='ultralytics/assets',
                        help='Image source (file path, camera index, or URL)')
    parser.add_argument('--model', default='yolov8n.pt',
                        help='Model weights (yolov8n.pt, yolov8s.pt, etc.)')
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed.")
        print("  Run: pip install ultralytics")
        sys.exit(1)

    print(f"Loading model: {args.model}")
    model = YOLO(args.model)

    print(f"Running inference on: {args.source}")
    results = model.predict(source=args.source, show=True, save=True)

    # Print detection summary
    for r in results:
        boxes = r.boxes
        if boxes is not None and len(boxes) > 0:
            print(f"\nDetected {len(boxes)} objects:")
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = model.names[cls_id]
                print(f"  {name}: {conf:.2f}")
        else:
            print("\nNo objects detected.")

    print(f"\nResults saved to: {model.predictor.save_dir}")


if __name__ == '__main__':
    main()
