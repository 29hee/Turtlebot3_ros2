#!/usr/bin/env python3
"""
train_yolo_digit.py
YOLOv8 분류(classify) 모드로 MNIST 손글씨 숫자 0~9 학습.
→ 결과 모델: ../models/yolo_digit_cls.pt

실행:  python3 scripts/train_yolo_digit.py
"""
import os
import shutil
from ultralytics import YOLO

HERE = os.path.dirname(os.path.realpath(__file__))
OUT_DIR = os.path.join(os.path.dirname(HERE), 'models')
OUT_PATH = os.path.join(OUT_DIR, 'yolo_digit_cls.pt')

os.makedirs(OUT_DIR, exist_ok=True)

model = YOLO('yolov8n-cls.pt')
model.train(
    data='mnist160',   # ultralytics 내장 MNIST 데이터셋
    epochs=10,
    imgsz=64,
    batch=128,
    name='yolo_digit_cls',
    verbose=False,
)

best = os.path.join(model.trainer.save_dir, 'weights', 'best.pt')
shutil.copy(best, OUT_PATH)
print(f'저장 완료: {OUT_PATH}')
