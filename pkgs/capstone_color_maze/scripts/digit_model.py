#!/usr/bin/env python3
"""
digit_model.py
손글씨 숫자(0~9) 인식용 작은 CNN. train_digit.py(학습)와 digit_detector.py(추론)가
같은 구조를 import 해서 쓴다(가중치 호환). cv2/ROS 의존 없음.
"""
import torch.nn as nn
import torch.nn.functional as F


class DigitCNN(nn.Module):
    """입력 1×28×28(흑백, MNIST 규격) → 10클래스 로짓."""
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 16, 3, padding=1)
        self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
        self.fc1 = nn.Linear(32 * 7 * 7, 128)
        self.fc2 = nn.Linear(128, 10)

    def forward(self, x):
        x = F.max_pool2d(F.relu(self.conv1(x)), 2)   # 28 → 14
        x = F.max_pool2d(F.relu(self.conv2(x)), 2)   # 14 → 7
        x = x.flatten(1)
        x = F.relu(self.fc1(x))
        return self.fc2(x)
