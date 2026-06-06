#!/usr/bin/env python3
"""
train_digit.py
MNIST 손글씨 숫자(0~9) CNN 학습 → ../models/mnist_cnn.pt 저장.
digit_detector / color_detector 가 이 가중치를 불러 추론한다.

실행:  python3 scripts/train_digit.py
"""
import os
import torch
from torch import nn, optim
from torch.utils.data import DataLoader
from torchvision import datasets, transforms

from digit_model import DigitCNN


def main():
    here = os.path.dirname(os.path.realpath(__file__))
    out_dir = os.path.join(os.path.dirname(here), 'models')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'mnist_cnn.pt')

    tf = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,)),
    ])
    data_dir = '/tmp/mnist_data'
    train = datasets.MNIST(data_dir, train=True, download=True, transform=tf)
    test = datasets.MNIST(data_dir, train=False, download=True, transform=tf)
    tl = DataLoader(train, batch_size=128, shuffle=True)
    vl = DataLoader(test, batch_size=256)

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = DigitCNN().to(dev)
    opt = optim.Adam(net.parameters(), lr=1e-3)
    lossf = nn.CrossEntropyLoss()

    for epoch in range(3):
        net.train()
        for x, y in tl:
            x, y = x.to(dev), y.to(dev)
            opt.zero_grad()
            lossf(net(x), y).backward()
            opt.step()
        net.eval()
        correct = total = 0
        with torch.no_grad():
            for x, y in vl:
                x, y = x.to(dev), y.to(dev)
                correct += (net(x).argmax(1) == y).sum().item()
                total += y.numel()
        print(f'epoch {epoch + 1}: test acc {correct / total:.4f}', flush=True)

    torch.save(net.state_dict(), out_path)
    print(f'saved {out_path}', flush=True)


if __name__ == '__main__':
    main()
