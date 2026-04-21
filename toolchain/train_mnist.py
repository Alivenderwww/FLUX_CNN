"""
train_mnist.py — 训练 all-conv MNIST 模型（无 Pool，用 stride=2 下采样）

网络结构（所有卷积 Cin/Cout ≤ 16，K ∈ {3,7}，现有硬件全支持）：
  L0: Conv(1→16, K3, p1, s1) + ReLU    28x28x16
  L1: Conv(16→16, K3, p1, s2) + ReLU   14x14x16
  L2: Conv(16→16, K3, p1, s1) + ReLU   14x14x16
  L3: Conv(16→16, K3, p1, s2) + ReLU   7x7x16
  L4: Conv(16→10, K7, p0, s1)          1x1x10   # FC 等价，无 ReLU

用法：
  .venv/Scripts/python.exe train_mnist.py         # 默认 8 epoch CPU
  .venv/Scripts/python.exe train_mnist.py --epochs 12 --lr 5e-4

输出：
  toolchain/models/mnist_allconv.pt    state_dict + calib_image + label
  测试准确率打印到 stdout
"""

import os
import argparse
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from torchvision import datasets, transforms


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR   = os.path.join(_SCRIPT_DIR, "models")
DATA_DIR    = os.path.join(_SCRIPT_DIR, "data")
CKPT_PATH   = os.path.join(MODEL_DIR, "mnist_allconv.pt")


def build_allconv():
    return nn.Sequential(
        nn.Conv2d(1,  16, 3, padding=1, stride=1, bias=True), nn.ReLU(),
        nn.Conv2d(16, 16, 3, padding=1, stride=2, bias=True), nn.ReLU(),
        nn.Conv2d(16, 16, 3, padding=1, stride=1, bias=True), nn.ReLU(),
        nn.Conv2d(16, 16, 3, padding=1, stride=2, bias=True), nn.ReLU(),
        nn.Conv2d(16, 10, 7, padding=0, stride=1, bias=True),  # 无 ReLU: logits
    )


class AllConvMNIST(nn.Module):
    """forward squeeze 1x1 → (N, 10) 便于 CE loss。"""
    def __init__(self):
        super().__init__()
        self.net = build_allconv()

    def forward(self, x):
        y = self.net(x)          # (N, 10, 1, 1)
        return y.flatten(1)      # (N, 10)


def train(epochs=8, lr=1e-3, batch_size=128, seed=0):
    torch.manual_seed(seed)
    os.makedirs(MODEL_DIR, exist_ok=True)
    os.makedirs(DATA_DIR,  exist_ok=True)

    # MNIST 标准 normalize: mean=0.1307, std=0.3081
    tfm = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,)),
    ])
    train_ds = datasets.MNIST(DATA_DIR, train=True,  download=True, transform=tfm)
    test_ds  = datasets.MNIST(DATA_DIR, train=False, download=True, transform=tfm)
    train_dl = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=0)
    test_dl  = DataLoader(test_ds,  batch_size=256, shuffle=False, num_workers=0)

    model = AllConvMNIST()
    opt   = optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.CrossEntropyLoss()

    for ep in range(1, epochs + 1):
        model.train()
        total = 0
        total_loss = 0.0
        correct = 0
        for imgs, labels in train_dl:
            opt.zero_grad()
            logits = model(imgs)
            loss   = loss_fn(logits, labels)
            loss.backward()
            opt.step()
            total += labels.size(0)
            total_loss += loss.item() * labels.size(0)
            correct += (logits.argmax(1) == labels).sum().item()

        # test
        model.eval()
        te_total = 0
        te_correct = 0
        with torch.no_grad():
            for imgs, labels in test_dl:
                logits = model(imgs)
                te_total += labels.size(0)
                te_correct += (logits.argmax(1) == labels).sum().item()
        print(f"[epoch {ep:>2}/{epochs}] "
              f"train_loss={total_loss/total:.4f} train_acc={correct/total*100:.2f}% "
              f"test_acc={te_correct/te_total*100:.2f}%")

    final_acc = te_correct / te_total

    # 取 test set 第 0 张作 calibration image（固定种子便于复现）
    calib_image, calib_label = test_ds[0]   # (1,28,28), int
    # calib input 用 "一批" 同 distribution 的图片做 scale calibration 更稳
    # 这里额外把前 64 张存一个 batch 给 scale 估计
    calib_batch = torch.stack([test_ds[i][0] for i in range(64)], dim=0)  # (64,1,28,28)
    calib_labels = torch.tensor([test_ds[i][1] for i in range(64)])

    torch.save({
        'state_dict'  : model.state_dict(),
        'calib_image' : calib_image.unsqueeze(0),   # (1,1,28,28) — 待验证那张
        'calib_label' : int(calib_label),
        'calib_batch' : calib_batch,                # scale 估计用
        'calib_labels': calib_labels,
        'test_acc'    : final_acc,
        'epochs'      : epochs,
    }, CKPT_PATH)
    print(f"saved → {CKPT_PATH}")
    print(f"calib image 0 label = {int(calib_label)}")
    print(f"final test_acc = {final_acc*100:.2f}%")
    return final_acc


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--epochs',     type=int,   default=8)
    p.add_argument('--lr',         type=float, default=1e-3)
    p.add_argument('--batch-size', type=int,   default=128)
    p.add_argument('--seed',       type=int,   default=0)
    args = p.parse_args()
    train(args.epochs, args.lr, args.batch_size, args.seed)
