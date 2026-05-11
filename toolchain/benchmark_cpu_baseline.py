"""
benchmark_cpu_baseline.py — PyTorch CPU forward 实测各经典网络的单图推理时间

跑 torchvision 内置网络 single-thread CPU forward, 拿 ms / fps 作为 baseline.
运行环境: 当前 PC, 单线程 (torch.set_num_threads(1)).
"""
import time
import torch
import torchvision.models as models

torch.set_num_threads(1)
torch.set_grad_enabled(False)

CONFIGS = [
    ("AlexNet",       lambda: models.alexnet(weights=None),                     227),
    ("VGG-16",        lambda: models.vgg16(weights=None),                       224),
    ("ResNet-18",     lambda: models.resnet18(weights=None),                    224),
    ("ResNet-50",     lambda: models.resnet50(weights=None),                    224),
    ("MobileNet-V1",  lambda: None,                                             224),  # torchvision 没有 v1, skip
    ("MobileNet-V2",  lambda: models.mobilenet_v2(weights=None),                224),
    ("YOLOv2-tiny",   lambda: None,                                             416),  # skip, 无 std impl
]

WARMUP = 3
REPEAT = 5

print(f"PyTorch {torch.__version__}, num_threads={torch.get_num_threads()}, FP32 inference")
print(f"{'Network':<22}{'Input':>10}{'fwd ms':>10}{'fps':>10}")
print("-" * 60)

for name, builder, hw in CONFIGS:
    if builder is None:
        print(f"{name:<22}{hw:>10}    SKIP (no torchvision model)")
        continue
    try:
        model = builder()
        model.eval()
        x = torch.randn(1, 3, hw, hw)
        # warmup
        for _ in range(WARMUP):
            _ = model(x)
        # measure
        t0 = time.perf_counter()
        for _ in range(REPEAT):
            _ = model(x)
        dt = (time.perf_counter() - t0) / REPEAT
        ms = dt * 1000
        fps = 1.0 / dt
        print(f"{name:<22}{hw:>10}{ms:>10.1f}{fps:>10.2f}")
    except Exception as e:
        print(f"{name:<22}{hw:>10}    ERR ({e})")
