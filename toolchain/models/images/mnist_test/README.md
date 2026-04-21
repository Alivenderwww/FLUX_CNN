# `images/mnist_test/`

MNIST 测试集 PNG 图像放这里。

## 自动 dump

```bash
cd toolchain/models
.venv/Scripts/python.exe dump_mnist_png.py --count 64    # 前 64 张
```

## 文件名约定

`NNNN_labelX.png`：4 位 0 padded 索引 + true label（class 0..9）。
例: `0000_label7.png`, `0001_label2.png`。

`run_model.py` 用 `--image-dir` 指向本目录就能批量推理。
