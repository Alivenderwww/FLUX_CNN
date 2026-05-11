# 表 5.1 功能仿真验证结果汇总
# Table 5.1 Functional Simulation Verification Summary

## 在论文中的角色

- 首次引入位置：§5.3（功能验证）
- 被哪些段引用：§5.3 末段（line 508 前后）
- 论证作用：将三套基础测试集（单核 ResNet 风格回归、单核形状鲁棒、多核 W 切片）与 ResNet11 整网仿真的功能验证结果汇总，证明 FLUX_CNN 在不同核数（*N*=1/2/4）、不同编译模式（基线 / Ky 折叠 / Ky 折叠+空间到深度 / 多核 W 切片）下输出与 PyTorch 浮点参考全部按位匹配。

## 列定义

| 列名 | 含义 | 数据来源 |
|------|------|----------|
| 测试集 | 测试集分类名称 | toolchain/run_regression.py 用例集 |
| 用例数 | 该测试集包含的独立用例数量 | 回归脚本 CASES 长度 |
| 配置 | 加速核数 *N*（1/2/4） | 编译器 `--num-cores` 参数 |
| 编译模式 | 是否启用 Ky 折叠 / 空间到深度 / W 切片 | 编译器开关 `--ky-fold` / `--s2d` / 多核切片自动 |
| 结果 | PASS 用例数 / 总用例数 | TB CASE_RESULT 输出统计 |

## 行定义

| 行 | 数据来源 | 置信度 |
|----|----------|--------|
| 单核 ResNet 风格回归（基线） | toolchain/run_regression.py 默认 22 case | sim 实测 |
| 单核 ResNet 风格回归（Ky 折叠） | run_regression.py --fold | sim 实测 |
| 单核 ResNet 风格回归（Ky 折叠 + 空间到深度） | run_regression.py --fold --s2d | sim 实测 |
| 单核形状鲁棒 | sim/tb_core_dma 形状鲁棒用例集 | sim 实测 |
| 多核 W 切片 *N*=2 | sim/tb_smc N=2 用例 | sim 实测 |
| 多核 W 切片 *N*=4 | sim/tb_smc N=4 用例 | sim 实测 |
| ResNet11 整网 *N*=1/2/4 | toolchain/models/run_model.py + STATUS.md §2.12 | sim 实测 |

## 表初稿（Markdown 表）

| 测试集 | 用例数 | 配置 | 编译模式 | 结果 |
|---|---|---|---|---|
| 单核 ResNet 风格回归 | 22 | *N*=1 | 未启用编译器优化 | 22/22 PASS |
| 单核 ResNet 风格回归 | 22 | *N*=1 | Ky 折叠 | 22/22 PASS |
| 单核 ResNet 风格回归 | 22 | *N*=1 | Ky 折叠 + 空间到深度 | 22/22 PASS |
| 单核形状鲁棒 | 24 | *N*=1 | 未启用编译器优化 | 24/24 PASS |
| 多核 W 切片 | 10 | *N*=2 | 编译器自动 | 10/10 PASS |
| 多核 W 切片 | 10 | *N*=4 | 编译器自动 | 10/10 PASS |
| ResNet11 整网 | 1 | *N*=1 | 含空间到深度 | 11/11 层 PASS |
| ResNet11 整网 | 1 | *N*=2 | 含空间到深度 + W 切片 | 11/11 层 PASS |
| ResNet11 整网 | 1 | *N*=4 | 含空间到深度 + W 切片 | 11/11 层 PASS |

## 不确定项

- [CHECK] 多核 W 切片 10 个用例的具体 case 列表是否与回归脚本最新一致？建议在 paper.md 引用前用 `grep -c CASES sim/tb_smc/...` 复核。
- [CHECK] 单核形状鲁棒 24 例用例集合当前以 STATUS.md §2.12 为准，回归脚本中是否有同名集合需复核。
- 整体所有行均来自 STATUS.md §2.12 已记录的实测结果，置信度高。

## image 生成提示词

无（表格直接以 markdown 形式嵌入论文）。
