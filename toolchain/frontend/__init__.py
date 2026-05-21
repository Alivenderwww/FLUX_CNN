"""frontend/ — 编译链前端: model.pt / Chain DSL YAML → ChainDSL IR.

Phase C: forwarding stubs (老 import 路径不破坏, 新代码可用 frontend.* import).
Phase E 后量化将独立到 frontend.quantize.

import 示例:
    from toolchain.frontend import pytorch_to_dsl
    # 或者
    from toolchain.frontend.dsl_loader import Layer
"""
