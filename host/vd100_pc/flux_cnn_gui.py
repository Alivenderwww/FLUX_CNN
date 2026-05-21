#!/usr/bin/env python3
"""flux_cnn_gui.py — VD100 通用 demo 上位机 GUI (重构版)

跟 vd100_rpc.py + deploy_smc_case.py + toolchain/run_multicore_chain.py 配对.
板上跑通用 RPC server, PC 端通过本 GUI 走完整流水:

  ① 硬件   — 加载 vd100.json (架构/部署 配置), 显示 arch + deployment region 布局
  ② 模型   — 选 demo (或已编译 case dir), 调 toolchain 编译生成 desc / SG cmd / wb
  ③ 部署   — 一键把所有 region (IFB/WB/RDMA/desc/SG cmd) 装载到板
  ④ 推理   — 图片上传 + 触发 RUN_LAYERS + 读 OFM + 比对 (逐 layer / 仅终层)
  ⑤ 性能   — cycles 统计, 历史曲线, jitter, per-layer breakdown

依赖:  pip install Pillow tkinterdnd2

启动:  python flux_cnn_gui.py     # 或 run_gui.bat 双击
"""
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    from tkinterdnd2 import TkinterDnD, DND_FILES
    HAS_DND = True
except ImportError:
    TkinterDnD = None
    DND_FILES = None
    HAS_DND = False

try:
    import yaml
    HAS_YAML = True
except ImportError:
    yaml = None
    HAS_YAML = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'toolchain'))
import vd100_rpc                                       # noqa: E402
import deploy_smc_case as dsc                          # noqa: E402
from vd100_rpc import Vd100Rpc, NUM_CORES, CSR_STATUS  # noqa: E402
try:
    from hardware.hardware_cfg import HardwareConfig    # noqa: E402
    HAS_HW_CFG = True
except Exception:
    HardwareConfig = None
    HAS_HW_CFG = False

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_TOOLCHAIN_DIR = os.path.join(_PROJECT_ROOT, 'toolchain')
_GUI_DIR       = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR     = os.path.join(_GUI_DIR, 'models')
DEMO_DIR       = os.path.join(MODELS_DIR, 'demo')
COMPILED_DIR   = os.path.join(MODELS_DIR, 'compiled')
DEFAULT_HW_JSON = os.path.join(_TOOLCHAIN_DIR, 'hardware', 'vd100.json')
# Default 找 models/compiled/resnet11_n2/, fallback 老路径 sim/tb_smc/cases/
DEFAULT_CASE_DIR = os.path.join(COMPILED_DIR, 'resnet11_n2')
_LEGACY_CASE_DIR = os.path.join(_PROJECT_ROOT, 'sim', 'tb_smc', 'cases', 'resnet11_n2')


# ---------------------------------------------------------------------------
# Color palette (Catppuccin Mocha)
# ---------------------------------------------------------------------------
CL_BG     = "#1e1e2e"
CL_PANEL  = "#2a2a3e"
CL_FG     = "#cdd6f4"
CL_DIM    = "#7f849c"
CL_OK     = "#a6e3a1"
CL_WARN   = "#f9e2af"
CL_ERR    = "#f38ba8"
CL_ACCENT = "#89b4fa"
CL_BAR    = "#74c7ec"


# ---------------------------------------------------------------------------
# 共享 GUI state (跨 tab)
# ---------------------------------------------------------------------------
class AppState:
    """所有 tab 共享的运行时 state. 用 callback 通知 UI 刷新."""
    def __init__(self):
        # 硬件配置
        self.hw_json_path: str | None = None
        self.hw_cfg = None                # HardwareConfig dataclass
        # 模型 case
        self.case_dir: str | None = None
        self.case_meta: dict | None = None
        self.global_base: int = 0
        # 当前 case 关联的 demo manifest (用于推理 tab 反推 input_cin 等)
        self.current_manifest: dict | None = None
        # RPC 连接
        self.rpc: Vd100Rpc | None = None
        # 部署状态
        self.deployed: bool = False
        # 图片 IFM
        self.image_path: str | None = None
        self.image_bytes_s2d: bytes | None = None
        self.image_uploaded: bool = False
        # 推理结果
        self.last_cycles: int = 0
        self.cycles_history: list = []
        self.per_layer_pass: list = []
        self.per_layer_fail: list = []
        # callback registry
        self._listeners: list = []

    def on_change(self, cb):
        """注册 state 改变 callback. cb() 不带参."""
        self._listeners.append(cb)

    def notify(self):
        for cb in self._listeners:
            try:
                cb()
            except Exception as e:
                print(f"[state] listener error: {e}", file=sys.stderr)


# ---------------------------------------------------------------------------
# 通用 widgets (跟老版相同)
# ---------------------------------------------------------------------------
class StatusLED(tk.Label):
    """彩色圆点指示灯."""
    def __init__(self, master, **kw):
        super().__init__(master, text="●", fg=CL_DIM, bg=CL_PANEL,
                         font=("Arial", 14, "bold"), **kw)

    def set_state(self, color: str):
        self.config(fg=color)


class LogPane(tk.Frame):
    """带滚动条 + 颜色等级的日志面板."""
    def __init__(self, master, height=8, **kw):
        super().__init__(master, bg=CL_PANEL, **kw)
        self.text = tk.Text(self, bg="#11111b", fg=CL_FG,
                            font=("Consolas", 9), height=height, wrap="none",
                            insertbackground=CL_FG, borderwidth=0)
        self.text.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(self, orient="vertical", command=self.text.yview)
        sb.pack(side="right", fill="y")
        self.text.configure(yscrollcommand=sb.set)
        self.text.tag_config("INFO", foreground=CL_FG)
        self.text.tag_config("OK",   foreground=CL_OK)
        self.text.tag_config("WARN", foreground=CL_WARN)
        self.text.tag_config("ERR",  foreground=CL_ERR)
        self.text.tag_config("RPC",  foreground=CL_ACCENT)
        self.text.tag_config("DIM",  foreground=CL_DIM)
        self.text.configure(state="disabled")

    def log(self, level: str, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.text.configure(state="normal")
        self.text.insert("end", f"[{ts}] ", "DIM")
        self.text.insert("end", f"{msg}\n", level)
        self.text.see("end")
        self.text.configure(state="disabled")

    def info(self, m): self.log("INFO", m)
    def ok  (self, m): self.log("OK",   m)
    def warn(self, m): self.log("WARN", m)
    def err (self, m): self.log("ERR",  m)
    def rpc (self, m): self.log("RPC",  m)


def set_text(widget: tk.Text, content: str):
    widget.configure(state="normal")
    widget.delete("1.0", "end")
    widget.insert("1.0", content)
    widget.configure(state="disabled")


def derive_input_spec(case_meta: dict, manifest: dict | None = None) -> dict:
    """从 case_meta 反推 layer 0 输入特性, 返回 dict.

    manifest (可选, model.yaml dict): 可写 input_cin 字段 override 反推. 因为
    case_meta 只有 cin_padded (= cin_slices*16), 反推不出真实通道数 (如 MNIST
    grayscale cin_real=1 但 cin_padded=16). 默认 fallback:
      - patch (K==stride>=2): cin_real = cin_padded // (K*K)
      - non-patch: cin_real = cin_padded (默认; manifest 写 input_cin=1 可 override)

    字段:
      h_board   : 板上 IFB 期望的 H (= SMC_LAYER_0_H_IN)
      w_board   : 板上 IFB 期望的 W (= SMC_LAYER_0_W_IN)
      cin_padded: 板上 padded 通道 = cin_slices * 16
      k, stride : layer 0 conv K + stride
      is_patch  : K==stride and K>=2 → True (输入做 s2d 重排)
      h_orig    : 原图 H = h_board * (K if is_patch else 1)
      w_orig    : 原图 W = w_board * (K if is_patch else 1)
      cin_real  : 原图通道 (manifest override 优先, 否则反推)
      pil_mode  : 'L'/'RGB'/'RGBA' or None (cin_real 不在 1/3/4 之列)
    """
    h_board = int(case_meta.get('SMC_LAYER_0_H_IN', 0))
    w_board = int(case_meta.get('SMC_LAYER_0_W_IN', 0))
    cin_slices = int(case_meta.get('SMC_LAYER_0_C_IN_SLICES', 1))
    cin_padded = cin_slices * 16
    k = int(case_meta.get('SMC_LAYER_0_K', 1))
    stride = int(case_meta.get('SMC_LAYER_0_STRIDE', 1))
    is_patch = (k == stride) and (k >= 2)
    # 反推 cin_real
    manifest_cin = None
    if manifest and 'input_cin' in manifest:
        try: manifest_cin = int(manifest['input_cin'])
        except (ValueError, TypeError): pass
    if manifest_cin is not None:
        cin_real = manifest_cin
    elif is_patch and (cin_padded % (k * k) == 0):
        cin_real = cin_padded // (k * k)
    else:
        cin_real = cin_padded
    # H/W: patch s2d 时原图是 board 的 K 倍
    if is_patch:
        h_orig = h_board * k
        w_orig = w_board * k
    else:
        h_orig = h_board
        w_orig = w_board
    pil_mode = {1: 'L', 3: 'RGB', 4: 'RGBA'}.get(cin_real)
    return dict(h_board=h_board, w_board=w_board, cin_slices=cin_slices,
                cin_padded=cin_padded, k=k, stride=stride, is_patch=is_patch,
                h_orig=h_orig, w_orig=w_orig, cin_real=cin_real, pil_mode=pil_mode)


def load_image_to_ifm(path: str, spec: dict, manifest: dict | None = None,
                       input_scale: float | None = None) -> tuple:
    """PIL 加载图 + resize 到 (h_orig, w_orig) + 量化 int8 + 可选 s2d 重排.

    manifest preprocess pipeline (跟训练 / toolchain calib 同源):
      - 'torchvision' + mean/std + input_scale: 走 ToTensor + Normalize + symmetric quantize
        (PyTorch 真模型必走, 跟 toolchain calib bit-exact 一致)
      - 无 preprocess (None) / synthetic_chain: 走简单 byte-128 centered (老逻辑)

    返回 (ifm_orig_bytes, ifm_board_bytes, orig_size).
    """
    from PIL import Image
    pil_mode = spec['pil_mode']
    if pil_mode is None:
        raise ValueError(f"不支持的通道数 cin_real={spec['cin_real']} (只支持 1/3/4)")
    img = Image.open(path).convert(pil_mode)
    orig_size = img.size
    img_resized = img.resize((spec['w_orig'], spec['h_orig']), Image.BILINEAR)

    pre = (manifest or {}).get('preprocess', {})
    if pre.get('pipeline') == 'torchvision' and input_scale is not None:
        # 走 toolchain pipeline: PIL → ToTensor (0..1 float) → Normalize → quantize_symmetric
        import torch
        from torchvision import transforms
        mean = pre.get('mean', [0.0])
        std  = pre.get('std',  [1.0])
        tfm = transforms.Compose([transforms.ToTensor(),
                                   transforms.Normalize(mean, std)])
        x = tfm(img_resized).unsqueeze(0)   # (1, C, H, W) float
        # symmetric quantize → int8 list
        q = torch.clamp(torch.round(x / input_scale), -128, 127).to(torch.int8).numpy()
        # NCHW → NHWC byte
        H, W = spec['h_orig'], spec['w_orig']
        cin_real = spec['cin_real']
        buf = bytearray(H * W * cin_real)
        for r in range(H):
            for c in range(W):
                for ch in range(cin_real):
                    v = int(q[0, ch, r, c]) & 0xFF
                    buf[(r*W + c) * cin_real + ch] = v
        ifm_orig = bytes(buf)
    else:
        # 老逻辑: uint8 → int8 centered (合成 demo 兼容)
        raw = img_resized.tobytes()
        ifm_orig = bytes((b - 128) & 0xFF for b in raw)

    if not spec['is_patch']:
        # 非 patch: NHWC, 每 pixel 取 cin_real byte; 但板上期望 cin_padded byte/pixel.
        # cin_real == cin_padded → 直接 return; 否则每 pixel pad (cin_padded - cin_real) 个 0.
        cin_real = spec['cin_real']
        cin_padded = spec['cin_padded']
        if cin_real == cin_padded:
            return ifm_orig, ifm_orig, orig_size
        Hb = spec['h_board']; Wb = spec['w_board']
        dst = bytearray(Hb * Wb * cin_padded)   # 默认 0
        for r in range(Hb):
            for c in range(Wb):
                src_off = (r * Wb + c) * cin_real
                dst_off = (r * Wb + c) * cin_padded
                dst[dst_off : dst_off + cin_real] = ifm_orig[src_off : src_off + cin_real]
        return ifm_orig, bytes(dst), orig_size

    # patch s2d 重排: 把 K×K block 的 cin_real 个通道展平到 cin_padded
    # 每个 (sr,sc) sub-pixel 占 cin_block = cin_padded // (K*K) byte, 前 cin_real
    # byte 是真实通道, 后面 0-padded (例如 RGB 3 → K=4 patch 时块大小 4, 余 1 byte 0).
    K = spec['k']
    W = spec['w_orig']
    Hb = spec['h_board']; Wb = spec['w_board']
    cin_real = spec['cin_real']
    cin_padded = spec['cin_padded']
    cin_block = cin_padded // (K * K)
    if cin_block < cin_real:
        raise ValueError(f"cin_padded={cin_padded} / K²={K*K} = {cin_block} < cin_real={cin_real}, "
                         "manifest input_cin 或 case layer 0 配置错")
    dst = bytearray(Hb * Wb * cin_padded)
    for rp in range(Hb):
        for cp in range(Wb):
            dst_base = (rp * Wb + cp) * cin_padded
            for sr in range(K):
                for sc in range(K):
                    r = rp * K + sr; c = cp * K + sc
                    src_base = (r * W + c) * cin_real
                    dst_off = dst_base + (sr * K + sc) * cin_block
                    dst[dst_off : dst_off + cin_real] = ifm_orig[src_base : src_base + cin_real]
                    # cin_block - cin_real byte 留 0 padding
    return ifm_orig, bytes(dst), orig_size


def scan_demos(demo_dir: str = DEMO_DIR) -> dict:
    """扫描 demo/ 子目录, 返回 {name: manifest_dict}.

    跳过没 model.yaml 的目录 + 解析失败的目录.
    manifest 额外注入 _dir (absolute path) + _name.
    """
    out = {}
    if not os.path.isdir(demo_dir):
        return out
    if not HAS_YAML:
        return out
    for name in sorted(os.listdir(demo_dir)):
        sub = os.path.join(demo_dir, name)
        yml = os.path.join(sub, 'model.yaml')
        if not (os.path.isdir(sub) and os.path.isfile(yml)):
            continue
        try:
            with open(yml, 'r', encoding='utf-8') as f:
                m = yaml.safe_load(f) or {}
            m['_dir']  = sub
            m['_name'] = name
            out[name] = m
        except Exception as e:
            print(f"[scan_demos] {name}: {e}", file=sys.stderr)
    return out


def parse_dnd_path(data: str) -> str:
    """tkinterdnd2 给的路径可能用 {} 包含空格."""
    s = data.strip()
    if s.startswith("{") and "}" in s:
        return s[1:s.index("}")]
    return s.split()[0] if s else ""


# ---------------------------------------------------------------------------
# 主 GUI (壳子 + tab 容器; tab 各 module 之后注入)
# ---------------------------------------------------------------------------
class FluxCnnGui:
    def __init__(self, root):
        self.root = root
        root.title("FLUX_CNN VD100 控制台")
        root.geometry("1320x880")
        root.configure(bg=CL_BG)

        self.state = AppState()
        self.msg_q: queue.Queue = queue.Queue()
        self._build_style()
        self._build_ui()
        self.root.after(80, self._poll_msgq)

        # 自动尝试加载默认 hw json (若存在)
        if os.path.isfile(DEFAULT_HW_JSON):
            self.root.after(150, lambda: self._auto_load_hw_json(DEFAULT_HW_JSON))

    # =========================================================================
    def _build_style(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure(".",            background=CL_BG, foreground=CL_FG,
                                    fieldbackground=CL_PANEL)
        s.configure("TLabelframe",  background=CL_BG, foreground=CL_FG,
                                    bordercolor=CL_DIM)
        s.configure("TLabelframe.Label", background=CL_BG, foreground=CL_ACCENT,
                                    font=("Arial", 9, "bold"))
        s.configure("TFrame",       background=CL_BG)
        s.configure("TLabel",       background=CL_BG, foreground=CL_FG)
        s.configure("TButton",      background=CL_PANEL, foreground=CL_FG, borderwidth=1)
        s.map      ("TButton",      background=[("active", CL_ACCENT)],
                                    foreground=[("active", "#11111b")])
        s.configure("Accent.TButton", background=CL_ACCENT, foreground="#11111b",
                                    font=("Arial", 10, "bold"))
        s.map      ("Accent.TButton", background=[("active", "#b4befe")])
        s.configure("OK.TButton",   background=CL_OK, foreground="#11111b",
                                    font=("Arial", 10, "bold"))
        s.configure("Treeview",     background=CL_PANEL, fieldbackground=CL_PANEL,
                                    foreground=CL_FG, rowheight=22)
        s.configure("Treeview.Heading", background=CL_BG, foreground=CL_ACCENT,
                                    font=("Arial", 9, "bold"))
        s.configure("TEntry",       fieldbackground=CL_PANEL, foreground=CL_FG)
        s.configure("TNotebook",    background=CL_BG, borderwidth=0)
        s.configure("TNotebook.Tab", background=CL_PANEL, foreground=CL_FG,
                                    padding=(14, 6), font=("Arial", 10, "bold"))
        s.map      ("TNotebook.Tab", background=[("selected", CL_ACCENT)],
                                     foreground=[("selected", "#11111b")])
        s.configure("Horizontal.TProgressbar", background=CL_BAR, troughcolor=CL_PANEL)

    def _build_ui(self):
        # ---- 顶部栏 ----
        top = tk.Frame(self.root, bg=CL_PANEL, height=44)
        top.pack(fill="x", side="top")
        top.pack_propagate(False)
        tk.Label(top, text="FLUX_CNN", bg=CL_PANEL, fg=CL_ACCENT,
                 font=("Arial", 14, "bold")).pack(side="left", padx=(12, 4))
        tk.Label(top, text="VD100 通用上位机 (硬件→模型→部署→推理→性能)",
                 bg=CL_PANEL, fg=CL_DIM, font=("Arial", 9)).pack(side="left", padx=4)

        rt = tk.Frame(top, bg=CL_PANEL)
        rt.pack(side="right", padx=12)
        self.stats_label = tk.Label(rt, text="", bg=CL_PANEL, fg=CL_DIM,
                                     font=("Consolas", 9))
        self.stats_label.pack(side="right", padx=8)
        self.ping_label = tk.Label(rt, text="ping —", bg=CL_PANEL, fg=CL_DIM,
                                    font=("Consolas", 9))
        self.ping_label.pack(side="right", padx=8)
        self.conn_led = StatusLED(rt)
        self.conn_led.pack(side="right", padx=4)
        self.conn_text = tk.Label(rt, text="未连接", bg=CL_PANEL, fg=CL_DIM,
                                   font=("Arial", 10, "bold"))
        self.conn_text.pack(side="right")

        # ---- 连接行 ----
        conn = ttk.LabelFrame(self.root, text="连接", padding=6)
        conn.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Label(conn, text="VD100 IP:").grid(row=0, column=0, sticky="w")
        self.ip_var = tk.StringVar(value="169.254.111.10")
        ttk.Entry(conn, textvariable=self.ip_var, width=18).grid(row=0, column=1, padx=(2, 12))
        ttk.Label(conn, text="Port:").grid(row=0, column=2, sticky="w")
        self.port_var = tk.StringVar(value="5000")
        ttk.Entry(conn, textvariable=self.port_var, width=8).grid(row=0, column=3, padx=(2, 12))
        self.connect_btn    = ttk.Button(conn, text="连接", style="Accent.TButton",
                                          command=self._on_connect)
        self.connect_btn.grid(row=0, column=4, padx=4)
        self.disconnect_btn = ttk.Button(conn, text="断开",
                                          command=self._on_disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=5, padx=4)
        self.ping_btn       = ttk.Button(conn, text="PING",
                                          command=self._on_ping_once, state="disabled")
        self.ping_btn.grid(row=0, column=6, padx=4)

        # ---- 中部 Notebook ----
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True, padx=8, pady=4)

        # 每个 tab 是一个 Frame, 真正内容由 Step 2-6 各 _build_tab_* 填充
        self.tab_hw     = ttk.Frame(self.nb, padding=8)
        self.tab_model  = ttk.Frame(self.nb, padding=8)
        self.tab_deploy = ttk.Frame(self.nb, padding=8)
        self.tab_infer  = ttk.Frame(self.nb, padding=8)
        self.tab_perf   = ttk.Frame(self.nb, padding=8)
        self.nb.add(self.tab_hw,     text="① 硬件")
        self.nb.add(self.tab_model,  text="② 模型")
        self.nb.add(self.tab_deploy, text="③ 部署")
        self.nb.add(self.tab_infer,  text="④ 推理")
        self.nb.add(self.tab_perf,   text="⑤ 性能")

        # Tab 内容由各 _build_tab_* 填充
        self._build_tab_hardware()
        self._build_tab_model()
        self._build_tab_deploy()
        self._build_tab_inference()
        self._build_tab_perf()

        # ---- 底部日志 ----
        log_frame = ttk.LabelFrame(self.root, text="日志 (RPC + 状态)", padding=6)
        log_frame.pack(fill="x", padx=8, pady=(4, 4))
        self.log = LogPane(log_frame, height=7)
        self.log.pack(fill="x")
        self.log.info("GUI 启动. " + ("拖拽 OK" if HAS_DND else "拖拽未装 (pip install tkinterdnd2)"))

        # ---- 状态栏 ----
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(self.root, textvariable=self.status_var, anchor="w",
                  background=CL_PANEL, foreground=CL_FG).pack(side="bottom", fill="x")

    # =========================================================================
    # 连接 RPC (跟老版逻辑等价)
    # =========================================================================
    def _on_connect(self):
        ip = self.ip_var.get().strip()
        try:
            port = int(self.port_var.get())
        except ValueError:
            messagebox.showerror("Port 非法", "Port 必须是数字"); return
        self.connect_btn.config(state="disabled")
        self.status_var.set(f"连接 {ip}:{port}...")
        self.log.info(f"连接 {ip}:{port}...")
        self.root.update_idletasks()
        try:
            rpc = Vd100Rpc(ip, port, timeout=5.0)
            rpc.connect()
            magic = rpc.ping()
            rpc.sock.settimeout(90.0)
        except Exception as e:
            self.log.err(f"连接失败: {e}")
            messagebox.showerror("连接失败", f"{ip}:{port}\n{e}")
            self.status_var.set(f"连接失败: {e}")
            self.connect_btn.config(state="normal")
            return
        self.state.rpc = rpc
        self.conn_led.set_state(CL_OK)
        self.conn_text.config(text="已连接", fg=CL_OK)
        self.disconnect_btn.config(state="normal")
        self.ping_btn.config(state="normal")
        self.log.ok(f"已连接 {ip}:{port}, ping=0x{magic:08x}")
        self.status_var.set(f"已连接 {ip}:{port}")
        self.state.notify()

    def _on_disconnect(self):
        if self.state.rpc:
            try: self.state.rpc.close()
            except Exception: pass
            self.state.rpc = None
        self.conn_led.set_state(CL_DIM)
        self.conn_text.config(text="未连接", fg=CL_DIM)
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.ping_btn.config(state="disabled")
        self.state.deployed = False
        self.state.image_uploaded = False
        self.ping_label.config(text="ping —")
        self.log.warn("已断开")
        self.status_var.set("已断开")
        self.state.notify()

    def _on_ping_once(self):
        if not self.state.rpc: return
        try:
            t0 = time.time()
            v = self.state.rpc.ping()
            rtt = (time.time() - t0) * 1000
            self.log.rpc(f"PING → 0x{v:08x} ({rtt:.1f}ms)")
            self.ping_label.config(text=f"ping {rtt:.1f}ms",
                                    fg=CL_OK if rtt < 50 else CL_WARN)
        except Exception as e:
            self.log.err(f"PING 失败: {e}")
            self._on_disconnect()

    # =========================================================================
    # Tab ① 硬件 - vd100.json 加载 + arch/deployment 视图
    # =========================================================================
    def _build_tab_hardware(self):
        t = self.tab_hw
        # 文件选择
        load = ttk.LabelFrame(t, text="硬件配置 JSON (toolchain/hardware/*.json)", padding=8)
        load.pack(fill="x", pady=(0, 6))
        self.hw_path_label = tk.Label(load, text="(拖入 vd100.json 或点 Browse)",
                                       bg="#181828", fg=CL_DIM,
                                       font=("Consolas", 9), anchor="w",
                                       padx=8, pady=10, relief="solid", borderwidth=1)
        self.hw_path_label.pack(fill="x", pady=(0, 6))
        if HAS_DND:
            self.hw_path_label.drop_target_register(DND_FILES)
            self.hw_path_label.dnd_bind("<<Drop>>", self._on_drop_hw)
        row = tk.Frame(load, bg=CL_BG); row.pack(fill="x")
        ttk.Button(row, text="Browse...", command=self._on_browse_hw).pack(side="left", padx=2)
        ttk.Button(row, text="重新加载", command=self._reload_hw).pack(side="left", padx=2)
        ttk.Button(row, text="校验布局", command=self._validate_hw).pack(side="left", padx=2)

        # 两栏: arch | deployment
        cols = tk.Frame(t, bg=CL_BG); cols.pack(fill="both", expand=True)
        arch_frame = ttk.LabelFrame(cols, text="ARCH (RTL 硬件常量, 改后需重综合)", padding=6)
        arch_frame.pack(side="left", fill="both", expand=True, padx=(0, 4))
        dep_frame  = ttk.LabelFrame(cols, text="DEPLOYMENT (部署地址 + region, 改后只需重 deploy)", padding=6)
        dep_frame.pack(side="left", fill="both", expand=True, padx=(4, 0))

        self.arch_tree = ttk.Treeview(arch_frame, columns=("value",), show="tree headings",
                                       height=18)
        self.arch_tree.heading("#0", text="字段"); self.arch_tree.column("#0", width=170, anchor="w")
        self.arch_tree.heading("value", text="值"); self.arch_tree.column("value", width=140, anchor="e")
        self.arch_tree.pack(fill="both", expand=True)

        self.dep_tree = ttk.Treeview(dep_frame, columns=("value",), show="tree headings",
                                      height=18)
        self.dep_tree.heading("#0", text="字段"); self.dep_tree.column("#0", width=200, anchor="w")
        self.dep_tree.heading("value", text="值"); self.dep_tree.column("value", width=160, anchor="e")
        self.dep_tree.pack(fill="both", expand=True)

        # 校验结果
        chk_frame = ttk.LabelFrame(t, text="布局校验 (region 不重叠 + ≤ smc_mem_stride)", padding=6)
        chk_frame.pack(fill="x", pady=(6, 0))
        self.hw_chk_text = tk.Text(chk_frame, height=4, bg=CL_PANEL, fg=CL_FG,
                                    font=("Consolas", 9), borderwidth=0, state="disabled")
        self.hw_chk_text.pack(fill="x")

    def _on_drop_hw(self, event):
        p = parse_dnd_path(event.data)
        if p and os.path.isfile(p):
            self._load_hw_json(p)
        else:
            self.log.err(f"拖入的不是文件: {p}")

    def _on_browse_hw(self):
        p = filedialog.askopenfilename(
            title="选硬件 JSON", initialdir=os.path.dirname(DEFAULT_HW_JSON),
            filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if p:
            self._load_hw_json(p)

    def _reload_hw(self):
        if self.state.hw_json_path:
            self._load_hw_json(self.state.hw_json_path)

    def _auto_load_hw_json(self, path: str):
        self._load_hw_json(path, silent=True)

    def _load_hw_json(self, path: str, silent: bool = False):
        if not HAS_HW_CFG:
            self.log.err("hardware_cfg 模块加载失败 (toolchain 路径不对?)")
            return
        try:
            cfg = HardwareConfig.load(path)
        except Exception as e:
            self.log.err(f"加载 hw json 失败: {e}")
            messagebox.showerror("加载失败", f"{path}\n{e}")
            return
        self.state.hw_json_path = path
        self.state.hw_cfg = cfg
        self.hw_path_label.config(text=f"✓ {path}", fg=CL_OK)

        # 填 ARCH tree
        self.arch_tree.delete(*self.arch_tree.get_children())
        arch = cfg.arch
        for k, v in arch.__dict__.items():
            self.arch_tree.insert("", "end", text=k, values=(str(v),))
        # 派生量
        self.arch_tree.insert("", "end", text="(派生) ifb_word_bytes",
                              values=(str(arch.ifb_word_bytes),))
        self.arch_tree.insert("", "end", text="(派生) wb_word_bytes",
                              values=(str(arch.wb_word_bytes),))

        # 填 DEPLOYMENT tree
        self.dep_tree.delete(*self.dep_tree.get_children())
        dep = cfg.deployment
        self.dep_tree.insert("", "end", text="target", values=(dep.target,))
        self.dep_tree.insert("", "end", text="num_cores", values=(str(dep.num_cores),))
        self.dep_tree.insert("", "end", text="ddr_base", values=(f"0x{dep.ddr_base:08x}",))
        self.dep_tree.insert("", "end", text="ddr_size_mb", values=(str(dep.ddr_size_mb),))
        self.dep_tree.insert("", "end", text="smc_mem_stride", values=(f"0x{dep.smc_mem_stride:08x}",))
        rg_node = self.dep_tree.insert("", "end", text="regions/", values=("",), open=True)
        r = dep.regions
        for k, v in r.__dict__.items():
            self.dep_tree.insert(rg_node, "end", text=k, values=(f"0x{v:08x}",))

        if not silent:
            self.log.ok(f"硬件 JSON 加载: {os.path.basename(path)}, "
                         f"NUM_PE={arch.num_pe} IFB_DEPTH={arch.ifb_depth} "
                         f"NUM_CORES={dep.num_cores}")
        self._validate_hw(silent=silent)
        self.state.notify()

    def _validate_hw(self, silent: bool = False):
        """检查 region 不重叠 + 全部 ≤ smc_mem_stride."""
        cfg = self.state.hw_cfg
        if cfg is None:
            set_text(self.hw_chk_text, "(尚未加载 JSON)")
            return
        r = cfg.deployment.regions
        stride = cfg.deployment.smc_mem_stride
        # 收集 region 起点 + 一个安全 size 估计 (next region - 当前 region)
        items = [
            ("ifm_ofm",   r.ifm_ofm_base),
            ("wb",        r.wb_base),
            ("rdma",      r.rdma_base),
            ("desc",      r.desc_base),
            ("idma_cmd",  r.idma_cmd_base),
            ("odma_cmd",  r.odma_cmd_base),
            ("input",     r.input_base),
            ("final_ofm", r.final_ofm_base),
        ]
        items_sorted = sorted(items, key=lambda x: x[1])
        msgs = []
        ok_all = True
        for i, (name, base) in enumerate(items_sorted):
            if base >= stride:
                msgs.append(f"[ERR] {name}@0x{base:x} ≥ smc_mem_stride 0x{stride:x}")
                ok_all = False
            if i + 1 < len(items_sorted):
                next_name, next_base = items_sorted[i + 1]
                if next_base <= base:
                    msgs.append(f"[ERR] {name}@0x{base:x} ≥ {next_name}@0x{next_base:x} (顺序错)")
                    ok_all = False
        if ok_all and not msgs:
            msgs.append(f"OK — 8 个 region 起点严格升序, 全部 < smc_mem_stride (0x{stride:x})")
        set_text(self.hw_chk_text, "\n".join(msgs))
        if not silent:
            (self.log.ok if ok_all else self.log.err)(
                "布局校验: " + ("PASS" if ok_all else "FAIL (见校验面板)"))

    # =========================================================================
    # Tab ② 模型 - demo 编译 + 已编译 case 加载 + layer 表
    def _build_tab_model(self):
        t = self.tab_model
        # 扫描 demo 目录
        self.demos = scan_demos()   # {name: manifest}

        # 上半: 编译 OR 加载已编译 case
        src = ttk.LabelFrame(t, text="模型来源", padding=8)
        src.pack(fill="x", pady=(0, 6))

        # 模式 radio
        self.model_src_var = tk.StringVar(value="demo")
        ttk.Radiobutton(src, text="① 选 demo 编译 (models/demo/*)", variable=self.model_src_var,
                        value="demo", command=self._refresh_model_src).grid(
                            row=0, column=0, sticky="w", padx=4)
        ttk.Radiobutton(src, text="② 加载已编译 case (models/compiled/*)", variable=self.model_src_var,
                        value="case", command=self._refresh_model_src).grid(
                            row=0, column=1, sticky="w", padx=4)
        ttk.Button(src, text="🔄 重扫 demo 目录", command=self._rescan_demos).grid(
                            row=0, column=2, sticky="w", padx=12)

        # demo 行 1: demo 下拉 + n_cores + case_name + 编译
        self.model_demo_frame = tk.Frame(src, bg=CL_BG)
        self.model_demo_frame.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        ttk.Label(self.model_demo_frame, text="demo:").pack(side="left")
        # 显示用 display_name, 内部用 name. demo_var 存 display_name 用
        self.demo_var = tk.StringVar()
        self.demo_combo = ttk.Combobox(self.model_demo_frame, textvariable=self.demo_var,
                                        width=46, state="readonly")
        self.demo_combo.pack(side="left", padx=4)
        self.demo_combo.bind("<<ComboboxSelected>>", lambda e: self._on_demo_selected())
        ttk.Label(self.model_demo_frame, text="n_cores:").pack(side="left", padx=(12, 0))
        self.n_cores_var = tk.StringVar(value="2")
        ttk.Spinbox(self.model_demo_frame, textvariable=self.n_cores_var, from_=1, to=4,
                    width=4, command=self._on_demo_selected).pack(side="left", padx=4)
        ttk.Label(self.model_demo_frame, text="case_name:").pack(side="left", padx=(12, 0))
        self.case_name_var = tk.StringVar(value="")
        ttk.Entry(self.model_demo_frame, textvariable=self.case_name_var, width=24).pack(side="left", padx=4)
        self.compile_btn = ttk.Button(self.model_demo_frame, text="▶ 编译",
                                       style="Accent.TButton", command=self._on_compile)
        self.compile_btn.pack(side="left", padx=8)

        # demo 行 2: 选中 demo manifest 摘要
        self.model_demo_info_frame = tk.Frame(src, bg=CL_BG)
        self.model_demo_info_frame.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        self.demo_info_text = tk.Text(self.model_demo_info_frame, height=5,
                                       bg=CL_PANEL, fg=CL_FG, font=("Consolas", 9),
                                       borderwidth=0, state="disabled")
        self.demo_info_text.pack(fill="x")

        self._populate_demo_combo()

        # case 行 (放 row=3, demo_info_frame 占 row=2)
        self.model_case_frame = tk.Frame(src, bg=CL_BG)
        self.model_case_frame.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        self.case_drop = tk.Label(self.model_case_frame,
                                   text=f"(拖入已编译 case 目录, 默认 {COMPILED_DIR})",
                                   bg="#181828", fg=CL_DIM, font=("Consolas", 9),
                                   anchor="w", padx=8, pady=10,
                                   relief="solid", borderwidth=1)
        self.case_drop.pack(fill="x", pady=(0, 4))
        if HAS_DND:
            self.case_drop.drop_target_register(DND_FILES)
            self.case_drop.dnd_bind("<<Drop>>", self._on_drop_case)
        row = tk.Frame(self.model_case_frame, bg=CL_BG); row.pack(fill="x")
        ttk.Button(row, text="Browse...", command=self._on_browse_case).pack(side="left", padx=2)
        ttk.Button(row, text="重新解析", command=self._reload_case).pack(side="left", padx=2)
        # 默认隐藏 (mode=demo)
        self.model_case_frame.grid_remove()

        # 中下部: 左侧 = case meta + layer 结构 (上下排); 右侧 = toolchain log (全高)
        main_split = ttk.PanedWindow(t, orient="horizontal")
        main_split.pack(fill="both", expand=True, pady=(6, 6))

        # 左侧
        left = tk.Frame(main_split, bg=CL_BG)
        main_split.add(left, weight=3)
        meta = ttk.LabelFrame(left, text="case meta 摘要", padding=6)
        meta.pack(fill="x", pady=(0, 6))
        self.model_meta = tk.Text(meta, height=4, bg=CL_PANEL, fg=CL_FG,
                                   font=("Consolas", 9), borderwidth=0, state="disabled")
        self.model_meta.pack(fill="x")

        ly = ttk.LabelFrame(left, text="layer 结构", padding=4)
        ly.pack(fill="both", expand=True)
        cols = ("L", "mode", "K", "stride", "h_in", "w_in", "c_in", "c_out",
                "h_out", "w_out", "ifb_off", "desc")
        self.layer_tree = ttk.Treeview(ly, columns=cols, show="headings", height=14)
        widths = {"L": 30, "mode": 40, "K": 30, "stride": 40, "h_in": 50, "w_in": 50,
                  "c_in": 50, "c_out": 50, "h_out": 50, "w_out": 50,
                  "ifb_off": 80, "desc": 50}
        for c in cols:
            self.layer_tree.heading(c, text=c)
            self.layer_tree.column(c, width=widths[c], anchor="center")
        self.layer_tree.pack(fill="both", expand=True)

        # 右侧 toolchain 编译输出 (整高跨 case meta + layer 表)
        comp = ttk.LabelFrame(main_split, text="toolchain 编译输出", padding=4)
        main_split.add(comp, weight=2)
        self.compile_log = LogPane(comp, height=20)
        self.compile_log.pack(fill="both", expand=True)

    def _refresh_model_src(self):
        if self.model_src_var.get() == "demo":
            self.model_demo_frame.grid()
            self.model_demo_info_frame.grid()
            self.model_case_frame.grid_remove()
        else:
            self.model_demo_frame.grid_remove()
            self.model_demo_info_frame.grid_remove()
            self.model_case_frame.grid()

    # ----- demo combo dynamic 扫描 -----
    def _populate_demo_combo(self):
        """根据 self.demos 重建 combo values + 默认选 resnet11 / 第一个."""
        if not HAS_YAML:
            self.demo_combo['values'] = ["(PyYAML 未装, pip install PyYAML)"]
            set_text(self.demo_info_text, "PyYAML 未安装, demo 扫描功能不可用")
            self.compile_btn.config(state="disabled")
            return
        if not self.demos:
            self.demo_combo['values'] = [f"(无 demo, 检查 {DEMO_DIR})"]
            self.demo_var.set(self.demo_combo['values'][0])
            set_text(self.demo_info_text, f"models/demo/ 下没找到任何 model.yaml.\n"
                                          f"参考 {os.path.join(MODELS_DIR, 'README.md')}")
            self.compile_btn.config(state="disabled")
            return
        # display "<name>  —  <display_name>" 字符串列表
        items = []
        for name, m in self.demos.items():
            disp = m.get('display_name', name)
            items.append(f"{name}  —  {disp}")
        self.demo_combo['values'] = items
        # 默认选 resnet11; 若不存在选第一项
        default_name = 'resnet11' if 'resnet11' in self.demos else next(iter(self.demos))
        for it in items:
            if it.startswith(default_name + ' '):
                self.demo_var.set(it); break
        else:
            self.demo_var.set(items[0])
        self.compile_btn.config(state="normal")
        self._on_demo_selected()

    def _rescan_demos(self):
        """重新扫描 demo/ 目录 + 刷新 combo."""
        self.demos = scan_demos()
        self._populate_demo_combo()
        self.log.ok(f"重扫 demo 目录: 找到 {len(self.demos)} 个 demo")

    def _selected_demo_name(self) -> str | None:
        """combo 当前选项字符串 → 内部 name."""
        s = self.demo_var.get()
        if not s or '—' not in s: return None
        return s.split('—', 1)[0].strip()

    def _on_demo_selected(self):
        name = self._selected_demo_name()
        if not name or name not in self.demos:
            return
        m = self.demos[name]
        typ = m.get('type', '?')
        chain = m.get('chain', '—')
        # 切换 demo 时, 用 manifest 的 default_n_cores 重置 spin (注意: 只在 demo 变化时
        # 重置, 用户在 spin 里改 n 然后调本函数 (Spinbox command) 不该被覆盖).
        if hasattr(self, '_last_demo_selected') and self._last_demo_selected != name:
            if 'default_n_cores' in m:
                self.n_cores_var.set(str(m['default_n_cores']))
        self._last_demo_selected = name
        try: n = int(self.n_cores_var.get())
        except ValueError: n = m.get('default_n_cores', 2)
        # 自动填 case_name
        self.case_name_var.set(f"{name}_n{n}")
        # 显示 manifest 摘要
        lines = [
            f"name        : {name}    type: {typ}",
            f"display     : {m.get('display_name', '—')}",
        ]
        if typ == 'synthetic_chain':
            lines.append(f"chain       : {chain}  (run_multicore_chain.py --demo {chain})")
        elif typ == 'pytorch':
            ckpt = m.get('checkpoint', '—')
            ckpt_full = os.path.join(m['_dir'], ckpt) if ckpt != '—' and not os.path.isabs(ckpt) else ckpt
            exists = "OK" if os.path.isfile(ckpt_full) else "MISSING!"
            lines.append(f"checkpoint  : {ckpt}  [{exists}]")
            calib = m.get('calib', {}).get('images', '—')
            lines.append(f"calib       : {calib}")
            lines.append("※ pytorch 入口待 Step D 接入 toolchain")
        cn = m.get('class_names') or []
        if cn:
            cn_s = ', '.join(map(str, cn[:8])) + ('…' if len(cn) > 8 else '')
            lines.append(f"classes     : {len(cn)} 类  ({cn_s})")
        notes = (m.get('notes') or '').strip()
        if notes:
            lines.append(f"notes       : {notes.splitlines()[0]}")
        set_text(self.demo_info_text, "\n".join(lines))

    # ----- compile demo (subprocess) -----
    def _on_compile(self):
        name = self._selected_demo_name()
        if not name or name not in self.demos:
            messagebox.showerror("非法 demo", "请先选一个 demo")
            return
        m = self.demos[name]
        typ = m.get('type', '?')
        try:
            n_cores = int(self.n_cores_var.get())
        except ValueError:
            messagebox.showerror("非法", "n_cores 必须是整数"); return
        case_name = self.case_name_var.get().strip() or f"{name}_n{n_cores}"
        if typ == 'synthetic_chain':
            demo = m.get('chain', name)
            worker_args = ('synthetic', demo, n_cores, case_name, None)
        elif typ == 'pytorch':
            ckpt_rel = m.get('checkpoint', 'weights.pt')
            ckpt = ckpt_rel if os.path.isabs(ckpt_rel) else os.path.join(m['_dir'], ckpt_rel)
            calib_rel = m.get('calib', {}).get('images', './calib/')
            calib = calib_rel if os.path.isabs(calib_rel) else os.path.join(m['_dir'], calib_rel)
            calib_limit = m.get('calib', {}).get('limit', 10)
            if not os.path.isfile(ckpt):
                messagebox.showerror("ckpt 缺", f"找不到 checkpoint: {ckpt}"); return
            if not os.path.isdir(calib):
                messagebox.showerror("calib 缺", f"找不到 calib 目录: {calib}"); return
            worker_args = ('pytorch', name, n_cores, case_name,
                           {'ckpt': ckpt, 'calib': calib, 'calib_limit': calib_limit})
        else:
            messagebox.showerror("未知 type", f"model.yaml type={typ} 不支持")
            return
        self.compile_btn.config(state="disabled")
        self.compile_log.info(f"开始编译 type={typ} {worker_args[1]} n_cores={n_cores} "
                               f"→ models/compiled/{case_name}/ ...")
        self.status_var.set(f"编译中: {case_name}")
        threading.Thread(target=self._compile_worker,
                         args=worker_args, daemon=True).start()

    def _compile_worker(self, typ: str, demo_or_model: str, n_cores: int,
                         case_name: str, pt_spec: dict | None):
        """跑 toolchain 编 case.

        typ='synthetic': --demo <demo_or_model>
        typ='pytorch'  : --pt <ckpt> --model <demo_or_model> --calib <dir>

        toolchain 默认输出到 sim/tb_smc/cases/<name>/, 编译完 move 到 models/compiled/.
        """
        import subprocess, shutil
        env = os.environ.copy()
        env['FLUX_SMC_GLOBAL_BASE'] = '0x10000000'
        venv_py = os.path.join(_TOOLCHAIN_DIR, '.venv', 'Scripts', 'python.exe')
        if not os.path.isfile(venv_py):
            venv_py = sys.executable
        cmd = [venv_py, 'run_multicore_chain.py', '--smc',
               '--n_cores', str(n_cores),
               '--case_name', case_name]
        if typ == 'pytorch':
            cmd += ['--pt', pt_spec['ckpt'],
                    '--model', demo_or_model,
                    '--calib', pt_spec['calib'],
                    '--calib-limit', str(pt_spec['calib_limit'])]
        else:
            cmd += ['--demo', demo_or_model]
        try:
            t0 = time.time()
            proc = subprocess.Popen(cmd, cwd=_TOOLCHAIN_DIR, env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, encoding='utf-8', errors='replace',
                                     bufsize=1)
            for line in proc.stdout:
                self.msg_q.put(("compile_line", line.rstrip()))
            rc = proc.wait()
            dt = time.time() - t0
            if rc != 0:
                self.msg_q.put(("compile_err", f"exit {rc} (耗时 {dt:.1f}s)"))
                return
            # 移动 case 到 models/compiled/
            src_dir = os.path.join(_PROJECT_ROOT, 'sim', 'tb_smc', 'cases', case_name)
            dst_dir = os.path.join(COMPILED_DIR, case_name)
            if os.path.isdir(dst_dir):
                shutil.rmtree(dst_dir)
            os.makedirs(COMPILED_DIR, exist_ok=True)
            shutil.move(src_dir, dst_dir)
            self.msg_q.put(("compile_line", f"[gui] moved → {dst_dir}"))
            self.msg_q.put(("compile_done", dst_dir, dt))
        except Exception as e:
            self.msg_q.put(("compile_err", str(e)))

    # ----- case 加载 (沿用 deploy_smc_case) -----
    def _on_drop_case(self, event):
        p = parse_dnd_path(event.data)
        if p and os.path.isdir(p):
            self._load_case_dir(p)
        else:
            self.log.err(f"拖入的不是目录: {p}")

    def _on_browse_case(self):
        d = filedialog.askdirectory(
            title="选 case 目录",
            initialdir=COMPILED_DIR if os.path.isdir(COMPILED_DIR) else _PROJECT_ROOT)
        if d:
            self._load_case_dir(d)

    def _reload_case(self):
        if self.state.case_dir:
            self._load_case_dir(self.state.case_dir)

    def _load_case_dir(self, path: str):
        try:
            meta = dsc.parse_meta(path)
            gb = dsc.derive_smc_global_base(meta)
        except Exception as e:
            self.log.err(f"加载 case 失败: {e}")
            messagebox.showerror("加载失败", f"{path}\n{e}")
            return
        self.state.case_dir = path
        self.state.case_meta = meta
        self.state.global_base = gb
        self.state.deployed = False
        n_layers = meta['NUM_LAYERS']
        n_cores  = meta['NUM_CORES']
        self.case_drop.config(text=f"✓ {path}", fg=CL_OK)
        last_l = n_layers - 1
        meta_lines = [
            f"路径: {path}",
            f"NUM_CORES={n_cores}   NUM_LAYERS={n_layers}   "
            f"SMC_GLOBAL_BASE=0x{gb:08x}",
            f"final OFM: {meta.get(f'LAYER_{last_l}_H_OUT')}×"
            f"{meta.get(f'LAYER_{last_l}_W_OUT')}×{meta.get(f'LAYER_{last_l}_C_OUT')}",
        ]
        set_text(self.model_meta, "\n".join(meta_lines))

        # layer 表
        self.layer_tree.delete(*self.layer_tree.get_children())
        for l in range(n_layers):
            mode = meta.get(f'SMC_LAYER_{l}_MODE', '?')
            k = meta.get(f'SMC_LAYER_{l}_K', '-')
            stride = meta.get(f'SMC_LAYER_{l}_STRIDE', '-')
            h_in = meta.get(f'SMC_LAYER_{l}_H_IN', '-')
            w_in = meta.get(f'SMC_LAYER_{l}_W_IN', '-')
            cin_slices = meta.get(f'SMC_LAYER_{l}_C_IN_SLICES', 0)
            c_in = (cin_slices * 16) if isinstance(cin_slices, int) else '-'
            c_out = meta.get(f'LAYER_{l}_C_OUT', '-')
            h_out = meta.get(f'LAYER_{l}_H_OUT', '-')
            w_out = meta.get(f'LAYER_{l}_W_OUT', '-')
            ifb_off = meta.get(f'SMC_LAYER_{l}_IFB_OFFSET', -1)
            ifb_off_s = f"0x{ifb_off:x}" if isinstance(ifb_off, int) and ifb_off >= 0 else "—"
            desc = max((meta.get(f'CORE_{c}_LAYER_{l}_DESC_COUNT', 0)
                        for c in range(n_cores)), default=0)
            self.layer_tree.insert("", "end", values=(
                l, mode, k, stride, h_in, w_in, c_in, c_out, h_out, w_out, ifb_off_s, desc))

        # 尝试关联 manifest: case_name (path basename) → demo name 推断
        # 约定: case_name 为 "<demo>_n<N>" 或 "<demo>"; 去掉 _nN 后缀查 demos
        case_name = os.path.basename(path.rstrip(os.sep))
        demo_guess = case_name.rsplit('_n', 1)[0] if '_n' in case_name else case_name
        if hasattr(self, 'demos') and demo_guess in self.demos:
            self.state.current_manifest = self.demos[demo_guess]
            self.log.info(f"  关联 manifest: {demo_guess} (type={self.state.current_manifest.get('type')})")
        else:
            self.state.current_manifest = None

        self.log.ok(f"case 加载: {os.path.basename(path)} ({n_layers} layers, {n_cores} cores)")
        self.state.notify()

    # =========================================================================
    # Tab ③ 部署 - per-region 装载 + SMC 布局可视化
    # =========================================================================
    REGIONS = [
        ("IFB",      "ifm_ofm_base",  CL_OK),
        ("WB",       "wb_base",       CL_BAR),
        ("RDMA",     "rdma_base",     CL_WARN),
        ("DESC",     "desc_base",     CL_ACCENT),
        ("IDMA cmd", "idma_cmd_base", "#cba6f7"),
        ("ODMA cmd", "odma_cmd_base", "#f5c2e7"),
        ("INPUT",    "input_base",    "#94e2d5"),
        ("OFM",      "final_ofm_base","#fab387"),
    ]

    def _build_tab_deploy(self):
        t = self.tab_deploy

        # 顶部: 按钮 + 总进度
        top = ttk.LabelFrame(t, text="部署控制", padding=8)
        top.pack(fill="x", pady=(0, 6))
        row = tk.Frame(top, bg=CL_BG); row.pack(fill="x")
        self.deploy_btn = ttk.Button(row, text="▶ 一键部署 (全部 region)",
                                      style="Accent.TButton",
                                      command=self._on_deploy, state="disabled")
        self.deploy_btn.pack(side="left", padx=2)
        self.redeploy_btn = ttk.Button(row, text="只重传 IFM/输入 (root layer IFB)",
                                        command=self._on_redeploy_ifm, state="disabled")
        self.redeploy_btn.pack(side="left", padx=2)
        # 总进度
        self.deploy_progress = ttk.Progressbar(top, mode="determinate",
                                                style="Horizontal.TProgressbar")
        self.deploy_progress.pack(fill="x", pady=(8, 4))
        self.deploy_stat = tk.Label(top, text="未部署", bg=CL_BG, fg=CL_DIM,
                                     font=("Consolas", 9))
        self.deploy_stat.pack(fill="x")

        # 中部: per-region 进度 (8 个 region × 进度条)
        rg = ttk.LabelFrame(t, text="Region 部署进度", padding=6)
        rg.pack(fill="x", pady=6)
        self.region_bars = {}
        self.region_stats = {}
        for i, (name, _key, color) in enumerate(self.REGIONS):
            r = tk.Frame(rg, bg=CL_BG); r.pack(fill="x", pady=1)
            lab = tk.Label(r, text=name, bg=CL_BG, fg=color,
                           font=("Consolas", 9, "bold"), width=10, anchor="w")
            lab.pack(side="left")
            bar = ttk.Progressbar(r, mode="determinate", length=400)
            bar.pack(side="left", fill="x", expand=True, padx=4)
            stat = tk.Label(r, text="—", bg=CL_BG, fg=CL_DIM,
                            font=("Consolas", 8), width=24, anchor="w")
            stat.pack(side="left", padx=4)
            self.region_bars[name] = bar
            self.region_stats[name] = stat

        # 下部: SMC layout 可视化 (canvas)
        viz = ttk.LabelFrame(t, text="SMC 布局 (per-mem 内的 region 分布)", padding=6)
        viz.pack(fill="both", expand=True)
        self.layout_canvas = tk.Canvas(viz, bg="#11111b", height=140,
                                        borderwidth=0, highlightthickness=0)
        self.layout_canvas.pack(fill="both", expand=True)
        self.layout_canvas.bind("<Configure>", lambda e: self._draw_smc_layout())
        # 注册 hw_cfg / case_meta 改变时重画
        self.state.on_change(self._draw_smc_layout)
        self.state.on_change(self._refresh_deploy_buttons)

    def _refresh_deploy_buttons(self):
        ok = bool(self.state.rpc and self.state.case_dir and self.state.case_meta)
        self.deploy_btn.config(state="normal" if ok else "disabled")
        self.redeploy_btn.config(state="normal" if (ok and self.state.deployed) else "disabled")

    def _draw_smc_layout(self):
        c = self.layout_canvas
        c.delete("all")
        w = c.winfo_width(); h = c.winfo_height()
        if w < 50 or h < 30:
            return
        cfg = self.state.hw_cfg
        if cfg is None:
            c.create_text(w//2, h//2, text="(先加载 hw json)",
                          fill=CL_DIM, font=("Arial", 10))
            return
        r = cfg.deployment.regions
        stride = cfg.deployment.smc_mem_stride
        items = [(name, getattr(r, key), color) for name, key, color in self.REGIONS]
        items_sorted = sorted(items, key=lambda x: x[1])
        # 加 end (stride) sentinel
        items_sorted.append(("(end)", stride, CL_DIM))

        margin = 8
        bar_y = 50
        bar_h = 30
        usable_w = w - 2 * margin
        for i in range(len(items_sorted) - 1):
            name, base, color = items_sorted[i]
            _, next_base, _ = items_sorted[i + 1]
            x0 = margin + int(usable_w * base / stride)
            x1 = margin + int(usable_w * next_base / stride)
            if x1 - x0 < 2:
                x1 = x0 + 2
            c.create_rectangle(x0, bar_y, x1, bar_y + bar_h, fill=color, outline=CL_BG)
            # 标签 (上方)
            mid = (x0 + x1) / 2
            c.create_text(mid, bar_y - 6, text=name, fill=color,
                          font=("Arial", 8, "bold"), anchor="s")
            # 起点地址 (下方)
            c.create_text(x0, bar_y + bar_h + 8, text=f"0x{base:06x}",
                          fill=CL_DIM, font=("Consolas", 7), anchor="nw")
        # 总轴
        c.create_text(margin, bar_y + bar_h + 28, text="0",
                      fill=CL_DIM, font=("Consolas", 8), anchor="nw")
        c.create_text(w - margin, bar_y + bar_h + 28,
                      text=f"stride 0x{stride:x}",
                      fill=CL_DIM, font=("Consolas", 8), anchor="ne")
        # 标题
        c.create_text(margin, 10,
                      text=f"NUM_CORES={cfg.deployment.num_cores}  "
                           f"per-mem stride=0x{stride:x}  "
                           f"DDR base=0x{cfg.deployment.ddr_base:x}",
                      fill=CL_FG, font=("Consolas", 9, "bold"), anchor="nw")

    # ----- deploy worker -----
    def _on_deploy(self):
        if not (self.state.rpc and self.state.case_dir and self.state.case_meta):
            return
        self.deploy_btn.config(state="disabled")
        self.redeploy_btn.config(state="disabled")
        # reset bars
        for name in self.region_bars:
            self.region_bars[name]["value"] = 0
            self.region_stats[name].config(text="—", fg=CL_DIM)
        threading.Thread(target=self._deploy_worker, daemon=True).start()

    def _deploy_worker(self):
        try:
            meta = self.state.case_meta
            case_dir = self.state.case_dir
            gb = self.state.global_base
            n_layers = meta['NUM_LAYERS']
            self.msg_q.put(("deploy_start", n_layers))
            t0 = time.time()
            # 各 region 单独计时 (按 layer 累计, 给 per-region 进度)
            timings = {k: 0.0 for k in ["IFB", "WB", "RDMA", "DESC", "IDMA cmd", "ODMA cmd"]}
            for l in range(n_layers):
                is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
                if is_root:
                    ts = time.time()
                    dsc.load_ifb_smc(self.state.rpc, case_dir, meta, l, gb)
                    timings["IFB"] += time.time() - ts
                    self.msg_q.put(("deploy_region", "IFB", l + 1, n_layers, timings["IFB"]))
                ts = time.time()
                dsc.load_wb_smc(self.state.rpc, case_dir, meta, l, gb)
                timings["WB"] += time.time() - ts
                self.msg_q.put(("deploy_region", "WB", l + 1, n_layers, timings["WB"]))
                ts = time.time()
                dsc.load_rdma_smc(self.state.rpc, case_dir, meta, l, gb)
                timings["RDMA"] += time.time() - ts
                self.msg_q.put(("deploy_region", "RDMA", l + 1, n_layers, timings["RDMA"]))
                ts = time.time()
                dsc.load_desc_smc(self.state.rpc, case_dir, meta, l)
                timings["DESC"] += time.time() - ts
                self.msg_q.put(("deploy_region", "DESC", l + 1, n_layers, timings["DESC"]))
                ts = time.time()
                dsc.load_sg_cmd_smc(self.state.rpc, case_dir, meta, l)
                # SG cmd 同时给 IDMA + ODMA region
                dt = time.time() - ts
                timings["IDMA cmd"] += dt / 2
                timings["ODMA cmd"] += dt / 2
                self.msg_q.put(("deploy_region", "IDMA cmd", l + 1, n_layers, timings["IDMA cmd"]))
                self.msg_q.put(("deploy_region", "ODMA cmd", l + 1, n_layers, timings["ODMA cmd"]))
                self.msg_q.put(("deploy_progress", l + 1, n_layers))
            self.msg_q.put(("deploy_done", time.time() - t0))
        except Exception as e:
            self.msg_q.put(("deploy_err", str(e)))

    def _on_redeploy_ifm(self):
        """重传所有 root layer IFB (覆盖图片更新后用)."""
        if not (self.state.rpc and self.state.case_meta):
            return
        self.redeploy_btn.config(state="disabled")
        threading.Thread(target=self._redeploy_ifm_worker, daemon=True).start()

    def _redeploy_ifm_worker(self):
        try:
            meta = self.state.case_meta
            n_layers = meta['NUM_LAYERS']
            roots = [l for l in range(n_layers)
                     if l == 0 or meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1]
            self.region_bars["IFB"]["value"] = 0
            self.region_bars["IFB"]["maximum"] = len(roots)
            t0 = time.time()
            for i, l in enumerate(roots):
                dsc.load_ifb_smc(self.state.rpc, self.state.case_dir, meta, l, self.state.global_base)
                self.msg_q.put(("deploy_region", "IFB", i + 1, len(roots), time.time() - t0))
            self.msg_q.put(("redeploy_ifm_done", time.time() - t0, roots))
        except Exception as e:
            self.msg_q.put(("deploy_err", str(e)))

    # =========================================================================
    # Tab ④ 推理 - 图片 + RUN + 校验 toggle + Top-5
    # =========================================================================
    def _build_tab_inference(self):
        t = self.tab_infer
        # 左右两栏: 左 (图片) | 右 (运行 + 结果)
        body = tk.Frame(t, bg=CL_BG); body.pack(fill="both", expand=True)
        left  = tk.Frame(body, bg=CL_BG); left.pack(side="left", fill="both", expand=True, padx=(0, 4))
        right = tk.Frame(body, bg=CL_BG); right.pack(side="left", fill="both", expand=True, padx=(4, 0))

        # ---- 左: 图片 ----
        img = ttk.LabelFrame(left, text="输入图片 (可选; 不传图用 case 自带 IFB)", padding=6)
        img.pack(fill="both", expand=True)
        self.img_drop = tk.Label(img, text="(拖入图片 / Browse / 清除)",
                                  bg="#181828", fg=CL_DIM, font=("Consolas", 9),
                                  anchor="w", padx=8, pady=10,
                                  relief="solid", borderwidth=1)
        self.img_drop.pack(fill="x", pady=(0, 4))
        if HAS_DND:
            self.img_drop.drop_target_register(DND_FILES)
            self.img_drop.dnd_bind("<<Drop>>", self._on_drop_image)
        row = tk.Frame(img, bg=CL_BG); row.pack(fill="x")
        ttk.Button(row, text="Browse...", command=self._on_browse_image).pack(side="left", padx=2)
        ttk.Button(row, text="清除",     command=self._clear_image).pack(side="left", padx=2)
        self.upload_btn = ttk.Button(row, text="↑ 上传到板",
                                      command=self._on_upload_image, state="disabled")
        self.upload_btn.pack(side="right", padx=2)

        prev = ttk.LabelFrame(left, text="缩略图 (resize 目标按当前 case layer 0 反推)", padding=4)
        prev.pack(fill="both", expand=True, pady=4)
        self.img_canvas = tk.Canvas(prev, width=360, height=360, bg="#11111b",
                                     borderwidth=0, highlightthickness=0)
        self.img_canvas.pack(expand=True)
        self.img_canvas.create_text(180, 180, text="(无图)", fill=CL_DIM, font=("Arial", 12))

        info = ttk.LabelFrame(left, text="量化/重排信息", padding=6)
        info.pack(fill="x")
        self.img_info = tk.Text(info, height=4, bg=CL_PANEL, fg=CL_FG,
                                 font=("Consolas", 9), borderwidth=0, state="disabled")
        self.img_info.pack(fill="x")

        # ---- 右: 运行 + 结果 ----
        run = ttk.LabelFrame(right, text="推理控制", padding=6)
        run.pack(fill="x")
        row = tk.Frame(run, bg=CL_BG); row.pack(fill="x")
        ttk.Label(row, text="迭代次数:").pack(side="left")
        self.iters_var = tk.StringVar(value="10")
        ttk.Entry(row, textvariable=self.iters_var, width=6).pack(side="left", padx=4)
        ttk.Label(row, text="校验模式:").pack(side="left", padx=(12, 0))
        self.verify_mode_var = tk.StringVar(value="final")
        ttk.Radiobutton(row, text="仅终层", variable=self.verify_mode_var,
                        value="final").pack(side="left", padx=2)
        ttk.Radiobutton(row, text="逐 layer", variable=self.verify_mode_var,
                        value="per_layer").pack(side="left", padx=2)
        ttk.Radiobutton(row, text="不校验", variable=self.verify_mode_var,
                        value="none").pack(side="left", padx=2)

        btn_row = tk.Frame(run, bg=CL_BG); btn_row.pack(fill="x", pady=(8, 0))
        self.infer_btn = ttk.Button(btn_row, text="▶ 推理 (RUN_LAYERS)",
                                     style="Accent.TButton",
                                     command=self._on_infer, state="disabled")
        self.infer_btn.pack(side="left", padx=2, fill="x", expand=True)
        self.allinone_btn = ttk.Button(btn_row, text="一键 (上传图 → 推理)",
                                        style="OK.TButton",
                                        command=self._on_allinone, state="disabled")
        self.allinone_btn.pack(side="left", padx=2, fill="x", expand=True)

        self.infer_progress = ttk.Progressbar(run, mode="determinate")
        self.infer_progress.pack(fill="x", pady=4)

        # KPI
        perf = ttk.LabelFrame(right, text="本次推理性能", padding=6)
        perf.pack(fill="x", pady=6)
        kpi_row = tk.Frame(perf, bg=CL_BG); kpi_row.pack(fill="x")
        self.kpi_cy  = self._make_kpi(kpi_row, "HW cycles", CL_BAR)
        self.kpi_ms  = self._make_kpi(kpi_row, "HW time (ms)", CL_OK)
        self.kpi_fps = self._make_kpi(kpi_row, "HW FPS", CL_ACCENT)
        self.kpi_rtt = self._make_kpi(kpi_row, "RTT (ms)", CL_WARN)

        # 校验结果
        ver = ttk.LabelFrame(right, text="OFM 校验", padding=6)
        ver.pack(fill="x", pady=4)
        self.verify_text = tk.Text(ver, height=6, bg=CL_PANEL, fg=CL_FG,
                                    font=("Consolas", 9), borderwidth=0, state="disabled")
        self.verify_text.pack(fill="x")

        # Top-5
        top5 = ttk.LabelFrame(right, text="Top-5 logit (终层 int8)", padding=6)
        top5.pack(fill="both", expand=True, pady=4)
        self.top5_text = tk.Text(top5, bg=CL_PANEL, fg=CL_FG,
                                  font=("Consolas", 11), borderwidth=0, state="disabled")
        self.top5_text.pack(fill="both", expand=True)

        # state 变化时刷新按钮
        self.state.on_change(self._refresh_infer_buttons)

    def _make_kpi(self, parent, label, color):
        f = tk.Frame(parent, bg=CL_PANEL, padx=8, pady=4)
        f.pack(side="left", fill="x", expand=True, padx=2)
        tk.Label(f, text=label, bg=CL_PANEL, fg=CL_DIM,
                 font=("Arial", 8)).pack(anchor="w")
        v = tk.Label(f, text="—", bg=CL_PANEL, fg=color,
                     font=("Consolas", 16, "bold"))
        v.pack(anchor="w")
        return v

    def _refresh_infer_buttons(self):
        infer_ok = bool(self.state.rpc and self.state.deployed)
        upload_ok = bool(self.state.rpc and self.state.image_bytes_s2d and self.state.case_meta)
        allinone_ok = bool(self.state.rpc and self.state.deployed and self.state.image_bytes_s2d)
        self.infer_btn.config(state="normal" if infer_ok else "disabled")
        self.upload_btn.config(state="normal" if upload_ok else "disabled")
        self.allinone_btn.config(state="normal" if allinone_ok else "disabled")

    # ---- 图片处理 ----
    def _on_drop_image(self, event):
        p = parse_dnd_path(event.data)
        if p and os.path.isfile(p):
            self._load_image(p)
        else:
            self.log.err(f"拖入的不是文件: {p}")

    def _on_browse_image(self):
        p = filedialog.askopenfilename(
            title="选图片", filetypes=[("Image", "*.png *.jpg *.jpeg *.bmp"), ("All", "*.*")])
        if p:
            self._load_image(p)

    def _clear_image(self):
        self.state.image_path = None
        self.state.image_bytes_s2d = None
        self.state.image_uploaded = False
        self.img_canvas.delete("all")
        self.img_canvas.create_text(180, 180, text="(无图)", fill=CL_DIM, font=("Arial", 12))
        self.img_drop.config(text="(拖入图片 / Browse / 清除)", fg=CL_DIM)
        set_text(self.img_info, "")
        self.state.notify()

    def _load_image(self, path: str):
        try:
            from PIL import Image, ImageTk
        except ImportError:
            messagebox.showerror("缺依赖", "需要 Pillow:\npip install Pillow")
            return
        if not self.state.case_meta:
            messagebox.showinfo("先加载 case", "先在 ② 模型 tab 加载 case 才能确定图片预处理参数")
            return
        try:
            spec = derive_input_spec(self.state.case_meta, self.state.current_manifest)
        except Exception as e:
            self.log.err(f"反推 input spec 失败: {e}")
            messagebox.showerror("失败", f"反推 input spec 失败: {e}")
            return
        if spec['pil_mode'] is None:
            messagebox.showerror("不支持的通道数",
                f"layer 0 反推得 cin_real={spec['cin_real']}, 但只支持 1/3/4 (L/RGB/RGBA)")
            return
        try:
            t0 = time.time()
            # 1) 预览缩略图 (按反推 H_orig×W_orig resize)
            img_pil = Image.open(path).convert(spec['pil_mode'])
            preview = img_pil.resize((spec['w_orig'], spec['h_orig']), Image.BILINEAR)
            # PIL 'L' grayscale 也能 ImageTk 化, 不需要 RGB convert
            preview_thumb = preview.copy(); preview_thumb.thumbnail((360, 360))
            self._img_tk = ImageTk.PhotoImage(preview_thumb)
            self.img_canvas.delete("all")
            self.img_canvas.create_image(180, 180, image=self._img_tk)
            # 2) IFM byte stream — 走 manifest preprocess (含 toolchain s_x 用 quant)
            input_scale = self.state.case_meta.get('SMC_LAYER_0_INPUT_SCALE') \
                          if self.state.case_meta else None
            if input_scale is not None:
                try: input_scale = float(input_scale)
                except (ValueError, TypeError): input_scale = None
            ifm_orig, ifm_board, orig_size = load_image_to_ifm(
                path, spec, manifest=self.state.current_manifest, input_scale=input_scale)
            t_dt = time.time() - t0
            self.state.image_path = path
            self.state.image_bytes_s2d = ifm_board   # 兼容老命名 (实际可能不是 s2d)
            self.state.image_uploaded = False
            self.img_drop.config(text=f"✓ {os.path.basename(path)}", fg=CL_OK)
            info_lines = [
                f"路径: {path}",
                f"原图: H={orig_size[1]} W={orig_size[0]} mode={spec['pil_mode']}",
                f"resize: H={spec['h_orig']} W={spec['w_orig']} Cin={spec['cin_real']} = {len(ifm_orig)/1024:.0f} KB",
            ]
            if spec['is_patch']:
                info_lines.append(
                    f"s2d (K={spec['k']}): H={spec['h_board']} W={spec['w_board']} "
                    f"Cin_padded={spec['cin_padded']} = {len(ifm_board)/1024:.0f} KB")
            else:
                info_lines.append(f"无 s2d (layer 0 K={spec['k']}, stride={spec['stride']}, 直接灌)")
            info_lines.append(f"耗时: {t_dt*1000:.0f}ms")
            set_text(self.img_info, "\n".join(info_lines))
            self.log.ok(f"图片加载: {os.path.basename(path)} ({t_dt*1000:.0f}ms)")
            self.state.notify()
        except Exception as e:
            self.log.err(f"加载图失败: {e}")
            messagebox.showerror("加载图失败", str(e))

    def _on_upload_image(self):
        if not (self.state.rpc and self.state.image_bytes_s2d and self.state.case_meta):
            return
        self.upload_btn.config(state="disabled")
        threading.Thread(target=self._upload_worker, daemon=True).start()

    def _upload_worker(self):
        try:
            t0 = time.time()
            dsc._load_ifb_override(self.state.rpc, self.state.image_bytes_s2d,
                                    self.state.case_meta, self.state.global_base)
            self.msg_q.put(("upload_done", time.time() - t0, len(self.state.image_bytes_s2d)))
        except Exception as e:
            self.msg_q.put(("upload_err", str(e)))

    # ---- 推理 ----
    def _on_infer(self):
        if not (self.state.rpc and self.state.deployed and self.state.case_meta):
            return
        try:
            n = max(1, int(self.iters_var.get()))
        except ValueError:
            messagebox.showerror("非法", "迭代次数必须是正整数"); return
        verify_mode = self.verify_mode_var.get()
        # 如果用户加载了图片, "推理" 也按"上传图"模式跑 (worker 内部会按 user image
        # 重传 IFM, 跟 "一键" 等价但跳过初始 upload). 用户期望: 拖图 → 推理 → Top5
        # 是上传图的结果, 不是 case 自带 calib 图的结果.
        upload_first = bool(self.state.image_bytes_s2d)
        self.infer_btn.config(state="disabled")
        self.allinone_btn.config(state="disabled")
        self.infer_progress["maximum"] = n
        self.infer_progress["value"] = 0
        if upload_first:
            self.log.info("推理: 检测到已加载图片, 用上传图 (无校验)")
        threading.Thread(target=self._infer_worker, args=(n, verify_mode, upload_first),
                         daemon=True).start()

    def _on_allinone(self):
        if not (self.state.rpc and self.state.deployed and self.state.image_bytes_s2d):
            return
        try:
            n = max(1, int(self.iters_var.get()))
        except ValueError:
            n = 1
        verify_mode = self.verify_mode_var.get()
        self.infer_btn.config(state="disabled")
        self.allinone_btn.config(state="disabled")
        self.upload_btn.config(state="disabled")
        self.infer_progress["maximum"] = n
        self.infer_progress["value"] = 0
        threading.Thread(target=self._infer_worker, args=(n, verify_mode, True),
                         daemon=True).start()

    def _infer_worker(self, n: int, verify_mode: str, upload_first: bool):
        try:
            meta = self.state.case_meta
            n_layers = meta['NUM_LAYERS']
            last_l = n_layers - 1
            cfgs = dsc.build_layer_cfgs(meta)
            case_dir = self.state.case_dir
            gb = self.state.global_base

            # 上传图模式: expected_ofm.txt 是 toolchain 用 calib 第 1 张图算的,
            # 跟用户上传的图无关. 强制 disable verify (校验面板提示).
            if upload_first and verify_mode != "none":
                self.msg_q.put(("log", "WARN", "[infer] 上传图模式: 无 ground truth, 自动 disable 校验"))
                verify_mode = "none"

            # 准备 expected (按 verify_mode)
            expected_per = None
            if verify_mode == "per_layer":
                expected_per = [bytes(dsc.load_expected_ofm(case_dir, meta, l))
                                for l in range(n_layers)]
            elif verify_mode == "final":
                expected_per = {last_l: bytes(dsc.load_expected_ofm(case_dir, meta, last_l))}
            # OFM offsets (复用 stress_test 逻辑)
            try:
                from stress_test_resnet11 import _layer_ofm_offsets
                ofm_offsets = _layer_ofm_offsets(case_dir, n_layers, gb)
            except Exception:
                ofm_offsets = None

            cycles_list = []
            per_layer_pass = [0] * n_layers
            per_layer_fail = [0] * n_layers
            per_layer_first_fail = [None] * n_layers
            last_ofm = None

            # 找 root layers (每 iter 跑前重传, 保证 iter 0 跟 iter 1+ IFB 一致)
            roots = [l for l in range(n_layers)
                     if l == 0 or meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1]

            def _reload_ifb_for_iter():
                """每 iter 跑前调一次. upload_first 时 layer 0 用用户图, 其它 root layer
                跟非 upload 模式一样用 case 自带 (一般没其它 root). 这样 iter 间 IFB 严格一致."""
                for rl in roots:
                    if rl == 0 and upload_first:
                        dsc._load_ifb_override(self.state.rpc, self.state.image_bytes_s2d,
                                                meta, gb)
                    else:
                        dsc.load_ifb_smc(self.state.rpc, case_dir, meta, rl, gb)

            # 进 loop 前先发 upload_done UI 事件 (一键模式), worker 内部用 _reload_ifb_for_iter
            if upload_first:
                t_up = time.time()
                self.msg_q.put(("upload_done", 0.0, len(self.state.image_bytes_s2d)))

            for i in range(n):
                # 每 iter 都 reload IFB (iter 0 也 reload, 跟 iter 1+ 一致, 修 stress race)
                if roots:
                    _reload_ifb_for_iter()
                t0 = time.time()
                cy = self.state.rpc.run_layers(cfgs)
                rtt = time.time() - t0
                cycles_list.append(cy)
                # 校验
                if verify_mode == "none":
                    # 不做 bit-exact 校验, 但仍要 read 终层 OFM 给 Top-5 用 (上传图模式核心)
                    off = ofm_offsets[last_l] if ofm_offsets else None
                    last_ofm = dsc.read_ofm_smc(self.state.rpc, meta, last_l, gb, off) \
                        if off is not None else dsc.read_ofm_smc(self.state.rpc, meta, last_l, gb)
                    iter_status = "PASS (no-verify)"
                else:
                    layers_to_check = list(range(n_layers)) if verify_mode == "per_layer" else [last_l]
                    fails = []
                    for l in layers_to_check:
                        off = ofm_offsets[l] if ofm_offsets else None
                        got = dsc.read_ofm_smc(self.state.rpc, meta, l, gb, off) \
                            if off is not None else dsc.read_ofm_smc(self.state.rpc, meta, l, gb)
                        exp = expected_per[l] if isinstance(expected_per, list) else expected_per[l]
                        n_diff = sum(1 for a, b in zip(got, exp) if a != b)
                        if l == last_l:
                            last_ofm = got
                        if n_diff == 0:
                            per_layer_pass[l] += 1
                        else:
                            per_layer_fail[l] += 1
                            fails.append((l, n_diff, len(got)))
                            if per_layer_first_fail[l] is None:
                                fdb = next(idx for idx, (a, b) in enumerate(zip(got, exp)) if a != b)
                                per_layer_first_fail[l] = (i, n_diff, len(got), fdb,
                                                            got[fdb], exp[fdb])
                    iter_status = "PASS" if not fails else f"FAIL {fails}"
                self.msg_q.put(("infer_progress", i + 1, n, cy, rtt, iter_status))

            self.msg_q.put(("infer_done", cycles_list, per_layer_pass, per_layer_fail,
                             per_layer_first_fail, last_ofm, verify_mode))
        except Exception as e:
            import traceback; traceback.print_exc()
            self.msg_q.put(("infer_err", str(e)))

    # =========================================================================
    # UI 主线程消费 msg queue (跨线程通信)
    # =========================================================================
    def _poll_msgq(self):
        try:
            while True:
                msg = self.msg_q.get_nowait()
                self._handle_msg(msg)
        except queue.Empty:
            pass
        self.root.after(80, self._poll_msgq)

    def _handle_msg(self, msg: tuple):
        """各 tab module 通过 msg_q.put(("kind", *args)) 通信."""
        kind = msg[0]
        if kind == "log":
            level, text = msg[1], msg[2]
            self.log.log(level, text)
        elif kind == "status":
            self.status_var.set(msg[1])
        elif kind == "compile_line":
            self.compile_log.info(msg[1])
        elif kind == "compile_done":
            case_dir, dt = msg[1], msg[2]
            self.compile_log.ok(f"编译完成 ({dt:.1f}s) → {case_dir}")
            self.log.ok(f"toolchain 编译完成: {os.path.basename(case_dir)} ({dt:.1f}s)")
            self.status_var.set("编译完成")
            self.compile_btn.config(state="normal")
            # 自动切到 case 模式 + 加载
            self.model_src_var.set("case")
            self._refresh_model_src()
            self._load_case_dir(case_dir)
        elif kind == "compile_err":
            self.compile_log.err(f"编译失败: {msg[1]}")
            self.log.err(f"toolchain 编译失败: {msg[1]}")
            self.status_var.set("编译失败")
            self.compile_btn.config(state="normal")
        elif kind == "deploy_start":
            n_layers = msg[1]
            self.deploy_progress["maximum"] = n_layers
            self.deploy_progress["value"] = 0
            for name in self.region_bars:
                self.region_bars[name]["maximum"] = n_layers
            self.deploy_stat.config(text=f"部署中 0/{n_layers}", fg=CL_WARN)
            self.log.info(f"开始部署 {n_layers} 层 ...")
        elif kind == "deploy_region":
            name, done, total, t = msg[1], msg[2], msg[3], msg[4]
            self.region_bars[name]["value"] = done
            self.region_stats[name].config(text=f"{done}/{total} ({t*1000:.0f}ms)", fg=CL_OK)
        elif kind == "deploy_progress":
            i, n = msg[1], msg[2]
            self.deploy_progress["value"] = i
            self.deploy_stat.config(text=f"部署中 {i}/{n}", fg=CL_WARN)
            self.status_var.set(f"部署 {i}/{n}")
        elif kind == "deploy_done":
            dt = msg[1]
            self.state.deployed = True
            self.deploy_stat.config(text=f"✓ 部署完成 ({dt:.1f}s)", fg=CL_OK)
            self.log.ok(f"模型部署完成 ({dt:.1f}s)")
            self.status_var.set("已部署")
            self.deploy_btn.config(state="normal")
            self.state.notify()
        elif kind == "deploy_err":
            self.deploy_stat.config(text=f"✗ 失败: {msg[1]}", fg=CL_ERR)
            self.log.err(f"部署失败: {msg[1]}")
            self.deploy_btn.config(state="normal")
            self.redeploy_btn.config(state="normal" if self.state.deployed else "disabled")
        elif kind == "redeploy_ifm_done":
            dt, roots = msg[1], msg[2]
            self.log.ok(f"重传 IFM 完成 ({dt*1000:.0f}ms, root layers={roots})")
            self.redeploy_btn.config(state="normal")
        elif kind == "upload_done":
            dt, n = msg[1], msg[2]
            self.state.image_uploaded = True
            self.log.ok(f"图片上传完成 ({n/1024:.0f}KB, {dt*1000:.0f}ms)")
            self.upload_btn.config(state="normal")
            self.state.notify()
        elif kind == "upload_err":
            self.log.err(f"上传图失败: {msg[1]}")
            self.upload_btn.config(state="normal")
        elif kind == "infer_progress":
            i, n, cy, rtt, status = msg[1:6]
            self.infer_progress["value"] = i
            self.kpi_cy.config(text=f"{cy:,}")
            self.kpi_ms.config(text=f"{cy/100000:.2f}")
            self.kpi_fps.config(text=f"{100e6/cy:.0f}" if cy else "—")
            self.kpi_rtt.config(text=f"{rtt*1000:.0f}")
            color = CL_OK if status.startswith("PASS") else CL_ERR
            self.log.log("RPC", f"iter {i}/{n}: cy={cy} rtt={rtt*1000:.0f}ms {status}")
            # 把 cycles 推到性能 tab (Step 6 用)
            self.state.cycles_history.append(cy)
            self._push_perf_sample(cy)
        elif kind == "infer_done":
            cycles_list, pl_pass, pl_fail, pl_first_fail, last_ofm, mode = msg[1:7]
            self._fill_verify_text(mode, cycles_list, pl_pass, pl_fail, pl_first_fail)
            if last_ofm:
                self._show_top5(last_ofm)
            self.log.ok(f"推理完成 ({len(cycles_list)} iter, mode={mode})")
            self.state.per_layer_pass = pl_pass
            self.state.per_layer_fail = pl_fail
            self._refresh_infer_buttons()
            self._push_perf_summary(cycles_list)
            self.state.notify()
        elif kind == "infer_err":
            self.log.err(f"推理失败: {msg[1]}")
            self._refresh_infer_buttons()

    def _fill_verify_text(self, mode, cycles_list, pl_pass, pl_fail, pl_first_fail):
        if mode == "none":
            # 上传图模式: expected_ofm.txt 是 toolchain 用 calib 第 1 张图算的,
            # 跟用户上传的手写图无关. 没法 bit-exact 校验, 看 Top-5 即可.
            note = "" if not self.state.image_bytes_s2d \
                    else "  (上传图模式: 看 Top-5; expected.txt 是 calib 算的不可比)"
            set_text(self.verify_text, f"(不校验) 完成 {len(cycles_list)} iter{note}")
            return
        n_layers = len(pl_pass)
        if mode == "final":
            l = n_layers - 1
            total = pl_pass[l] + pl_fail[l]
            line = f"final 层 {l}: PASS {pl_pass[l]}/{total}, FAIL {pl_fail[l]}"
            if pl_first_fail[l]:
                it, nd, tot, fdb, g, e = pl_first_fail[l]
                line += f"  first_fail @ iter {it}: {nd}/{tot} byte (byte[{fdb}] got=0x{g:02x} exp=0x{e:02x})"
            set_text(self.verify_text, line)
            return
        # per_layer
        lines = []
        for l in range(n_layers):
            total = pl_pass[l] + pl_fail[l]
            if total == 0: continue
            status = "OK " if pl_fail[l] == 0 else "BAD"
            line = f"L{l:2d} {status} PASS={pl_pass[l]}/{total}"
            if pl_first_fail[l]:
                it, nd, tot, fdb, g, e = pl_first_fail[l]
                line += f"  fail@i{it} {nd}/{tot}B (b[{fdb}] {g:02x}≠{e:02x})"
            lines.append(line)
        set_text(self.verify_text, "\n".join(lines))

    def _show_top5(self, ofm: bytes):
        if not ofm:
            set_text(self.top5_text, "(无终层 OFM)")
            return
        n = min(len(ofm), 1024)
        logits = [int.from_bytes([b], byteorder='little', signed=True) for b in ofm[:n]]
        idx_val = sorted(enumerate(logits), key=lambda x: -x[1])[:5]
        lines = [f"#{r}  class[{idx:>3d}] = {val:>+4d}"
                 for r, (idx, val) in enumerate(idx_val, 1)]
        lines.append("")
        lines.append(f"注: 终层 OFM 共 {len(ofm)} byte, 取前 {n} byte 当 int8 logit.")
        set_text(self.top5_text, "\n".join(lines))

    # =========================================================================
    # Tab ⑤ 性能 - cycles 历史 + 分布 + 统计 + per-layer 表
    # =========================================================================
    def _build_tab_perf(self):
        t = self.tab_perf
        # 顶部: 统计 KPI
        kpi = ttk.LabelFrame(t, text="累计统计 (本次 GUI 启动以来全部 iter)", padding=8)
        kpi.pack(fill="x", pady=(0, 6))
        kpi_row = tk.Frame(kpi, bg=CL_BG); kpi_row.pack(fill="x")
        self.perf_kpi_n      = self._make_kpi(kpi_row, "iter 数",          CL_FG)
        self.perf_kpi_mean   = self._make_kpi(kpi_row, "mean cy",         CL_OK)
        self.perf_kpi_minmax = self._make_kpi(kpi_row, "min / max",       CL_ACCENT)
        self.perf_kpi_std    = self._make_kpi(kpi_row, "stddev",          CL_WARN)
        self.perf_kpi_jit    = self._make_kpi(kpi_row, "jitter (%)",      CL_ERR)
        self.perf_kpi_fps    = self._make_kpi(kpi_row, "mean FPS @100MHz", CL_BAR)

        # 中部: 两栏图
        body = tk.Frame(t, bg=CL_BG); body.pack(fill="both", expand=True, pady=4)

        hist = ttk.LabelFrame(body, text="cycles 历史曲线", padding=4)
        hist.pack(side="left", fill="both", expand=True, padx=(0, 4))
        self.perf_hist_canvas = tk.Canvas(hist, bg="#11111b", height=240,
                                           borderwidth=0, highlightthickness=0)
        self.perf_hist_canvas.pack(fill="both", expand=True)
        self.perf_hist_canvas.bind("<Configure>", lambda e: self._redraw_perf_hist())

        distr = ttk.LabelFrame(body, text="cycles 分布直方图 (20 bin)", padding=4)
        distr.pack(side="left", fill="both", expand=True, padx=(4, 0))
        self.perf_distr_canvas = tk.Canvas(distr, bg="#11111b", height=240,
                                            borderwidth=0, highlightthickness=0)
        self.perf_distr_canvas.pack(fill="both", expand=True)
        self.perf_distr_canvas.bind("<Configure>", lambda e: self._redraw_perf_distr())

        # 下部: per-layer 校验表 (上次推理结果)
        pl = ttk.LabelFrame(t, text="上次 per-layer 校验", padding=4)
        pl.pack(fill="x", pady=(4, 0))
        cols = ("L", "checked", "pass", "fail", "first_fail")
        self.perf_layer_tree = ttk.Treeview(pl, columns=cols, show="headings", height=6)
        for c, w in [("L", 30), ("checked", 60), ("pass", 60), ("fail", 60), ("first_fail", 380)]:
            self.perf_layer_tree.heading(c, text=c)
            self.perf_layer_tree.column(c, width=w, anchor="w" if c == "first_fail" else "center")
        self.perf_layer_tree.pack(fill="x")

        # 控制栏
        ctrl = tk.Frame(t, bg=CL_BG); ctrl.pack(fill="x", pady=(4, 0))
        ttk.Button(ctrl, text="清空历史", command=self._clear_perf).pack(side="left", padx=2)
        ttk.Button(ctrl, text="导出 CSV...", command=self._export_perf_csv).pack(side="left", padx=2)

    def _push_perf_sample(self, cy: int):
        """每 iter 完成后调一次, 把 cy 加入 history + 重画."""
        self._redraw_perf_hist()
        self._redraw_perf_distr()
        self._refresh_perf_kpi()

    def _push_perf_summary(self, cycles_list: list):
        """整个推理结束后调, 刷新 per-layer 表."""
        # per-layer 表
        self.perf_layer_tree.delete(*self.perf_layer_tree.get_children())
        pl_pass = self.state.per_layer_pass
        pl_fail = self.state.per_layer_fail
        if pl_pass and pl_fail:
            for l in range(len(pl_pass)):
                total = pl_pass[l] + pl_fail[l]
                if total == 0:
                    continue
                ff = "—"
                # 注: 详细 first_fail 在 verify_text 里有, 这里只标 OK/BAD
                if pl_fail[l] > 0:
                    ff = f"{pl_fail[l]} fail (见推理 tab 校验面板)"
                self.perf_layer_tree.insert("", "end",
                    values=(l, total, pl_pass[l], pl_fail[l], ff))

    def _refresh_perf_kpi(self):
        h = self.state.cycles_history
        if not h:
            for kpi in (self.perf_kpi_n, self.perf_kpi_mean, self.perf_kpi_minmax,
                        self.perf_kpi_std, self.perf_kpi_jit, self.perf_kpi_fps):
                kpi.config(text="—")
            return
        n = len(h); mn = min(h); mx = max(h)
        mean = sum(h) / n
        var = sum((c - mean) ** 2 for c in h) / n
        std = var ** 0.5
        jit = (mx - mn) / mean * 100 if mean else 0
        self.perf_kpi_n.config(text=f"{n}")
        self.perf_kpi_mean.config(text=f"{mean:,.0f}")
        self.perf_kpi_minmax.config(text=f"{mn:,}/{mx:,}")
        self.perf_kpi_std.config(text=f"{std:.1f}")
        self.perf_kpi_jit.config(text=f"{jit:.3f}")
        self.perf_kpi_fps.config(text=f"{100e6/mean:.1f}" if mean else "—")

    def _redraw_perf_hist(self):
        c = self.perf_hist_canvas
        c.delete("all")
        w = c.winfo_width(); h = c.winfo_height()
        if w < 50 or h < 30: return
        hist = self.state.cycles_history
        if not hist:
            c.create_text(w//2, h//2, text="(无数据)", fill=CL_DIM, font=("Arial", 10))
            return
        # 坐标轴 + 折线
        margin = 24
        plot_w = w - 2 * margin
        plot_h = h - 2 * margin
        mn, mx = min(hist), max(hist)
        rng = max(mx - mn, 1)
        # 轴
        c.create_line(margin, h - margin, w - margin, h - margin, fill=CL_DIM)
        c.create_line(margin, margin, margin, h - margin, fill=CL_DIM)
        # 标签
        c.create_text(margin, margin - 8, text=f"max {mx:,}", fill=CL_DIM,
                       font=("Consolas", 8), anchor="sw")
        c.create_text(margin, h - margin + 12, text=f"min {mn:,}", fill=CL_DIM,
                       font=("Consolas", 8), anchor="nw")
        c.create_text(w - margin, h - margin + 12, text=f"n={len(hist)}", fill=CL_DIM,
                       font=("Consolas", 8), anchor="ne")
        # 折线
        n = len(hist)
        pts = []
        for i, v in enumerate(hist):
            x = margin + (i / max(n - 1, 1)) * plot_w
            y = h - margin - ((v - mn) / rng) * plot_h
            pts.extend((x, y))
        if len(pts) >= 4:
            c.create_line(*pts, fill=CL_BAR, width=2, smooth=True)
        # 点
        for i in range(0, n, max(1, n // 50)):
            x = margin + (i / max(n - 1, 1)) * plot_w
            y = h - margin - ((hist[i] - mn) / rng) * plot_h
            c.create_oval(x - 2, y - 2, x + 2, y + 2, fill=CL_ACCENT, outline="")

    def _redraw_perf_distr(self, n_bins: int = 20):
        c = self.perf_distr_canvas
        c.delete("all")
        w = c.winfo_width(); h = c.winfo_height()
        if w < 50 or h < 30: return
        hist = self.state.cycles_history
        if len(hist) < 2:
            c.create_text(w//2, h//2, text="(< 2 个样本)", fill=CL_DIM,
                          font=("Arial", 10))
            return
        mn, mx = min(hist), max(hist)
        rng = max(mx - mn, 1)
        bins = [0] * n_bins
        for v in hist:
            idx = min(n_bins - 1, int((v - mn) / rng * n_bins))
            bins[idx] += 1
        margin = 20
        plot_w = w - 2 * margin
        plot_h = h - 2 * margin
        bin_w = plot_w / n_bins
        max_count = max(bins)
        c.create_line(margin, h - margin, w - margin, h - margin, fill=CL_DIM)
        for i, cnt in enumerate(bins):
            if cnt == 0: continue
            x0 = margin + i * bin_w + 1
            x1 = margin + (i + 1) * bin_w - 1
            bar_h = (cnt / max_count) * plot_h
            y0 = h - margin - bar_h
            c.create_rectangle(x0, y0, x1, h - margin, fill=CL_BAR, outline="")
            if bar_h > 14:
                c.create_text((x0 + x1) / 2, y0 + 2, text=str(cnt),
                              fill="#11111b", font=("Consolas", 8, "bold"), anchor="n")
        c.create_text(margin, h - margin + 12, text=f"{mn:,}",
                      fill=CL_DIM, font=("Consolas", 8), anchor="nw")
        c.create_text(w - margin, h - margin + 12, text=f"{mx:,}",
                      fill=CL_DIM, font=("Consolas", 8), anchor="ne")

    def _clear_perf(self):
        self.state.cycles_history = []
        self.state.per_layer_pass = []
        self.state.per_layer_fail = []
        self.perf_layer_tree.delete(*self.perf_layer_tree.get_children())
        self._refresh_perf_kpi()
        self._redraw_perf_hist()
        self._redraw_perf_distr()
        self.log.info("性能历史已清空")

    def _export_perf_csv(self):
        if not self.state.cycles_history:
            messagebox.showinfo("无数据", "没有性能数据可导出"); return
        p = filedialog.asksaveasfilename(
            title="保存 cycles CSV", defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All", "*.*")])
        if not p: return
        try:
            with open(p, 'w', encoding='utf-8') as f:
                f.write("iter,cycles,hw_time_ms,fps@100MHz\n")
                for i, cy in enumerate(self.state.cycles_history):
                    f.write(f"{i},{cy},{cy/1e5:.4f},{1e8/cy:.2f}\n")
            self.log.ok(f"性能数据已导出: {p}")
        except Exception as e:
            self.log.err(f"导出失败: {e}")


# ---------------------------------------------------------------------------
def main():
    if HAS_DND:
        root = TkinterDnD.Tk()
    else:
        root = tk.Tk()
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    FluxCnnGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
