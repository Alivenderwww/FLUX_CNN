#!/usr/bin/env python3
"""
resnet11_gui.py — VD100 demo 上位机 GUI (新版, 通用 RPC)

跟 vd100_rpc.py + deploy_smc_case.py 配对. 板上跑通用 RPC server (vd100_rpc_server.c),
PC 端通过本 GUI:
  - 拖拽 / browse 加载模型 case 目录 (sim/tb_smc/cases/<name>/)
  - 拖拽 / browse 加载图片 (PNG/JPG)
  - 一键部署模型到板 (RPC LOAD_DDR 全部 region)
  - 一键推理 + 实时显示 STATUS / Top-5 / FPS / cycles
  - 实时 RPC 计数 + ping 延迟 + 滚动日志

依赖:
  pip install Pillow tkinterdnd2

启动:
  python resnet11_gui.py
"""
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# 拖拽支持 (可选, 没装 fallback 到只能 browse)
try:
    from tkinterdnd2 import TkinterDnD, DND_FILES
    HAS_DND = True
except ImportError:
    TkinterDnD = None
    DND_FILES = None
    HAS_DND = False

# 复用 RPC + deploy 模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vd100_rpc
import deploy_smc_case as dsc
from vd100_rpc import Vd100Rpc, RpcError, NUM_CORES, CSR_STATUS, CSR_CTRL


DEFAULT_CASE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'sim', 'tb_smc', 'cases', 'vd100_resnet11_n3'
)

# Color palette
CL_BG       = "#1e1e2e"
CL_PANEL    = "#2a2a3e"
CL_FG       = "#cdd6f4"
CL_DIM      = "#7f849c"
CL_OK       = "#a6e3a1"
CL_WARN     = "#f9e2af"
CL_ERR      = "#f38ba8"
CL_ACCENT   = "#89b4fa"
CL_BAR      = "#74c7ec"


# ---------------------------------------------------------------------------
# Helper widgets
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
    def __init__(self, master, **kw):
        super().__init__(master, bg=CL_PANEL, **kw)
        self.text = tk.Text(self, bg="#11111b", fg=CL_FG,
                            font=("Consolas", 9), height=8, wrap="none",
                            insertbackground=CL_FG, borderwidth=0)
        self.text.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(self, orient="vertical", command=self.text.yview)
        sb.pack(side="right", fill="y")
        self.text.configure(yscrollcommand=sb.set)
        # 等级 tag
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

    def info(self, msg): self.log("INFO", msg)
    def ok  (self, msg): self.log("OK",   msg)
    def warn(self, msg): self.log("WARN", msg)
    def err (self, msg): self.log("ERR",  msg)
    def rpc (self, msg): self.log("RPC",  msg)


class HistoryChart(tk.Canvas):
    """跑分历史图: 横向条形, 显示最近 N 次推理 cycles."""
    def __init__(self, master, max_points=20, **kw):
        super().__init__(master, bg="#11111b", height=80, borderwidth=0,
                         highlightthickness=0, **kw)
        self.max_points = max_points
        self.history = []
        self.bind("<Configure>", lambda e: self.redraw())

    def push(self, value: float):
        self.history.append(value)
        if len(self.history) > self.max_points:
            self.history.pop(0)
        self.redraw()

    def reset(self):
        self.history = []
        self.redraw()

    def redraw(self):
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 20 or h < 10 or not self.history:
            self.create_text(w//2, h//2, text="(历史数据空)",
                             fill=CL_DIM, font=("Arial", 9))
            return
        n = len(self.history)
        bar_w = max(2, (w - 20) / max(n, 1))
        vmax = max(self.history)
        vmin = min(self.history)
        rng = max(vmax - vmin, 1)
        for i, v in enumerate(self.history):
            x0 = 10 + i * bar_w
            x1 = x0 + bar_w * 0.85
            # 高度归一化 (顶/底 留 padding)
            ratio = 0.2 + 0.8 * (v - vmin) / rng if rng > 0 else 0.5
            bar_h = ratio * (h - 20)
            y0 = h - 5 - bar_h
            y1 = h - 5
            self.create_rectangle(x0, y0, x1, y1, fill=CL_BAR, outline="")
        # 顶/底 数值
        self.create_text(5, 5, text=f"max {vmax:.0f}",
                         fill=CL_DIM, font=("Consolas", 8), anchor="nw")
        self.create_text(5, h - 5, text=f"min {vmin:.0f}",
                         fill=CL_DIM, font=("Consolas", 8), anchor="sw")
        self.create_text(w - 5, h//2, text=f"n={n}",
                         fill=CL_DIM, font=("Consolas", 8), anchor="e")


class StatusBitGrid(tk.Frame):
    """显示一个 ConvCore 的 STATUS reg bit-by-bit (彩色方块)."""
    BITS = [
        (0,  "done"),   (1,  "lyr_b"), (2,  "idma_b"), (3,  "wdma_b"),
        (4,  "odma_b"), (5,  "idma_d"), (6,  "wdma_d"), (7,  "odma_d"),
        (8,  "dfe_b"),  (9,  "dfe_d"),  (11, "lyr_d"),
    ]

    def __init__(self, master, core_id: int, **kw):
        super().__init__(master, bg=CL_PANEL, **kw)
        self.core_id = core_id
        tk.Label(self, text=f"Core {core_id}", bg=CL_PANEL, fg=CL_FG,
                 font=("Arial", 9, "bold")).grid(row=0, column=0, sticky="w", padx=2)
        self.bit_labels = {}
        for col, (bit, name) in enumerate(self.BITS, 1):
            lab = tk.Label(self, text=name, bg="#181828", fg=CL_DIM,
                           font=("Consolas", 7), padx=2, pady=1, width=6)
            lab.grid(row=0, column=col, padx=1)
            self.bit_labels[bit] = lab
        self.raw = tk.Label(self, text="0x00000000", bg=CL_PANEL, fg=CL_DIM,
                            font=("Consolas", 8))
        self.raw.grid(row=0, column=len(self.BITS) + 1, padx=4)

    def update(self, status_val: int):
        self.raw.config(text=f"0x{status_val:08x}")
        for bit, lab in self.bit_labels.items():
            on = (status_val >> bit) & 1
            if on:
                color = CL_OK if bit in (0, 5, 6, 7, 9, 11) else CL_WARN
                lab.config(bg=color, fg="#11111b")
            else:
                lab.config(bg="#181828", fg=CL_DIM)


# ---------------------------------------------------------------------------
# Main GUI
# ---------------------------------------------------------------------------
class Vd100Gui:
    def __init__(self, root):
        self.root = root
        root.title("FLUX_CNN VD100 — Demo 控制台")
        root.geometry("1280x800")
        root.configure(bg=CL_BG)

        # State
        self.rpc: Vd100Rpc | None = None
        self.case_dir: str | None = None
        self.case_meta: dict | None = None
        self.global_base: int = 0
        self.image_path: str | None = None
        self.image_bytes_orig: bytes | None = None  # 540×960×4 NHWC
        self.image_bytes_s2d: bytes | None = None   # 240×135×64 NHWC
        self.deployed = False
        self.image_uploaded = False
        self.rpc_calls = 0
        self.rpc_bytes_out = 0
        self.rpc_bytes_in = 0
        self.msg_q = queue.Queue()
        self._poll_thread: threading.Thread | None = None
        self._poll_stop = threading.Event()
        self._ping_thread: threading.Thread | None = None
        self._ping_stop = threading.Event()

        # ttk 主题 (深色)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=CL_BG, foreground=CL_FG, fieldbackground=CL_PANEL)
        style.configure("TLabelframe", background=CL_BG, foreground=CL_FG, bordercolor=CL_DIM)
        style.configure("TLabelframe.Label", background=CL_BG, foreground=CL_ACCENT,
                        font=("Arial", 9, "bold"))
        style.configure("TFrame", background=CL_BG)
        style.configure("TLabel", background=CL_BG, foreground=CL_FG)
        style.configure("TButton", background=CL_PANEL, foreground=CL_FG, borderwidth=1)
        style.map("TButton", background=[("active", CL_ACCENT)],
                  foreground=[("active", "#11111b")])
        style.configure("Accent.TButton", background=CL_ACCENT, foreground="#11111b",
                        font=("Arial", 10, "bold"))
        style.map("Accent.TButton", background=[("active", "#b4befe")])
        style.configure("OK.TButton", background=CL_OK, foreground="#11111b",
                        font=("Arial", 10, "bold"))
        style.configure("Treeview", background=CL_PANEL, fieldbackground=CL_PANEL,
                        foreground=CL_FG, rowheight=22)
        style.configure("Treeview.Heading", background=CL_BG, foreground=CL_ACCENT,
                        font=("Arial", 9, "bold"))
        style.configure("TEntry", fieldbackground=CL_PANEL, foreground=CL_FG)
        style.configure("Horizontal.TProgressbar", background=CL_BAR, troughcolor=CL_PANEL)

        self._build_ui()
        self.root.after(80, self._poll_msgq)

        # 启动后默认尝试加载 default case (如果存在)
        if os.path.isdir(DEFAULT_CASE):
            self.root.after(200, lambda: self._load_case_dir(DEFAULT_CASE))

    # =========================================================================
    # UI 布局
    # =========================================================================
    def _build_ui(self):
        # ---- 顶部状态栏 ----
        top = tk.Frame(self.root, bg=CL_PANEL, height=44)
        top.pack(fill="x", side="top", padx=0, pady=0)
        top.pack_propagate(False)

        tk.Label(top, text="VD100", bg=CL_PANEL, fg=CL_ACCENT,
                 font=("Arial", 14, "bold")).pack(side="left", padx=(12, 4))
        tk.Label(top, text="ResNet11 N=3 Demo Console", bg=CL_PANEL, fg=CL_DIM,
                 font=("Arial", 9)).pack(side="left", padx=4)

        right_top = tk.Frame(top, bg=CL_PANEL)
        right_top.pack(side="right", padx=12)
        # 实时 RPC 统计
        self.stats_label = tk.Label(right_top, text="", bg=CL_PANEL, fg=CL_DIM,
                                     font=("Consolas", 9))
        self.stats_label.pack(side="right", padx=8)
        # ping 延迟
        self.ping_label = tk.Label(right_top, text="ping —", bg=CL_PANEL,
                                    fg=CL_DIM, font=("Consolas", 9))
        self.ping_label.pack(side="right", padx=8)
        # 连接 LED
        self.conn_led = StatusLED(right_top)
        self.conn_led.pack(side="right", padx=4)
        self.conn_text = tk.Label(right_top, text="未连接", bg=CL_PANEL, fg=CL_DIM,
                                   font=("Arial", 10, "bold"))
        self.conn_text.pack(side="right")

        # ---- 上方连接 panel ----
        conn_frame = ttk.LabelFrame(self.root, text="连接", padding=6)
        conn_frame.pack(fill="x", padx=8, pady=(8, 4))

        ttk.Label(conn_frame, text="VD100 IP:").grid(row=0, column=0, sticky="w")
        self.ip_var = tk.StringVar(value="169.254.111.10")
        ttk.Entry(conn_frame, textvariable=self.ip_var, width=18).grid(
            row=0, column=1, padx=(2, 12))
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, sticky="w")
        self.port_var = tk.StringVar(value="5000")
        ttk.Entry(conn_frame, textvariable=self.port_var, width=8).grid(
            row=0, column=3, padx=(2, 12))
        self.connect_btn = ttk.Button(conn_frame, text="连接", style="Accent.TButton",
                                       command=self._on_connect)
        self.connect_btn.grid(row=0, column=4, padx=4)
        self.disconnect_btn = ttk.Button(conn_frame, text="断开",
                                          command=self._on_disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=5, padx=4)
        self.ping_btn = ttk.Button(conn_frame, text="PING", command=self._on_ping_once,
                                    state="disabled")
        self.ping_btn.grid(row=0, column=6, padx=4)

        # ---- 主体 PanedWindow: 左(模型) | 中(图片) | 右(推理结果) ----
        body = ttk.PanedWindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=8, pady=4)

        self._build_model_panel(body)
        self._build_image_panel(body)
        self._build_infer_panel(body)

        # ---- 底部: STATUS 实时 + 日志 ----
        bottom = ttk.Frame(self.root)
        bottom.pack(fill="x", padx=8, pady=(2, 8))

        status_frame = ttk.LabelFrame(bottom, text="ConvCore CSR STATUS (实时 poll)", padding=6)
        status_frame.pack(fill="x", pady=(0, 4))
        self.status_grids = []
        for c in range(NUM_CORES):
            g = StatusBitGrid(status_frame, c)
            g.pack(fill="x", pady=1)
            self.status_grids.append(g)
        # poll 控制
        ctrl_row = tk.Frame(status_frame, bg=CL_BG)
        ctrl_row.pack(fill="x", pady=(4, 0))
        self.poll_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctrl_row, text="自动 poll STATUS (每 100ms)",
                        variable=self.poll_var,
                        command=self._toggle_poll).pack(side="left", padx=4)
        ttk.Button(ctrl_row, text="单次 PEEK", command=self._peek_status_once).pack(side="left", padx=4)

        log_frame = ttk.LabelFrame(bottom, text="日志 (RPC + 状态)", padding=6)
        log_frame.pack(fill="x")
        self.log = LogPane(log_frame)
        self.log.pack(fill="x")
        self.log.info("GUI 启动. " + ("拖拽支持已启用 (tkinterdnd2)" if HAS_DND
                                       else "拖拽未安装 (pip install tkinterdnd2)"))

        # ---- 底部状态栏 ----
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(self.root, textvariable=self.status_var, anchor="w",
                  background=CL_PANEL, foreground=CL_FG).pack(side="bottom", fill="x")

    def _build_model_panel(self, parent):
        outer = ttk.Frame(parent, padding=4)
        parent.add(outer, weight=2)

        # 模型加载
        load_frame = ttk.LabelFrame(outer, text="① 模型 (case_dir)", padding=6)
        load_frame.pack(fill="x", pady=(0, 4))

        self.case_label = tk.Label(load_frame, text="(拖入 sim case 目录, 或点 Browse)",
                                    bg="#181828", fg=CL_DIM,
                                    font=("Consolas", 9), anchor="w", padx=8, pady=12,
                                    relief="solid", borderwidth=1)
        self.case_label.pack(fill="x", pady=(0, 4))
        if HAS_DND:
            self.case_label.drop_target_register(DND_FILES)
            self.case_label.dnd_bind("<<Drop>>", self._on_drop_case)

        btn_row = tk.Frame(load_frame, bg=CL_BG)
        btn_row.pack(fill="x")
        ttk.Button(btn_row, text="Browse...",
                   command=self._on_browse_case).pack(side="left", padx=2)
        ttk.Button(btn_row, text="刷新", command=self._reload_case).pack(side="left", padx=2)
        self.deploy_btn = ttk.Button(btn_row, text="▶ 部署到板", style="Accent.TButton",
                                      command=self._on_deploy, state="disabled")
        self.deploy_btn.pack(side="right", padx=2)

        # 模型元信息
        meta_frame = ttk.LabelFrame(outer, text="模型元信息", padding=6)
        meta_frame.pack(fill="x", pady=4)
        self.meta_text = tk.Text(meta_frame, height=5, bg=CL_PANEL, fg=CL_FG,
                                  font=("Consolas", 9), borderwidth=0,
                                  state="disabled")
        self.meta_text.pack(fill="x")

        # Layer 列表
        layer_frame = ttk.LabelFrame(outer, text="11 层结构", padding=4)
        layer_frame.pack(fill="both", expand=True, pady=4)
        cols = ("L", "mode", "K", "stride", "h_in", "w_in", "c_in", "c_out",
                "h_out", "w_out", "desc")
        self.layer_tree = ttk.Treeview(layer_frame, columns=cols, show="headings",
                                        height=11)
        widths = {"L": 30, "mode": 40, "K": 30, "stride": 40,
                  "h_in": 50, "w_in": 50, "c_in": 50, "c_out": 50,
                  "h_out": 50, "w_out": 50, "desc": 50}
        for c in cols:
            self.layer_tree.heading(c, text=c)
            self.layer_tree.column(c, width=widths[c], anchor="center")
        self.layer_tree.pack(fill="both", expand=True)

        # 部署进度
        prog_frame = ttk.LabelFrame(outer, text="部署进度", padding=4)
        prog_frame.pack(fill="x", pady=(4, 0))
        self.deploy_progress = ttk.Progressbar(prog_frame, mode="determinate",
                                                style="Horizontal.TProgressbar")
        self.deploy_progress.pack(fill="x")
        self.deploy_stat = tk.Label(prog_frame, text="未部署", bg=CL_BG, fg=CL_DIM,
                                     font=("Consolas", 9))
        self.deploy_stat.pack(fill="x", pady=(2, 0))

    def _build_image_panel(self, parent):
        outer = ttk.Frame(parent, padding=4)
        parent.add(outer, weight=2)

        load_frame = ttk.LabelFrame(outer, text="② 输入图片", padding=6)
        load_frame.pack(fill="x", pady=(0, 4))

        self.img_drop = tk.Label(load_frame, text="(拖入图片, 或点 Browse)",
                                  bg="#181828", fg=CL_DIM, font=("Consolas", 9),
                                  anchor="w", padx=8, pady=12,
                                  relief="solid", borderwidth=1)
        self.img_drop.pack(fill="x", pady=(0, 4))
        if HAS_DND:
            self.img_drop.drop_target_register(DND_FILES)
            self.img_drop.dnd_bind("<<Drop>>", self._on_drop_image)

        btn_row = tk.Frame(load_frame, bg=CL_BG)
        btn_row.pack(fill="x")
        ttk.Button(btn_row, text="Browse...",
                   command=self._on_browse_image).pack(side="left", padx=2)
        ttk.Button(btn_row, text="清除", command=self._clear_image).pack(side="left", padx=2)
        self.upload_btn = ttk.Button(btn_row, text="上传到板",
                                      command=self._on_upload_image, state="disabled")
        self.upload_btn.pack(side="right", padx=2)

        # 缩略图
        prev_frame = ttk.LabelFrame(outer, text="缩略图 (resize 540×960)", padding=4)
        prev_frame.pack(fill="both", expand=True, pady=4)
        self.img_canvas = tk.Canvas(prev_frame, width=380, height=380, bg="#11111b",
                                     borderwidth=0, highlightthickness=0)
        self.img_canvas.pack(expand=True)
        self.img_canvas.create_text(190, 190, text="(无图)",
                                     fill=CL_DIM, font=("Arial", 12))

        # 量化信息
        info_frame = ttk.LabelFrame(outer, text="量化信息", padding=6)
        info_frame.pack(fill="x", pady=(4, 0))
        self.img_info = tk.Text(info_frame, height=4, bg=CL_PANEL, fg=CL_FG,
                                 font=("Consolas", 9), borderwidth=0, state="disabled")
        self.img_info.pack(fill="x")

    def _build_infer_panel(self, parent):
        outer = ttk.Frame(parent, padding=4)
        parent.add(outer, weight=3)

        # 推理控制
        ctrl_frame = ttk.LabelFrame(outer, text="③ 推理触发", padding=6)
        ctrl_frame.pack(fill="x", pady=(0, 4))

        rep_row = tk.Frame(ctrl_frame, bg=CL_BG)
        rep_row.pack(fill="x", pady=2)
        ttk.Label(rep_row, text="重复次数:").pack(side="left")
        self.repeat_var = tk.StringVar(value="10")
        ttk.Entry(rep_row, textvariable=self.repeat_var, width=6).pack(side="left", padx=4)
        ttk.Label(rep_row, text="(板上自循环, 不重发图)", foreground=CL_DIM).pack(side="left")

        btn_row = tk.Frame(ctrl_frame, bg=CL_BG)
        btn_row.pack(fill="x", pady=4)
        self.infer_btn = ttk.Button(btn_row, text="▶ 推理 (RUN_LAYERS)",
                                     style="Accent.TButton",
                                     command=self._on_infer, state="disabled")
        self.infer_btn.pack(side="left", padx=2, fill="x", expand=True)
        self.allinone_btn = ttk.Button(btn_row, text="一键 (部署+上传+推理)",
                                        style="OK.TButton",
                                        command=self._on_allinone, state="disabled")
        self.allinone_btn.pack(side="left", padx=2, fill="x", expand=True)

        # 推理进度
        self.infer_progress = ttk.Progressbar(ctrl_frame, mode="determinate")
        self.infer_progress.pack(fill="x", pady=2)

        # 性能数字
        perf_frame = ttk.LabelFrame(outer, text="性能", padding=6)
        perf_frame.pack(fill="x", pady=4)

        # 大字显示 cycles + ms + FPS
        big = tk.Frame(perf_frame, bg=CL_BG)
        big.pack(fill="x")

        def _kpi(parent, label, color):
            f = tk.Frame(parent, bg=CL_PANEL, padx=8, pady=4)
            f.pack(side="left", fill="x", expand=True, padx=2)
            tk.Label(f, text=label, bg=CL_PANEL, fg=CL_DIM,
                     font=("Arial", 8)).pack(anchor="w")
            v = tk.Label(f, text="—", bg=CL_PANEL, fg=color,
                         font=("Consolas", 18, "bold"))
            v.pack(anchor="w")
            return v

        self.kpi_cy = _kpi(big, "HW cycles", CL_BAR)
        self.kpi_ms = _kpi(big, "HW time (ms)", CL_OK)
        self.kpi_fps = _kpi(big, "HW FPS", CL_ACCENT)
        self.kpi_rtt = _kpi(big, "RTT (ms)", CL_WARN)

        # 历史图
        hist_frame = ttk.LabelFrame(outer, text="历史 cycles (最近 20 次)", padding=4)
        hist_frame.pack(fill="x", pady=4)
        self.history_chart = HistoryChart(hist_frame)
        self.history_chart.pack(fill="x")

        # Top-5 logit
        top5_frame = ttk.LabelFrame(outer, text="Top-5 logit", padding=6)
        top5_frame.pack(fill="both", expand=True, pady=4)
        self.top5_text = tk.Text(top5_frame, bg=CL_PANEL, fg=CL_FG,
                                  font=("Consolas", 11), borderwidth=0, state="disabled")
        self.top5_text.pack(fill="both", expand=True)

    # =========================================================================
    # 拖拽 + browse 处理
    # =========================================================================
    @staticmethod
    def _parse_dnd_path(data: str) -> str:
        """tkinterdnd2 给的路径可能用 {} 包含空格. 取第一个."""
        s = data.strip()
        if s.startswith("{") and "}" in s:
            return s[1:s.index("}")]
        return s.split()[0] if s else ""

    def _on_drop_case(self, event):
        path = self._parse_dnd_path(event.data)
        if path and os.path.isdir(path):
            self._load_case_dir(path)
        else:
            self.log.err(f"拖入的不是目录: {path}")

    def _on_browse_case(self):
        d = filedialog.askdirectory(title="选 sim/tb_smc/cases/<name>/ 目录")
        if d:
            self._load_case_dir(d)

    def _reload_case(self):
        if self.case_dir:
            self._load_case_dir(self.case_dir)

    def _load_case_dir(self, path: str):
        try:
            meta = dsc.parse_meta(path)
        except Exception as e:
            self.log.err(f"解析 meta 失败: {e}")
            return
        try:
            gb = dsc.derive_smc_global_base(meta)
        except Exception as e:
            self.log.err(f"反推 SMC_GLOBAL_BASE 失败: {e}")
            return

        self.case_dir = path
        self.case_meta = meta
        self.global_base = gb
        self.deployed = False
        n_layers = meta['NUM_LAYERS']
        n_cores = meta['NUM_CORES']

        # UI 更新
        self.case_label.config(text=f"✓ {path}", fg=CL_OK)
        meta_lines = [
            f"路径: {path}",
            f"NUM_CORES = {n_cores}    NUM_LAYERS = {n_layers}",
            f"SMC_GLOBAL_BASE = 0x{gb:08x}    (避开 0x00000000-0x{0x01000000:08x} a72 ELF)",
            f"FINAL OFM: H×W×C = {meta.get(f'LAYER_{n_layers-1}_H_OUT')}×"
            f"{meta.get(f'LAYER_{n_layers-1}_W_OUT')}×"
            f"{meta.get(f'LAYER_{n_layers-1}_C_OUT')}",
        ]
        self._set_text(self.meta_text, "\n".join(meta_lines))

        # 11 层 Treeview
        self.layer_tree.delete(*self.layer_tree.get_children())
        for l in range(n_layers):
            mode = meta.get(f'SMC_LAYER_{l}_MODE', '?')
            k = meta.get(f'SMC_LAYER_{l}_K', '-')
            stride = meta.get(f'SMC_LAYER_{l}_STRIDE', '-')
            h_in = meta.get(f'SMC_LAYER_{l}_H_IN', '-')
            w_in = meta.get(f'SMC_LAYER_{l}_W_IN', '-')
            cin_slices = meta.get(f'SMC_LAYER_{l}_C_IN_SLICES', 0)
            cout_slices = meta.get(f'SMC_LAYER_{l}_C_OUT_SLICES', 0)
            h_out = meta.get(f'LAYER_{l}_H_OUT', '-')
            w_out = meta.get(f'LAYER_{l}_W_OUT', '-')
            c_out = meta.get(f'LAYER_{l}_C_OUT', '-')
            c_in = (cin_slices * 16) if isinstance(cin_slices, int) else '-'
            desc = max((meta.get(f'CORE_{c}_LAYER_{l}_DESC_COUNT', 0) for c in range(n_cores)),
                       default=0)
            self.layer_tree.insert("", "end", values=(
                l, mode, k, stride, h_in, w_in, c_in, c_out, h_out, w_out, desc))

        self.log.ok(f"加载 case: {os.path.basename(path)}, "
                     f"{n_layers} layers, {n_cores} cores, "
                     f"SMC_GLOBAL_BASE=0x{gb:08x}")
        self._refresh_buttons()

    def _on_drop_image(self, event):
        path = self._parse_dnd_path(event.data)
        if path and os.path.isfile(path):
            self._load_image(path)
        else:
            self.log.err(f"拖入的不是文件: {path}")

    def _on_browse_image(self):
        p = filedialog.askopenfilename(
            title="选图片", filetypes=[("Image", "*.png *.jpg *.jpeg *.bmp"), ("All", "*.*")])
        if p:
            self._load_image(p)

    def _clear_image(self):
        self.image_path = None
        self.image_bytes_orig = None
        self.image_bytes_s2d = None
        self.image_uploaded = False
        self.img_canvas.delete("all")
        self.img_canvas.create_text(190, 190, text="(无图)",
                                     fill=CL_DIM, font=("Arial", 12))
        self.img_drop.config(text="(拖入图片, 或点 Browse)", fg=CL_DIM)
        self._set_text(self.img_info, "")
        self._refresh_buttons()

    def _load_image(self, path: str):
        try:
            from PIL import Image, ImageTk
        except ImportError:
            messagebox.showerror("缺依赖", "需要 Pillow:\npip install Pillow")
            return
        try:
            t0 = time.time()
            img_pil = Image.open(path).convert("RGB")
            orig_size = img_pil.size
            img_resized = img_pil.resize((540, 960), Image.BILINEAR)

            # 缩略图 (380×380 max, keep aspect)
            preview = img_resized.copy()
            preview.thumbnail((380, 380))
            self._img_tk = ImageTk.PhotoImage(preview)
            self.img_canvas.delete("all")
            self.img_canvas.create_image(190, 190, image=self._img_tk)

            # NHWC int8 (centered, R-G-B-pad0)
            from run_resnet11_demo import load_image_int8, s2d_rearrange, \
                ORIG_H, ORIG_W, ORIG_CIN, PATCH_STRIDE, H_S2D, W_S2D, CIN_S2D
            self.image_bytes_orig = bytes(load_image_int8(path))
            self.image_bytes_s2d = s2d_rearrange(self.image_bytes_orig,
                                                  ORIG_H, ORIG_W, ORIG_CIN, PATCH_STRIDE)
            t_load = time.time() - t0

            self.image_path = path
            self.image_uploaded = False
            self.img_drop.config(text=f"✓ {os.path.basename(path)}", fg=CL_OK)
            info_lines = [
                f"路径: {path}",
                f"原图: {orig_size[0]}×{orig_size[1]}",
                f"resize: {ORIG_W}×{ORIG_H}×{ORIG_CIN} = {len(self.image_bytes_orig)/1024:.0f} KB",
                f"s2d:   {W_S2D}×{H_S2D}×{CIN_S2D} = {len(self.image_bytes_s2d)/1024:.0f} KB"
                f"  (load+s2d {t_load*1000:.0f}ms)",
            ]
            self._set_text(self.img_info, "\n".join(info_lines))
            self.log.ok(f"加载图: {os.path.basename(path)} → s2d {len(self.image_bytes_s2d)/1024:.0f} KB")
            self._refresh_buttons()
        except Exception as e:
            self.log.err(f"加载图失败: {e}")
            messagebox.showerror("加载图失败", str(e))

    # =========================================================================
    # 连接 / RPC
    # =========================================================================
    def _on_connect(self):
        ip = self.ip_var.get().strip()
        try:
            port = int(self.port_var.get())
        except ValueError:
            messagebox.showerror("Port 非法", "Port 必须是数字")
            return
        self.connect_btn.config(state="disabled")
        self.status_var.set(f"连接 {ip}:{port}...")
        self.log.info(f"连接 {ip}:{port}...")
        self.root.update_idletasks()

        try:
            rpc = Vd100Rpc(ip, port, timeout=5.0)
            rpc.connect()
            magic = rpc.ping()
            rpc.sock.settimeout(60.0)
        except Exception as e:
            self.log.err(f"连接失败: {e}")
            messagebox.showerror("连接失败", f"{ip}:{port}\n{e}")
            self.status_var.set(f"连接失败: {e}")
            self.connect_btn.config(state="normal")
            return

        self.rpc = rpc
        self.conn_led.set_state(CL_OK)
        self.conn_text.config(text="已连接", fg=CL_OK)
        self.disconnect_btn.config(state="normal")
        self.ping_btn.config(state="normal")
        self.log.ok(f"已连接 {ip}:{port}, ping=0x{magic:08x}")
        self.status_var.set(f"已连接 {ip}:{port}")
        self._inc_rpc(0, 16, 16)
        self._refresh_buttons()
        # 启动后台 ping (低频, 5s 一次)
        self._start_ping_loop()

    def _on_disconnect(self):
        self._stop_poll_status()
        self._stop_ping_loop()
        if self.rpc:
            try: self.rpc.close()
            except Exception: pass
            self.rpc = None
        self.conn_led.set_state(CL_DIM)
        self.conn_text.config(text="未连接", fg=CL_DIM)
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.ping_btn.config(state="disabled")
        self.poll_var.set(False)
        self.deployed = False
        self.image_uploaded = False
        self.ping_label.config(text="ping —")
        self.log.warn("已断开")
        self.status_var.set("已断开")
        self._refresh_buttons()

    def _on_ping_once(self):
        if not self.rpc:
            return
        try:
            t0 = time.time()
            v = self.rpc.ping()
            rtt = (time.time() - t0) * 1000
            self.log.rpc(f"PING → 0x{v:08x} ({rtt:.1f}ms)")
            self.ping_label.config(text=f"ping {rtt:.1f}ms",
                                    fg=CL_OK if rtt < 50 else CL_WARN)
            self._inc_rpc(0, 16, 16)
        except Exception as e:
            self.log.err(f"PING 失败: {e}")
            self._on_disconnect()

    # 后台 ping 循环
    def _start_ping_loop(self):
        self._stop_ping_loop()
        self._ping_stop.clear()
        t = threading.Thread(target=self._ping_loop_worker, daemon=True)
        t.start()
        self._ping_thread = t

    def _stop_ping_loop(self):
        self._ping_stop.set()
        self._ping_thread = None

    def _ping_loop_worker(self):
        while not self._ping_stop.is_set():
            time.sleep(5)
            if self._ping_stop.is_set():
                break
            if not self.rpc:
                break
            try:
                t0 = time.time()
                self.rpc.ping()
                rtt = (time.time() - t0) * 1000
                self.msg_q.put(("ping_ok", rtt))
            except Exception as e:
                self.msg_q.put(("ping_fail", str(e)))
                break

    # =========================================================================
    # 部署 / 上传图 / 推理
    # =========================================================================
    def _on_deploy(self):
        if not (self.rpc and self.case_dir and self.case_meta):
            return
        self.deploy_btn.config(state="disabled")
        self.allinone_btn.config(state="disabled")
        self.infer_btn.config(state="disabled")
        threading.Thread(target=self._deploy_worker, daemon=True).start()

    def _deploy_worker(self):
        try:
            meta = self.case_meta
            n_layers = meta['NUM_LAYERS']
            self.msg_q.put(("deploy_start", n_layers))
            for l in range(n_layers):
                is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
                if is_root:
                    dsc.load_ifb_smc(self.rpc, self.case_dir, meta, l, self.global_base)
                dsc.load_wb_smc(self.rpc, self.case_dir, meta, l, self.global_base)
                dsc.load_rdma_smc(self.rpc, self.case_dir, meta, l, self.global_base)
                dsc.load_desc_smc(self.rpc, self.case_dir, meta, l)
                dsc.load_sg_cmd_smc(self.rpc, self.case_dir, meta, l)
                self.msg_q.put(("deploy_progress", l + 1, n_layers))
            self.msg_q.put(("deploy_done",))
        except Exception as e:
            self.msg_q.put(("deploy_err", str(e)))

    def _on_upload_image(self):
        if not (self.rpc and self.image_bytes_s2d and self.case_meta):
            return
        self.upload_btn.config(state="disabled")
        threading.Thread(target=self._upload_worker, daemon=True).start()

    def _upload_worker(self):
        try:
            t0 = time.time()
            dsc._load_ifb_override(self.rpc, self.image_bytes_s2d,
                                    self.case_meta, self.global_base)
            dt = time.time() - t0
            self.msg_q.put(("upload_done", dt, len(self.image_bytes_s2d)))
        except Exception as e:
            self.msg_q.put(("upload_err", str(e)))

    def _on_infer(self):
        if not (self.rpc and self.deployed and self.case_meta):
            return
        try:
            n = max(1, int(self.repeat_var.get()))
        except ValueError:
            messagebox.showerror("非法", "重复次数必须是正整数")
            return
        self.infer_btn.config(state="disabled")
        self.allinone_btn.config(state="disabled")
        self.infer_progress["maximum"] = n
        self.infer_progress["value"] = 0
        threading.Thread(target=self._infer_worker, args=(n,), daemon=True).start()

    def _infer_worker(self, n: int):
        try:
            cfgs = dsc.build_layer_cfgs(self.case_meta)
            for i in range(n):
                t0 = time.time()
                cy = self.rpc.run_layers(cfgs)
                rtt = time.time() - t0
                self.msg_q.put(("infer_progress", i + 1, n, cy, rtt))

            # 读 OFM (最后一层) 取 Top-5
            n_layers = self.case_meta['NUM_LAYERS']
            t0 = time.time()
            ofm = dsc.read_ofm_smc(self.rpc, self.case_meta, n_layers - 1, self.global_base)
            t_read = time.time() - t0
            self.msg_q.put(("infer_done", ofm, t_read))
        except Exception as e:
            self.msg_q.put(("infer_err", str(e)))

    def _on_allinone(self):
        if not (self.rpc and self.case_dir and self.image_bytes_s2d):
            messagebox.showinfo("提示", "需要先连板 + 加载 case + 加载图片")
            return
        self.deploy_btn.config(state="disabled")
        self.allinone_btn.config(state="disabled")
        self.infer_btn.config(state="disabled")
        self.upload_btn.config(state="disabled")
        threading.Thread(target=self._allinone_worker, daemon=True).start()

    def _allinone_worker(self):
        # 部署 + 上传图 + 推理
        try:
            meta = self.case_meta
            n_layers = meta['NUM_LAYERS']
            self.msg_q.put(("deploy_start", n_layers))

            # ① 部署
            for l in range(n_layers):
                is_root = (l == 0) or (meta.get(f'LAYER_{l}_PRELOAD_IFB', 0) == 1)
                if is_root and l > 0:
                    dsc.load_ifb_smc(self.rpc, self.case_dir, meta, l, self.global_base)
                dsc.load_wb_smc(self.rpc, self.case_dir, meta, l, self.global_base)
                dsc.load_rdma_smc(self.rpc, self.case_dir, meta, l, self.global_base)
                dsc.load_desc_smc(self.rpc, self.case_dir, meta, l)
                dsc.load_sg_cmd_smc(self.rpc, self.case_dir, meta, l)
                self.msg_q.put(("deploy_progress", l + 1, n_layers))

            # ② 上传图作为 layer 0 IFB
            t0 = time.time()
            dsc._load_ifb_override(self.rpc, self.image_bytes_s2d, meta, self.global_base)
            self.msg_q.put(("upload_done", time.time() - t0, len(self.image_bytes_s2d)))

            # ③ 推理
            try:
                n = max(1, int(self.repeat_var.get()))
            except ValueError:
                n = 1
            cfgs = dsc.build_layer_cfgs(meta)
            for i in range(n):
                t0 = time.time()
                cy = self.rpc.run_layers(cfgs)
                rtt = time.time() - t0
                self.msg_q.put(("infer_progress", i + 1, n, cy, rtt))
            ofm = dsc.read_ofm_smc(self.rpc, meta, n_layers - 1, self.global_base)
            self.msg_q.put(("infer_done", ofm, 0.0))
        except Exception as e:
            self.msg_q.put(("infer_err", f"all-in-one 失败: {e}"))

    # =========================================================================
    # STATUS poll 后台
    # =========================================================================
    def _toggle_poll(self):
        if self.poll_var.get():
            self._start_poll_status()
        else:
            self._stop_poll_status()

    def _start_poll_status(self):
        self._stop_poll_status()
        self._poll_stop.clear()
        t = threading.Thread(target=self._poll_status_worker, daemon=True)
        t.start()
        self._poll_thread = t

    def _stop_poll_status(self):
        self._poll_stop.set()
        self._poll_thread = None

    def _peek_status_once(self):
        if not self.rpc:
            return
        try:
            for c in range(NUM_CORES):
                v = self.rpc.peek_csr(c, CSR_STATUS)
                self.status_grids[c].update(v)
                self._inc_rpc(0, 16, 16)
        except Exception as e:
            self.log.err(f"PEEK STATUS 失败: {e}")
            self._on_disconnect()

    def _poll_status_worker(self):
        while not self._poll_stop.is_set():
            if not self.rpc:
                break
            try:
                vals = []
                for c in range(NUM_CORES):
                    v = self.rpc.peek_csr(c, CSR_STATUS)
                    vals.append((c, v))
                self.msg_q.put(("status_update", vals))
            except Exception as e:
                self.msg_q.put(("status_err", str(e)))
                break
            time.sleep(0.1)

    # =========================================================================
    # UI 主线程消费 msg queue
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
        kind = msg[0]
        if kind == "deploy_start":
            n_layers = msg[1]
            self.deploy_progress["maximum"] = n_layers
            self.deploy_progress["value"] = 0
            self.deploy_stat.config(text=f"部署中 0/{n_layers}", fg=CL_WARN)
            self.log.info(f"开始部署模型, 共 {n_layers} 层...")
        elif kind == "deploy_progress":
            i, n = msg[1], msg[2]
            self.deploy_progress["value"] = i
            self.deploy_stat.config(text=f"部署中 {i}/{n}", fg=CL_WARN)
            self.status_var.set(f"部署 {i}/{n}")
        elif kind == "deploy_done":
            self.deployed = True
            self.deploy_stat.config(text="✓ 部署完成", fg=CL_OK)
            self.log.ok("模型部署完成")
            self.status_var.set("模型已部署")
            self._refresh_buttons()
        elif kind == "deploy_err":
            self.deploy_stat.config(text=f"✗ 失败: {msg[1]}", fg=CL_ERR)
            self.log.err(f"部署失败: {msg[1]}")
            self._refresh_buttons()
        elif kind == "upload_done":
            dt, n = msg[1], msg[2]
            self.image_uploaded = True
            self.log.ok(f"图片上传完成 ({n/1024:.0f} KB, {dt*1000:.0f}ms)")
            self.upload_btn.config(state="normal")
            self._refresh_buttons()
        elif kind == "upload_err":
            self.log.err(f"上传图失败: {msg[1]}")
            self.upload_btn.config(state="normal")
        elif kind == "infer_progress":
            i, n, cy, rtt = msg[1:]
            self.infer_progress["value"] = i
            self.kpi_cy.config(text=f"{cy:,}")
            self.kpi_ms.config(text=f"{cy/100000:.2f}")
            self.kpi_fps.config(text=f"{100e6/cy:.0f}" if cy else "—")
            self.kpi_rtt.config(text=f"{rtt*1000:.0f}")
            self.history_chart.push(cy)
            self.log.rpc(f"infer #{i}/{n}: cy={cy} ({cy/100000:.2f}ms HW), rtt={rtt*1000:.0f}ms")
            self.status_var.set(f"推理 {i}/{n} cy={cy}")
        elif kind == "infer_done":
            ofm, t_read = msg[1], msg[2]
            self._show_top5(ofm)
            self.log.ok(f"推理完成, 读 OFM {len(ofm)} byte ({t_read*1000:.0f}ms)")
            self._refresh_buttons()
        elif kind == "infer_err":
            self.log.err(f"推理失败: {msg[1]}")
            self._refresh_buttons()
        elif kind == "status_update":
            for c, v in msg[1]:
                self.status_grids[c].update(v)
                self._inc_rpc(0, 16, 16)
        elif kind == "status_err":
            self.log.err(f"poll STATUS 失败: {msg[1]}")
            self.poll_var.set(False)
            self._on_disconnect()
        elif kind == "ping_ok":
            rtt = msg[1]
            self.ping_label.config(text=f"ping {rtt:.1f}ms",
                                    fg=CL_OK if rtt < 50 else CL_WARN)
            self._inc_rpc(0, 16, 16)
        elif kind == "ping_fail":
            self.log.err(f"ping 失败: {msg[1]}")
            self._on_disconnect()

    def _show_top5(self, ofm: bytes):
        if len(ofm) < 522:
            self.log.warn(f"OFM 太小 ({len(ofm)} byte)")
            return
        # int8 logit
        logits = [int.from_bytes([b], byteorder='little', signed=True) for b in ofm[:522]]
        idx_val = sorted(enumerate(logits), key=lambda x: -x[1])[:5]
        lines = [f"#{r}  class[{idx:>3d}] = {val:>+4d}"
                 for r, (idx, val) in enumerate(idx_val, 1)]
        lines.append("")
        lines.append("注: 522 类 random weights demo, 验证数据通路 bit-exact, 类别本身无意义.")
        self._set_text(self.top5_text, "\n".join(lines))

    # =========================================================================
    # Helpers
    # =========================================================================
    def _set_text(self, widget: tk.Text, text: str):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", text)
        widget.configure(state="disabled")

    def _inc_rpc(self, calls_delta=1, tx_bytes=0, rx_bytes=0):
        self.rpc_calls += max(1, calls_delta)
        self.rpc_bytes_out += tx_bytes
        self.rpc_bytes_in += rx_bytes
        # 用 vd100_rpc 的 send/recv 不会直接计 byte. 这是粗略估计.
        self.stats_label.config(
            text=f"RPC: {self.rpc_calls}  |  TX: {self.rpc_bytes_out/1024:.0f}KB  "
                 f"RX: {self.rpc_bytes_in/1024:.0f}KB"
        )

    def _refresh_buttons(self):
        # deploy 按钮: 需要 rpc + case
        if self.rpc and self.case_dir:
            self.deploy_btn.config(state="normal")
        else:
            self.deploy_btn.config(state="disabled")
        # upload 按钮: 需要 rpc + 图 + case
        if self.rpc and self.image_bytes_s2d and self.case_meta:
            self.upload_btn.config(state="normal")
        else:
            self.upload_btn.config(state="disabled")
        # infer 按钮: 需要 rpc + 已部署 (图片可选, 不传就是用上次/sim 默认 IFB)
        if self.rpc and self.deployed:
            self.infer_btn.config(state="normal")
        else:
            self.infer_btn.config(state="disabled")
        # all-in-one: 需要 rpc + case + 图
        if self.rpc and self.case_dir and self.image_bytes_s2d:
            self.allinone_btn.config(state="normal")
        else:
            self.allinone_btn.config(state="disabled")


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
    Vd100Gui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
