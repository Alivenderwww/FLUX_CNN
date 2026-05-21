"""
vd100_rpc.py — VD100 板 PC 端 RPC client lib

跟 host/vd100_ps_baremetal/vd100_rpc_server.c 配对. 板上烧的是 bitstream + 通用
RPC server (无业务知识), PC 通过本 lib 把所有数据/配置/触发/读结果走 TCP socket
推到板上, 跟 sim TB tb_smc_chain.sv 行为等价.

协议: 16-byte header + optional payload (header & footer 都 little-endian).
  REQ:  uint32 magic 'FXNN' | uint32 cmd | uint32 arg0 | uint32 arg1 | <payload>
  RESP: uint32 magic         | uint32 status | uint32 ret | uint32 payload_len | <payload>

Cmds:
  1 LOAD_DDR    arg0=ddr_addr arg1=len   payload=data       resp: status,0,0,0
  2 READ_DDR    arg0=ddr_addr arg1=len   no payload         resp: status,0,0,len, [data]
  3 POKE_CSR    arg0=csr_offset arg1=value                  resp: status,0,0,0
  4 PEEK_CSR    arg0=csr_offset                              resp: status,value,0,0
  5 RUN_LAYERS  arg0=n_layers arg1=payload_len  payload=LayerCfg[]  resp: status,cycles,0,0
  6 PING                                                    resp: status,0xCAFEBABE,0,0

LayerCfg (32 byte / layer, little-endian, 跟 server 端 layer_cfg_t 一致):
  uint32 desc_list_base[3]
  uint32 desc_count[3]
  uint32 reserved[2]   (pad to 32 byte)

CSR 地址 (跟 RTL/flux_cnn_params.svh 一致, BD addr[13:12]=core_id):
  CTRL=0x000  STATUS=0x004
  DESC_LIST_BASE=0x180  DESC_COUNT=0x184
  per-core 4KB stride: csr_offset = (core_id << 12) | reg_offset

用法: 见 test_ddr_loopback.py / run_resnet11_demo.py
"""
import socket
import struct
import time

MAGIC = 0x46584E4E   # 'FXNN'

# Cmds
CMD_LOAD_DDR    = 1
CMD_READ_DDR    = 2
CMD_POKE_CSR    = 3
CMD_PEEK_CSR    = 4
CMD_RUN_LAYERS  = 5
CMD_PING        = 6

# Status
STATUS_OK             = 0
STATUS_BAD_MAGIC      = 1
STATUS_BAD_CMD        = 2
STATUS_BAD_LEN        = 3
STATUS_RUN_TIMEOUT    = 4
STATUS_OOM            = 5

STATUS_NAMES = {
    0: "OK", 1: "BAD_MAGIC", 2: "BAD_CMD", 3: "BAD_LEN",
    4: "RUN_TIMEOUT", 5: "OOM",
}

# CSR offsets (跟 flux_cnn_params.svh 一致)
CSR_CTRL          = 0x000
CSR_STATUS        = 0x004
CSR_DESC_LIST_BASE = 0x180
CSR_DESC_COUNT    = 0x184
CTRL_START_DFE    = 0x10   # bit 4

NUM_CORES = 3
CSR_PER_CORE_STRIDE = 0x1000


def csr_addr(core_id: int, reg_offset: int) -> int:
    """csr_offset = (core_id << 12) | reg_offset, 跟 BD csr_axil 14-bit 解码一致."""
    return (core_id * CSR_PER_CORE_STRIDE) | reg_offset


class RpcError(RuntimeError):
    def __init__(self, status, msg=""):
        super().__init__(f"RPC error {status} ({STATUS_NAMES.get(status, '?')}): {msg}")
        self.status = status


class Vd100Rpc:
    """TCP RPC client for VD100 board.

    用法 (with-block 自动 close):
        with Vd100Rpc('169.254.111.10') as rpc:
            rpc.ping()
            rpc.load_ddr(0x10000000, b'\\x01\\x02...')
            data = rpc.read_ddr(0x10000000, 4)
            rpc.poke_csr(0, CSR_CTRL, CTRL_START_DFE)
            cycles = rpc.run_layers([(desc_base[c], desc_count[c]) for c in range(3)] for ...)
    """
    def __init__(self, host: str = '169.254.111.10', port: int = 5000, timeout: float = 90.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        # 减少 Nagle 延迟, 大 LOAD_DDR 也走快
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.host, self.port))

    def close(self):
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()
            self.sock = None

    # -------------------- 底层 send/recv --------------------
    def _send_all(self, data: bytes):
        self.sock.sendall(data)

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(min(65536, n - len(buf)))
            if not chunk:
                raise RpcError(-1, f"server closed (got {len(buf)}/{n} bytes)")
            buf.extend(chunk)
        return bytes(buf)

    def _request(self, cmd: int, arg0: int = 0, arg1: int = 0,
                 payload: bytes = b'') -> tuple:
        """发 req + 收 resp. 返回 (status, ret, resp_payload)."""
        hdr = struct.pack('<IIII', MAGIC, cmd, arg0, arg1)
        if payload:
            self._send_all(hdr + payload)
        else:
            self._send_all(hdr)

        resp_hdr = self._recv_exact(16)
        magic, status, ret, plen = struct.unpack('<IIII', resp_hdr)
        if magic != MAGIC:
            raise RpcError(-2, f"bad resp magic 0x{magic:08x}")
        resp_payload = self._recv_exact(plen) if plen > 0 else b''
        if status != STATUS_OK:
            raise RpcError(status, f"cmd={cmd} arg0=0x{arg0:x} arg1=0x{arg1:x}")
        return status, ret, resp_payload

    # -------------------- 高层 RPC API --------------------
    def ping(self) -> int:
        """返回 server 'magic' 0xCAFEBABE 验证通路通."""
        _, ret, _ = self._request(CMD_PING)
        if ret != 0xCAFEBABE:
            raise RpcError(-3, f"ping ret expect 0xCAFEBABE got 0x{ret:08x}")
        return ret

    def load_ddr(self, ddr_addr: int, data: bytes):
        """把 data 写到板 DDR 地址 ddr_addr (server 端 DCacheFlushRange)."""
        if not data:
            return
        # 一次最大 ~1 MB chunk, 避免 lwIP buffer 溢出 (server 是流式 memcpy, 没限制
        # 但 socket buffer 大块会拖累 throughput).
        CHUNK = 1024 * 1024
        if len(data) <= CHUNK:
            self._request(CMD_LOAD_DDR, ddr_addr, len(data), data)
        else:
            for off in range(0, len(data), CHUNK):
                chunk = data[off:off + CHUNK]
                self._request(CMD_LOAD_DDR, ddr_addr + off, len(chunk), chunk)

    def read_ddr(self, ddr_addr: int, length: int) -> bytes:
        """从板 DDR 地址 ddr_addr 读 length 字节 (server 端 DCacheInvalidateRange).
        分段读避免 lwIP 大 transfer 超 timeout.
        Chunk 8 KB (lwIP TCP send window 4 KB 友好), 单 chunk timeout ≤ 30s.
        """
        CHUNK = 1024 * 1024  # 1 MB chunk (回到原默认), 配合 timeout=90s 跑大 OFM
        if length <= CHUNK:
            _, _, data = self._request(CMD_READ_DDR, ddr_addr, length)
            return data
        out = bytearray()
        for off in range(0, length, CHUNK):
            n = min(CHUNK, length - off)
            _, _, chunk = self._request(CMD_READ_DDR, ddr_addr + off, n)
            out.extend(chunk)
        return bytes(out)

    def poke_csr(self, core_id: int, reg_offset: int, value: int):
        """写一个 ConvCore CSR. 等价 sim TB axi_lite_write."""
        self._request(CMD_POKE_CSR, csr_addr(core_id, reg_offset), value & 0xFFFFFFFF)

    def peek_csr(self, core_id: int, reg_offset: int) -> int:
        """读一个 ConvCore CSR."""
        _, ret, _ = self._request(CMD_PEEK_CSR, csr_addr(core_id, reg_offset))
        return ret

    def run_layers(self, layers: list) -> int:
        """跑 N 层推理. layers 是一个 list of dict, 每个 dict 含
        'desc_list_base': [c0, c1, c2]   (3 核 desc list 起始 DDR addr, 0=该核不参与)
        'desc_count':     [c0, c1, c2]   (3 核 desc count)
        返回 100 MHz cycles (a72 timer 实测).

        a72 server 会 11 层循环: 写 DESC_LIST_BASE/COUNT + start_dfe + poll done.
        在 sim 这一步是 axi_lite_write + write_dfe_start + wait_dfe_done.
        """
        # 打包 LayerCfg array (32 byte / layer): 3*u32 desc_base + 3*u32 desc_count + 2*u32 pad.
        # 协议固定 3 core (跟 PS server 端 layer_cfg_t 一致). NUM_CORES<3 时把多余 slot
        # 填 0, PS server 见 desc_count[c]==0 会跳过, 不写不存在的 csr_axil.
        n = len(layers)
        if n == 0:
            return 0
        buf = bytearray()
        for cfg in layers:
            base = list(cfg['desc_list_base'])
            cnt  = list(cfg['desc_count'])
            assert len(base) == NUM_CORES and len(cnt) == NUM_CORES
            # pad 到长度 3
            while len(base) < 3:
                base.append(0)
            while len(cnt) < 3:
                cnt.append(0)
            buf.extend(struct.pack('<III', base[0], base[1], base[2]))
            buf.extend(struct.pack('<III', cnt[0],  cnt[1],  cnt[2]))
            buf.extend(struct.pack('<II', 0, 0))  # reserved
        payload = bytes(buf)
        _, cycles, _ = self._request(CMD_RUN_LAYERS, n, len(payload), payload)
        return cycles


# -------------------- helper: 构造 LayerCfg --------------------
def make_layer_cfg(desc_list_base, desc_count):
    """返回单层 cfg dict. desc_list_base / desc_count 都是长 NUM_CORES 的 list/tuple."""
    return {
        'desc_list_base': list(desc_list_base),
        'desc_count':     list(desc_count),
    }
