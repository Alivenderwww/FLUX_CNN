#!/usr/bin/env python3
"""
resnet11_client.py — VD100 demo Windows 上位机 client

功能:
  - 加载本地图片 → resize → INT8 量化
  - TCP socket 发图给 VD100 板 (PS Linux 跑 server)
  - 收 OFM 回来, 打印 522-d logit top-5 (random weights, 仅 demo)
  - 测延迟 / 吞吐 (单图 / batch 测试)

协议 (跟 host/vd100_ps/resnet11_server.c 配对):
  REQ:
    magic   uint32 = 0x46584E4E ('FXNN')
    cmd     uint32 = 1 (= INFER)
    ifm_len uint32 = H × W × cin × 1 (= 540 * 960 * 4 = 2073600 byte)
    ifm     bytes  (int8, [-128, 127], H-major W-major C-minor)
  RESP:
    magic   uint32 = 0x46584E4E
    status  uint32 (0 = OK, !=0 = error)
    cy      uint32 (硬件实测 cycles, hw_count)
    ofm_len uint32 (522 × 1 = 522 byte for FC layer 输出)
    ofm     bytes  (int8 logit)

用法 (Windows 命令行):
  python resnet11_client.py --image gradient.png --vd100-ip 192.168.1.100 --port 5000
"""
import argparse
import socket
import struct
import sys
import time

MAGIC = 0x46584E4E   # 'FXNN'
CMD_INFER  = 1   # 上传 IFM + 跑推理 + 回 OFM
CMD_REPEAT = 2   # 不上传 IFM, 复用上次 server DDR 里的 IFM, 跑推理 + 回 OFM

# ResNet11 layer 0 输入: 960x540x4 (跟 run_regression.py CASES 一致)
IFM_H, IFM_W, IFM_CIN = 960, 540, 4
IFM_LEN = IFM_H * IFM_W * IFM_CIN   # = 2,073,600 byte ≈ 2 MB
OFM_LEN = 522   # FC 输出 logit 数 (跟 run_regression.py CASES[10] c_out=522 一致)


def load_image_to_int8(image_path):
    """PNG/JPG → resize to (IFM_H, IFM_W) → cin=4 INT8 byte stream."""
    try:
        from PIL import Image
    except ImportError:
        sys.exit("ERROR: 需要 Pillow. 装: pip install Pillow")

    img = Image.open(image_path).convert('RGB')
    img = img.resize((IFM_W, IFM_H), Image.BILINEAR)
    pixels = list(img.getdata())   # [H*W] tuples (R,G,B)

    # uint8 → int8 (center normalize: -128)
    buf = bytearray(IFM_LEN)
    idx = 0
    for r in range(IFM_H):
        for c in range(IFM_W):
            R, G, B = pixels[r * IFM_W + c]
            buf[idx]   = (R - 128) & 0xFF
            buf[idx+1] = (G - 128) & 0xFF
            buf[idx+2] = (B - 128) & 0xFF
            buf[idx+3] = 0   # cin=4 pad alpha = 0
            idx += 4
    return bytes(buf)


def send_inference_request(sock, ifm_bytes):
    """打包 + 发 INFER 请求 (含 IFM 上传), 收 response, 返回 (status, cy, ofm_bytes, t_breakdown).
    t_breakdown = (upload_s, recv_resp_s)"""
    assert len(ifm_bytes) == IFM_LEN, f"ifm len {len(ifm_bytes)} != {IFM_LEN}"
    req_hdr = struct.pack('<III', MAGIC, CMD_INFER, IFM_LEN)
    t_send = time.time()
    sock.sendall(req_hdr + ifm_bytes)
    t_upload_done = time.time()
    upload_s = t_upload_done - t_send

    return _recv_resp(sock, upload_s)


def send_repeat_request(sock):
    """打包 + 发 REPEAT 请求 (不带 IFM, 复用 server 上次 DDR 里的 IFM), 收 response."""
    req_hdr = struct.pack('<III', MAGIC, CMD_REPEAT, 0)
    t_send = time.time()
    sock.sendall(req_hdr)
    upload_s = time.time() - t_send   # 12 byte 的发送时间, 几乎 0
    return _recv_resp(sock, upload_s)


def _recv_resp(sock, upload_s):
    """收 16 byte resp header + OFM bytes. 返回 (status, cy, ofm, t_break)."""
    t_resp_start = time.time()
    resp_hdr = b''
    while len(resp_hdr) < 16:
        chunk = sock.recv(16 - len(resp_hdr))
        if not chunk:
            sys.exit("ERROR: VD100 提前断开 (header)")
        resp_hdr += chunk
    t_hdr_done = time.time()
    magic, status, cy, ofm_len = struct.unpack('<IIII', resp_hdr)
    if magic != MAGIC:
        sys.exit(f"ERROR: bad magic 0x{magic:08x}")
    if status != 0:
        sys.exit(f"ERROR: VD100 status = {status}")

    ofm = b''
    while len(ofm) < ofm_len:
        chunk = sock.recv(min(4096, ofm_len - len(ofm)))
        if not chunk:
            sys.exit("ERROR: VD100 提前断开 (ofm)")
        ofm += chunk
    t_recv_done = time.time()

    return status, cy, ofm, {
        "upload_s": upload_s,
        "wait_resp_s": t_hdr_done - t_resp_start,  # 收完 hdr (含 server HW 时间)
        "ofm_recv_s": t_recv_done - t_hdr_done,    # OFM 下载
    }


def parse_logit_top5(ofm_bytes):
    """522-d INT8 logit 取 top-5 (random weights, 仅 demo)."""
    logits = [int.from_bytes([b], byteorder='little', signed=True) for b in ofm_bytes]
    indexed = list(enumerate(logits))
    indexed.sort(key=lambda x: -x[1])
    return indexed[:5]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', required=True, help='PNG/JPG 图片路径')
    p.add_argument('--vd100-ip', default='169.254.111.10', help='VD100 板 IP')
    p.add_argument('--port', type=int, default=5000)
    p.add_argument('--repeat', type=int, default=10,
                   help='重复推理次数测吞吐. IFM 只上传 1 次, 后续 N-1 次走 CMD_REPEAT 复用')
    args = p.parse_args()

    print(f"[1/3] 加载图片: {args.image}")
    t0 = time.time()
    ifm = load_image_to_int8(args.image)
    print(f"  IFM 大小: {len(ifm)/1024:.1f} KB, 用时 {(time.time()-t0)*1000:.0f} ms")

    print(f"[2/3] 连接 VD100: {args.vd100_ip}:{args.port}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30.0)
    try:
        sock.connect((args.vd100_ip, args.port))
    except (socket.timeout, ConnectionRefusedError) as e:
        sys.exit(f"ERROR: 连不上 VD100 ({e})")
    print(f"  连接成功")

    print(f"[3/3] 推理 × {args.repeat}  (IFM 上传 1 次, 后续 {args.repeat-1} 次走 REPEAT)")
    cy_list = []
    rtt_list = []
    breakdown_list = []   # list of dict
    cmd_kind = []         # "INFER" or "REPEAT" 标记
    for i in range(args.repeat):
        t0 = time.time()
        if i == 0:
            status, cy, ofm, tb = send_inference_request(sock, ifm)
            cmd_kind.append("INFER")
        else:
            status, cy, ofm, tb = send_repeat_request(sock)
            cmd_kind.append("REPEAT")
        rtt = time.time() - t0
        cy_list.append(cy)
        rtt_list.append(rtt)
        breakdown_list.append(tb)
        if i == 0:
            top5 = parse_logit_top5(ofm)
            print(f"  Top-5 logit (random weights, 仅演示数据通路):")
            for rank, (idx, val) in enumerate(top5, 1):
                print(f"    #{rank}  class[{idx:3d}] = {val:+d}")

    sock.close()

    # 详细 timing 拆分
    print(f"\n=== Per-iteration breakdown ===")
    print(f"  {'#':>3}  {'cmd':<6}  {'upload':>10}  {'wait+HW':>10}  {'ofm dl':>10}  "
          f"{'RTT':>10}  {'cy':>10}")
    for i, (kind, tb, rtt, cy) in enumerate(zip(cmd_kind, breakdown_list, rtt_list, cy_list)):
        print(f"  {i:>3}  {kind:<6}  {tb['upload_s']*1000:>8.1f}ms  "
              f"{tb['wait_resp_s']*1000:>8.1f}ms  {tb['ofm_recv_s']*1000:>8.1f}ms  "
              f"{rtt*1000:>8.1f}ms  {cy:>10}")

    # 统计 (分 INFER vs REPEAT)
    infer_idxs  = [i for i,k in enumerate(cmd_kind) if k == "INFER"]
    repeat_idxs = [i for i,k in enumerate(cmd_kind) if k == "REPEAT"]
    avg_cy = sum(cy_list) / len(cy_list)

    print(f"\n=== 性能 ===")
    if infer_idxs:
        rtt_infer  = [rtt_list[i] for i in infer_idxs]
        print(f"  CMD_INFER  (含上传) RTT = {sum(rtt_infer)/len(rtt_infer)*1000:.1f} ms")
    if repeat_idxs:
        rtt_repeat = [rtt_list[i] for i in repeat_idxs]
        print(f"  CMD_REPEAT (无上传) RTT = {sum(rtt_repeat)/len(rtt_repeat)*1000:.1f} ms"
              f"  → 吞吐 {1.0/(sum(rtt_repeat)/len(rtt_repeat)):.1f} FPS")
    avg_rtt = sum(rtt_list) / len(rtt_list)
    print(f"  整体平均 RTT = {avg_rtt*1000:.1f} ms ({1.0/avg_rtt:.1f} FPS)")
    print(f"  HW cycles 平均: {avg_cy:.0f} cy ({avg_cy/100000:.2f} ms @ 100MHz)")
    if avg_cy > 0:
        print(f"  纯计算吞吐: {100000000/avg_cy:.0f} FPS @ 100MHz")


if __name__ == '__main__':
    main()
