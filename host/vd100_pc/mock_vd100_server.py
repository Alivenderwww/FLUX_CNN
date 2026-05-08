#!/usr/bin/env python3
"""
mock_vd100_server.py — VD100 本地测试 mock server

模拟 VD100 板的 PS Linux server (resnet11_server.c) 在 Windows / Linux 本地跑,
让 resnet11_client.py 能 standalone 测试 (没有 VD100 板时验证 client 协议).

跟 host/vd100_ps/resnet11_server.c 协议完全一致, 但:
- 不真做推理 (没 PL, 也没 DDR4)
- 收 IFM 后等价于"假推理": cy=190258 (fake), ofm=随机 522 byte (跟硬件 random
  weights × IFM 输出值域类似, demo 协议用)

用法:
  Terminal 1 (mock server):
    python mock_vd100_server.py
    # [mock] listen on 0.0.0.0:5000

  Terminal 2 (client, 跟真 VD100 一样的命令, IP 改 127.0.0.1):
    python resnet11_client.py --image gradient.png --vd100-ip 127.0.0.1
"""
import socket
import struct
import sys
import random
import argparse

MAGIC = 0x46584E4E   # 'FXNN'
CMD_INFER = 1
IFM_LEN_EXPECTED = 960 * 540 * 4   # 2,073,600
OFM_LEN = 522
FAKE_CY = 190258                    # 跟 sim ResNet11 N=4 SIMD 一致 (mock 数据)


def recv_all(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def handle_client(cfd, addr):
    print(f"[mock] {addr} connected")
    try:
        while True:
            hdr = recv_all(cfd, 12)
            if hdr is None:
                break
            magic, cmd, ifm_len = struct.unpack('<III', hdr)
            if magic != MAGIC:
                print(f"[mock] bad magic 0x{magic:08x}")
                break
            if cmd != CMD_INFER:
                print(f"[mock] unsupported cmd {cmd}")
                break
            print(f"[mock] INFER req, ifm_len={ifm_len}")
            ifm = recv_all(cfd, ifm_len)
            if ifm is None:
                print("[mock] truncated ifm")
                break

            # 假装推理: 生成 522 byte 随机 logit (-128..127)
            ofm = bytes(random.randint(-128, 127) % 256 for _ in range(OFM_LEN))

            resp = struct.pack('<IIII', MAGIC, 0, FAKE_CY, OFM_LEN) + ofm
            cfd.sendall(resp)
            print(f"[mock] resp sent (cy={FAKE_CY}, ofm={OFM_LEN}B)")
    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        cfd.close()
        print(f"[mock] {addr} disconnected")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=5000)
    p.add_argument('--bind', default='0.0.0.0')
    args = p.parse_args()

    sfd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sfd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sfd.bind((args.bind, args.port))
    sfd.listen(1)
    print(f"[mock] listen on {args.bind}:{args.port}")
    print(f"[mock] 模拟 VD100 板 PS server, fake cy={FAKE_CY}, ofm_len={OFM_LEN}")
    print(f"[mock] client cmd: python resnet11_client.py --image XX.png --vd100-ip 127.0.0.1")

    try:
        while True:
            cfd, addr = sfd.accept()
            handle_client(cfd, addr)
    except KeyboardInterrupt:
        print("\n[mock] shutdown")
        sfd.close()


if __name__ == '__main__':
    main()
