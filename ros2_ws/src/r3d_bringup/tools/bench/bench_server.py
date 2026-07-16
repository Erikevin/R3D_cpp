#!/usr/bin/env python3
"""End-to-end WS load client: connect to a bridge, receive for T seconds, report
delivered frame rates, end-to-end latency (wall receive time minus the frame's
ROS stamp `t`), and throughput. stdlib-only WebSocket client (with fragment
reassembly). Used by run_benchmark.sh for the Tier-2 comparison.

Usage: bench_server.py HOST PORT SECS
"""

import base64
import json
import os
import socket
import struct
import sys
import time


class WsClient:
    def __init__(self, host, port, path="/data"):
        self.host, self.port, self.path = host, port, path

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            f"GET {self.path} HTTP/1.1\r\nHost: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        ).encode())
        data = b""
        while b"\r\n\r\n" not in data:
            data += self.sock.recv(4096)
        assert b"101" in data.split(b"\r\n")[0], "upgrade failed"
        self.sock.settimeout(1.0)

    def _ex(self, n):
        b = b""
        while len(b) < n:
            c = self.sock.recv(n - len(b))
            if not c:
                raise RuntimeError("closed")
            b += c
        return b

    def _send(self, op, pl=b""):
        m = os.urandom(4)
        h = bytearray([0x80 | op])
        ln = len(pl)
        if ln < 126:
            h.append(0x80 | ln)
        elif ln < 65536:
            h.append(0x80 | 126); h += struct.pack(">H", ln)
        else:
            h.append(0x80 | 127); h += struct.pack(">Q", ln)
        self.sock.sendall(bytes(h) + m + bytes(b ^ m[i % 4] for i, b in enumerate(pl)))

    def recv(self):
        buf = bytearray()
        while True:
            h = self._ex(2)
            fin, op = bool(h[0] & 0x80), h[0] & 0x0F
            masked, ln = bool(h[1] & 0x80), h[1] & 0x7F
            if ln == 126:
                ln = struct.unpack(">H", self._ex(2))[0]
            elif ln == 127:
                ln = struct.unpack(">Q", self._ex(8))[0]
            mask = self._ex(4) if masked else None
            chunk = self._ex(ln) if ln else b""
            if masked:
                chunk = bytes(b ^ mask[i % 4] for i, b in enumerate(chunk))
            if op == 0x8:
                return None
            if op == 0x9:
                self._send(0xA, chunk); continue
            if op == 0xA:
                continue
            buf += chunk
            if fin:
                return bytes(buf)


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8765
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 5

    c = WsClient(host, port)
    c.connect()

    odom = pcd = 0
    total_bytes = 0
    lat = []
    t0 = time.time()
    while time.time() - t0 < secs:
        raw = c.recv()
        if raw is None:
            break
        total_bytes += len(raw)
        try:
            m = json.loads(raw.decode())
        except Exception:
            continue
        t = m.get("type")
        if t == "odom":
            odom += 1
        elif t == "pcd":
            pcd += 1
            ft = m.get("data", {}).get("t")
            if ft is not None:
                lat.append((time.time() - ft) * 1000.0)
    dt = time.time() - t0
    c.sock.close()

    lat.sort()
    def pct(p):
        return lat[min(len(lat) - 1, int(len(lat) * p))] if lat else float("nan")

    print(f"delivered_pcd_hz={pcd/dt:.2f}")
    print(f"delivered_odom_hz={odom/dt:.2f}")
    print(f"pcd_latency_p50_ms={pct(0.50):.2f}")
    print(f"pcd_latency_p95_ms={pct(0.95):.2f}")
    print(f"throughput_kbs={total_bytes/dt/1024:.1f}")


if __name__ == "__main__":
    main()
