#!/usr/bin/env python3
"""Minimal stdlib-only WebSocket client for verifying r3d_ws_bridge.

Uses only the Python standard library (python3-websockets is NOT installed here).
Supports the subset needed to exercise the bridge: text frames, the 126/127
extended-length forms (pcd frames are ~0.5 MB), and a timeout-bounded recv.

Usage:
  ws_client_test.py command HOST PORT        # deviceInfo + getStorage + GET_FILE_LIST
  ws_client_test.py stream   HOST PORT SECS  # count odom/pcd frames over SECS seconds
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
        self.host = host
        self.port = port
        self.path = path
        self.sock = None

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(req.encode())
        # read the HTTP handshake response (ends with \r\n\r\n)
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("handshake closed")
            data += chunk
        if b"101" not in data.split(b"\r\n")[0]:
            raise RuntimeError("upgrade failed: " + data.split(b"\r\n")[0].decode(errors="replace"))
        self.sock.settimeout(5)

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise RuntimeError("socket closed")
            buf += chunk
        return buf

    def send_json(self, obj):
        payload = json.dumps(obj).encode()
        header = bytearray([0x81])  # FIN + text
        mask = os.urandom(4)
        ln = len(payload)
        if ln < 126:
            header.append(0x80 | ln)
        elif ln < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", ln)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", ln)
        header += mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)

    def _send_frame(self, opcode, payload=b""):
        # Client→server frames must be masked.
        mask = os.urandom(4)
        header = bytearray([0x80 | opcode])
        ln = len(payload)
        if ln < 126:
            header.append(0x80 | ln)
        elif ln < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", ln)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", ln)
        header += mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)

    def recv_json(self, timeout=None):
        """Read one logical message, reassembling continuation frames, answering pings."""
        if timeout is not None:
            self.sock.settimeout(timeout)
        payload = bytearray()
        try:
            while True:
                h = self._recv_exact(2)
                b0, b1 = h[0], h[1]
                fin = bool(b0 & 0x80)
                opcode = b0 & 0x0F
                masked = bool(b1 & 0x80)
                length = b1 & 0x7F
                if length == 126:
                    length = struct.unpack(">H", self._recv_exact(2))[0]
                elif length == 127:
                    length = struct.unpack(">Q", self._recv_exact(8))[0]
                mask = self._recv_exact(4) if masked else None
                chunk = self._recv_exact(length) if length else b""
                if masked:
                    chunk = bytes(b ^ mask[i % 4] for i, b in enumerate(chunk))
                if opcode == 0x8:    # close
                    return None
                if opcode == 0x9:    # ping → pong
                    self._send_frame(0xA, chunk)
                    continue
                if opcode == 0xA:    # pong
                    continue
                payload += chunk      # text(1) / binary(2) / continuation(0)
                if fin:
                    return json.loads(bytes(payload).decode())
        except (socket.timeout, RuntimeError):
            return None

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "command"
    host = sys.argv[2] if len(sys.argv) > 2 else "127.0.0.1"
    port = int(sys.argv[3]) if len(sys.argv) > 3 else 8765

    c = WsClient(host, port)
    c.connect()

    if mode == "command":
        # deviceInfo is pushed on connect
        m = c.recv_json(timeout=5)
        print("connect-push:", "deviceInfo" if m and m.get("type") == "deviceInfo" else "FAIL", m)
        assert m and m.get("type") == "deviceInfo", "expected deviceInfo on connect"
        assert "storageTotal" in m["data"], "deviceInfo missing fields"

        c.send_json({"cmd": "getStorage"})
        m = c.recv_json(timeout=5)
        print("getStorage:", "ok" if m and m.get("type") == "deviceInfo" else "FAIL", m)
        assert m and m.get("type") == "deviceInfo"

        c.send_json({"type": "GET_FILE_LIST"})
        m = c.recv_json(timeout=5)
        print("GET_FILE_LIST:", "ok" if m and m.get("code") == 200 else "FAIL",
              "entries=%d" % len(m.get("data", [])) if m else "?")
        assert m and m.get("code") == 200
        print("\nCOMMAND PATH: OK")
    elif mode == "stream":
        secs = int(sys.argv[4]) if len(sys.argv) > 4 else 3
        odom = pcd = other = 0
        t0 = time.time()
        while time.time() - t0 < secs:
            m = c.recv_json(timeout=1)
            if m is None:
                continue
            t = m.get("type")
            if t == "odom":
                odom += 1
            elif t == "pcd":
                pcd += 1
                npts = len(m.get("data", {}).get("points", []))
            else:
                other += 1
        print(f"over {secs}s: odom={odom} pcd={pcd} other={other}")
        assert pcd > 0, "no pcd frames received"
        assert odom > 0, "no odom frames received"
        print("DATA PATH: OK (pcd points/frame=%d)" % npts)
    else:
        print("unknown mode:", mode, file=sys.stderr)
        sys.exit(2)

    c.close()


if __name__ == "__main__":
    main()
