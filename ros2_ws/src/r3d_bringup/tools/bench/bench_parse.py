#!/usr/bin/env python3
"""Python microbenchmarks for the bridge's hot paths — mirrors src/bench.cpp so
the two implementations are compared apples-to-apples. stdlib only (no rclpy /
websockets needed), so it runs anywhere.

`parse_pointcloud2` is copied VERBATIM from R3D/web_a.py.
Prints machine-readable `key=value` lines matching the C++ keys.
"""

import base64
import json
import struct
import time

KPOINTS = 8000
ITEM = 32  # point_step (web_a.py hardcodes item_size = 32)


# ---- verbatim copy of web_a.py::parse_pointcloud2 (operates on raw bytes) ----
def parse_pointcloud2(data):
    points = []
    for i in range(0, len(data), ITEM):
        try:
            x, y, z, intensity = struct.unpack_from("ffff", data, i)
            points.append([x, y, z])
        except Exception:
            continue
    return points[:KPOINTS]


def make_cloud():
    """Same per-point values as src/bench.cpp so JSON sizes are identical."""
    ba = bytearray(KPOINTS * ITEM)
    for i in range(KPOINTS):
        v = (i % 1000 * 0.01, i % 800 * 0.02, i % 600 * 0.03, i % 400 * 0.1)
        struct.pack_into("ffff", ba, i * ITEM, *v)
    return bytes(ba)


def bench(fn, iters):
    fn()  # warm up
    t0 = time.perf_counter()
    for _ in range(iters):
        fn()
    return (time.perf_counter() - t0) / iters


def main():
    cloud = make_cloud()

    # parse
    per = bench(lambda: parse_pointcloud2(cloud), 200)
    print(f"parse_us_per_frame={per*1e6:.3f}")
    print(f"parse_ns_per_point={per*1e9/KPOINTS:.2f}")

    pts = parse_pointcloud2(cloud)

    # pcd json (inner data object — same shape both impls serialize)
    def pcd_json():
        return json.dumps({"t": 0.0, "points": pts})
    per = bench(pcd_json, 200)
    s = pcd_json()
    print(f"pcd_json_us_per_frame={per*1e6:.3f}")
    print(f"pcd_json_bytes={len(s)}")

    # odom json
    odom = {"t": 1.0, "x": 0.5, "y": 0.2, "z": 0.0, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}
    per = bench(lambda: json.dumps(odom), 100000)
    s = json.dumps(odom)
    print(f"odom_json_us_per_frame={per*1e6:.4f}")
    print(f"odom_json_bytes={len(s)}")

    # base64 of a JPEG-sized blob
    blob = bytes([0xAB]) * (100 * 1024)
    per = bench(lambda: base64.b64encode(blob), 500)
    s = base64.b64encode(blob)
    print(f"base64_us_per_100kb={per*1e6:.3f}")
    print(f"base64_bytes={len(s)}")


if __name__ == "__main__":
    main()
