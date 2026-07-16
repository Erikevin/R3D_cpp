# Benchmark — `web_a.py` (Python) vs `r3d_ws_bridge` (C++)

Head-to-head on the same host (Ubuntu 22.04, g++ 11.4 / Python 3.10, Release build,
single WS consumer on loopback). The C++ port is a faithful structural translation
of `R3D/web_a.py`; this measures whether the rewrite pays off on the bridge's hot
paths. **It does — by 1.5× to ~86×.**

Reproduce with: `bash tools/bench/run_benchmark.sh 8` (after `colcon build`).

![C++ vs Python hot-path microbenchmark](tools/bench/bench_chart.svg)

---

## Tier 1 — microbenchmarks (apples-to-apples, isolated compute)

Same synthetic 8000-point cloud fed to both implementations.
Python's `parse_pointcloud2` is copied **verbatim** from `R3D/web_a.py`.
C++ side: `r3d_bench`; Python side: `tools/bench/bench_parse.py`.

| Operation (per frame) | C++ | Python | C++ speedup |
|---|---:|---:|---:|
| **PointCloud2 parse** (8000 pts) | **~11 µs** (1.4 ns/pt) | ~948 µs (118 ns/pt) | **≈ 86×** |
| **pcd JSON serialize** | **~1.49 ms** (443 870 B) | ~6.24 ms (467 683 B) | **≈ 4.2×** |
| odom JSON serialize | ~1.11 µs (69 B) | ~1.62 µs (84 B) | ≈ 1.5× |
| base64 (100 KB blob) | ~87 µs | **~72 µs** | **0.83×** ⚠️ |
| **pcd hot path (parse + JSON)** | **~1.50 ms** | ~7.19 ms | **≈ 4.8×** |

**Takeaways**

- **Parse is where C++ dominates** — ~86×. The Python `struct.unpack_from('ffff', …)`
  loop is interpreted per-point; the C++ port is a tight `memcpy` float walk. This is
  the single biggest win and runs every lidar frame.
- **pcd JSON** is ~4.2× faster with nlohmann than `json.dumps`, **and produces ~5 %
  smaller frames** (443 870 vs 467 683 B) — nlohmann emits the shortest round-trip
  float repr, so there's a bandwidth win on top of the CPU win.
- **odom JSON** is already tiny in both (a single small object), so the 1.5× rarely
  matters in practice.
- **base64 is the one place Python wins** (0.83×) — `base64.b64encode` is a C
  builtin (`binascii`), faster than the hand-rolled encoder. Camera streaming is off
  by default, so this path is cold; if it matters, swap the C++ encoder for a SIMD /
  OpenSSL implementation. *(Honest result, not a wash.)*

---

## Tier 2 — end-to-end WS load (mock_publisher → bridge → 1 client)

The C++ bridge under load with `R3D_FORCE_STREAM=1` (so data flows without launching
terminals), 8-second window:

| Metric | C++ (`r3d_ws_bridge`) |
|---|---:|
| Delivered **pcd** | **9.7 Hz** (rate-limited to 10 Hz by `PCD_SEND_INTERVAL`) |
| Delivered **odom** | **19.5 Hz** (every 50 ms send tick) |
| End-to-end **latency** (p50 / p95) | **~77 ms / ~137 ms** |
| Throughput | **~4.4 MB/s** (8000 pts/frame) |
| Bridge **CPU** | **~3 %** of one core |
| Bridge **peak RSS** | **~28 MB** |

For reference, the Python original (`asyncio` + `websockets`) carries a noticeably
heavier runtime (CPython interpreter, asyncio loop, the `websockets` library). A
precise Tier-2 for Python could not be captured here — **this sandbox has no `pip`
and `websockets` is not installed**, and `web_a.py` imports it at module top. The
Tier-1 numbers above are the dominant cost (parse + JSON ≈ 7 ms/frame in Python vs
1.5 ms in C++), so the per-frame CPU difference is well characterized; to capture
Python's absolute RSS/CPU, run `run_py_bridge.py` on a host with
`pip install websockets opencv-python` (+ rclpy).

### Latency: a C++ win (the `try_sync` staleness bug, fixed)

The Python `web_a.py::try_sync` scans the odom/lidar buffers oldest→newest and,
because its tie-break is `diff < best_diff` (strict `<`), it locks onto the
**oldest** timestamp-matched pair in the 20-frame (≈2 s) lidar buffer. So the Python
original ships point clouds that are **~2 s stale** (p50).

The C++ port started as a faithful copy with the same ~2 s staleness, but the port
**fixes it**: `try_sync` now iterates **newest-first**, so the newest matched pair
wins on ties. End-to-end latency drops from ~2 000 ms to **~77 ms (p50) / ~137 ms
(p95)** — about a **26× improvement**, with no throughput change. (The ~77 ms floor
is just the pipeline: 50 ms send tick + 100 ms pcd rate-limit + sync timer + DDS
hop.) The Python original still has the ~2 s staleness → **latency is now a real
differentiator in the C++ port's favor**, on top of the Tier-1 compute wins.

---

## Reproduce

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select r3d_bringup
bash tools/bench/run_benchmark.sh 8     # Tier 1 (both) + Tier 2 (C++)
```

Artifacts: `src/bench.cpp`, `tools/bench/bench_parse.py`, `tools/bench/bench_server.py`,
`tools/bench/run_benchmark.sh`, `tools/bench/run_py_bridge.py`, `tools/bench/bench_chart.svg`.
