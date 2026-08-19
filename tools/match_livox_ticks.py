#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""图片两列时间戳 → 同 tick 雷达 base_time（GPS/MCU 域）离线匹配。

输入：
  1) livox_<时间>.txt：驱动逐帧追加的两列 —— 第1列 base_time(雷达硬钟 秒.微秒)，
     第2列 发布时刻主机UTC(秒.微秒)。与图片第二列同源（同一次 WriteTimeStamp 写出）。
  2) 图片目录（一个或多个）：文件名 <col1_ns>_<col2_ns>.jpg
     col1 = 曝光时刻转墙钟 ns；col2 = 帧到达时采样的 timeshare 第二时间戳 ns。

匹配逻辑（严格，无模糊匹配）：
  ① col2 截断到 µs 与 txt 第2列精确相等 → 唯一命中行 i → b_i。
  ② tick 归属：e = θ − Δ（Δ = u_i − col1；e = 曝光相对命中窗起点的位置，
     θ = 扫描窗起点→发布的延迟）。e∈[0,T) → b_i；e<0 → b_{i−1}（曝光早于 i 窗
     起点）；e≥T → b_{i+1}（曝光晚于 i 窗结束）。T = 相邻窗中位间隔(~100ms)。
  ③ θ 自标定：真 θ 下窗号序列对图片序列恒定 +stride 递增（5Hz图/10Hz雷达 → +2）；
     θ 偏差使边界附近图像滑窗 → 增量跳变，网格搜索取一致性最高者。
     消歧先验：评分关于 θ 以 T 为周期（θ+T 使全部分配平移一格、增量不变），
     故必须限定网格范围。物理上雷达收满一帧扫描(≥T)后才发布、组帧处理延迟 <T，
     ⟹ θ∈[T,2T)，即 θ = T + L_proc。网格限定 [T,2T) 后解唯一。

用法：
  python3 match_livox_ticks.py <livox.txt> <图片目录> [<图片目录2>...] [--out result.csv]
"""

import argparse
import csv
import os
import sys
from collections import Counter

MS = 1_000_000  # ns → ms 缩放用（内部全部用 ns 整数，避免浮点精度损失）


def parse_sec_usec(s):
    """'1768514483.683396' → ns 整数。纯整数解析，不经 float（16 位有效数字会丢精度）。"""
    s = s.strip()
    if '.' in s:
        sec, frac = s.split('.', 1)
    else:
        sec, frac = s, ''
    frac = (frac + '000000')[:6]  # 补齐 6 位 µs
    return int(sec) * 1_000_000_000 + int(frac) * 1000


def load_txt(path):
    """txt → [(b_ns, u_ns)]，严格按行序（= 发布序）。"""
    rows = []
    with open(path, 'r', encoding='utf-8') as f:
        for ln, line in enumerate(f, 1):
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                b_ns = parse_sec_usec(parts[0])
                u_ns = parse_sec_usec(parts[1])
            except ValueError:
                print(f"⚠️ 第 {ln} 行解析失败，跳过: {line!r}", file=sys.stderr)
                continue
            rows.append((b_ns, u_ns))
    if not rows:
        sys.exit(f"❌ {path} 无有效行")
    return rows


def load_images(dirs):
    """图片目录 → [(col1_ns, col2_ns, name, camera)]，按 col1 排序。"""
    imgs = []
    for d in dirs:
        cam = os.path.basename(os.path.normpath(d))
        for fn in sorted(os.listdir(d)):
            stem, ext = os.path.splitext(fn)
            if ext.lower() != '.jpg' or '_' not in stem:
                continue
            try:
                c1, c2 = (int(x) for x in stem.split('_'))
            except ValueError:
                print(f"⚠️ 文件名不符合 <ns>_<ns>.jpg，跳过: {fn}", file=sys.stderr)
                continue
            imgs.append((c1, c2, fn, cam))
    imgs.sort(key=lambda x: x[0])
    if not imgs:
        sys.exit("❌ 未找到图片（文件名需为 <col1_ns>_<col2_ns>.jpg）")
    return imgs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('txt', help='livox_*.txt 路径')
    ap.add_argument('dirs', nargs='+', help='图片目录（可多个，如 images/camera0 camera1）')
    ap.add_argument('--out', default='match_result.csv', help='输出 csv 路径')
    args = ap.parse_args()

    rows = load_txt(args.txt)
    # 第2列 → 行号。截断到 µs 边界作键（txt 本身就是 µs 精度）。
    u_map = {}
    for i, (b, u) in enumerate(rows):
        if u in u_map:
            print(f"⚠️ txt 第2列重复 {u}（行 {i}），后者覆盖前者", file=sys.stderr)
        u_map[u] = i

    # 相邻窗中位间隔作为周期 T（比硬编码 100ms 稳，容忍雷达实际 ~100.0x ms）
    diffs = sorted(rows[i + 1][0] - rows[i][0] for i in range(len(rows) - 1))
    period_ns = diffs[len(diffs) // 2]
    print(f"✅ txt {len(rows)} 行 | 雷达窗周期(中位) {period_ns / MS:.3f} ms")

    imgs = load_images(args.dirs)
    warmup, unmatched, samples = [], [], []
    for c1, c2, fn, cam in imgs:
        if c2 == 0:
            warmup.append(fn)
            continue
        i = u_map.get((c2 // 1000) * 1000)
        if i is None:
            unmatched.append(fn)
            continue
        samples.append({'col1': c1, 'col2': c2, 'fn': fn, 'cam': cam,
                        'i': i, 'u': rows[i][1], 'b_hit': rows[i][0],
                        'delta': rows[i][1] - c1})

    print(f"📷 图片共 {len(imgs)} | 精确命中 {len(samples)} | 热身(col2=0) {len(warmup)} | "
          f"未命中 {len(unmatched)}")
    if unmatched:
        print(f"   未命中示例: {unmatched[:3]}")
    if not samples:
        sys.exit("❌ 无可匹配图片（检查 txt 与录制是否同一时段）")

    # ---- θ 自标定：网格限定 [T, 2T)（= T + L_proc，L_proc 为组帧处理延迟 ∈[0,T)）， ----
    # ---- 按相机流内窗号增量一致性评分。范围不限定则 θ 与 θ±T 同分、解不唯一。    ----
    grid = [period_ns + x * 500_000 for x in range(0, int(period_ns / 500_000) + 1)]  # 0.5ms 步
    best_theta, best_score = grid[0], -1.0
    for theta in grid:
        # 按相机分流的（调整后窗号, col1）序列
        seqs = {}
        for s in samples:
            e = theta - s['delta']
            shift = e // period_ns  # floor：e<0 → −1（上一窗）；e∈[0,T) → 0；e≥T → +1（下一窗）
            widx = s['i'] + shift
            seqs.setdefault(s['cam'], []).append((s['col1'], widx))
        score = 0.0
        for seq in seqs.values():
            seq.sort()
            incs = [b - a for (_, a), (_, b) in zip(seq, seq[1:])]
            if not incs:
                continue
            mode, cnt = Counter(incs).most_common(1)[0]
            score += cnt * mode / len(samples)  # 加权：占主导的增量 × 增量值
        if score > best_score:
            best_score, best_theta = score, theta
    theta = best_theta
    print(f"✅ θ = T + L_proc = {theta / MS:.3f} ms（T = {period_ns / MS:.3f}，"
          f"组帧处理延迟 L_proc = {(theta - period_ns) / MS:.3f} ms）")

    # ---- 终分配 + 残差统计 ----
    out_rows, residuals = [], []
    seqs = {}
    for s in samples:
        e = theta - s['delta']
        shift = e // period_ns
        k = s['i'] + shift
        if not (0 <= k < len(rows)):
            print(f"⚠️ {s['fn']} 调整后行号越界，退回命中行", file=sys.stderr)
            k, shift = s['i'], 0
            e = e % period_ns
        base = rows[k][0]
        r = e - shift * period_ns  # 曝光在归属窗内的位置，应落在 [0, T)
        residuals.append(r)
        seqs.setdefault(s['cam'], []).append((s['col1'], k))
        out_rows.append({'image': s['fn'], 'camera': s['cam'],
                         'col1_ns': s['col1'], 'col2_ns': s['col2'],
                         'hit_row': s['i'] + 1, 'shift': shift,
                         'delta_ms': round(s['delta'] / MS, 3),
                         'in_window_ms': round(r / MS, 3),
                         'base_same_tick_ns': base,
                         'base_same_tick_sec': f"{base // 10**9}.{base % 10**9 // 1000:06d}"})

    # 不变量检查：各相机流窗号增量应恒定
    for cam, seq in sorted(seqs.items()):
        seq.sort()
        incs = [b - a for (_, a), (_, b) in zip(seq, seq[1:])]
        mode, cnt = Counter(incs).most_common(1)[0] if incs else (0, 0)
        print(f"   [{cam}] 窗号增量众数 +{mode}（占比 {cnt}/{len(incs)}）"
              f"{' ✅' if incs and cnt == len(incs) else ' ⚠️ 存在跳变，检查 θ 或丢帧'}")

    rs = sorted(residuals)
    n = len(rs)
    print(f"✅ 窗内残差 min/中位/max = {rs[0] / MS:.2f}/{rs[n // 2] / MS:.2f}/"
          f"{rs[-1] / MS:.2f} ms（应单簇落在 [0, {period_ns / MS:.0f}) 内）")

    with open(args.out, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        w.writeheader()
        w.writerows(out_rows)
    print(f"📝 结果已写出: {args.out}（{len(out_rows)} 行）")


if __name__ == '__main__':
    main()
