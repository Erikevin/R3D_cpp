#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""图片两列时间戳 → 同 tick 雷达 base_time（GPS/MCU 域）离线匹配。

输入：
  1) livox_<时间>.txt：驱动逐帧追加的两列 —— 第1列 base_time(雷达硬钟 秒.微秒)，
     第2列 发布时刻主机UTC(秒.微秒)。与图片/时间戳csv第二列同源（同一次 WriteTimeStamp 写出）。
  2) 每相机一个输入源（可多个）：
     - cameraX.csv：每行 col1,col2（无表头，与图片文件名两列同义）；行号=原文件物理行号；
     - 或图片目录：文件名 <col1_ns>_<col2_ns>.jpg。

输出：
  - <每个 csv 输入>_aligned.csv（本脚本主交付）：每行两列 ——
      第1列 = 原图片在 cameraX.csv 里的行号；第2列 = 同 tick 的 livox GPS 时间戳
      （与 txt 第1列同格式 秒.微秒）。**只保留左右相机能对齐同一 tick 的图片**（tick 交集）。
  - --out 详细诊断 csv（每图：命中行/shift/Δ/窗内残差/base 等）+ 终端不变量统计。

匹配逻辑（严格，无模糊匹配）：
  ① col2 截断到 µs 与 txt 第2列精确相等 → 唯一命中行 i → b_i。
  ② tick 归属：e = θ − Δ（Δ = u_i − col1；e = 曝光相对命中窗起点的位置，
     θ = 扫描窗起点→发布的延迟）。e∈[0,T) → b_i；e<0 → b_{i−1}（曝光早于 i 窗
     起点）；e≥T → b_{i+1}（曝光晚于 i 窗结束）。T = 相邻窗中位间隔(~100ms)。
  ③ θ 自标定：真 θ 下窗号序列对图片序列恒定 +stride 递增（10Hz图/10Hz雷达 → +1）；
     θ 偏差使边界附近图像滑窗 → 增量跳变，网格搜索取一致性最高者。
     消歧先验：评分关于 θ 以 T 为周期（θ+T 使全部分配平移一格、增量不变），
     故必须限定网格范围。物理上雷达收满一帧扫描(≥T)后才发布、组帧处理延迟 <T，
     ⟹ θ∈[T,2T)，即 θ = T + L_proc。网格限定 [T,2T) 后解唯一。

用法：
  python3 match_livox_ticks.py <livox.txt> camera0.csv camera1.csv [--out diag.csv]
  python3 match_livox_ticks.py <livox.txt> images/camera0 images/camera1 [--out diag.csv]
"""

import argparse
import csv
import os
import sys
from collections import Counter, defaultdict

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


def load_sources(paths):
    """输入源 → [entry]。csv 文件（col1,col2/行）或图片目录（文件名/序号）。
    entry: col1, col2, cam(流名), row(原行号), src(输入路径), is_csv。"""
    items = []
    for p in paths:
        if os.path.isdir(p):
            cam = os.path.basename(os.path.normpath(p))
            names = []
            for fn in sorted(os.listdir(p)):
                stem, ext = os.path.splitext(fn)
                if ext.lower() != '.jpg' or '_' not in stem:
                    continue
                try:
                    c1, c2 = (int(x) for x in stem.split('_'))
                except ValueError:
                    print(f"⚠️ 文件名不符合 <ns>_<ns>.jpg，跳过: {fn}", file=sys.stderr)
                    continue
                names.append((c1, c2, fn))
            for idx, (c1, c2, fn) in enumerate(sorted(names), 1):
                items.append({'col1': c1, 'col2': c2, 'cam': cam, 'row': idx,
                              'fn': fn, 'src': p, 'is_csv': False})
        else:
            cam = os.path.splitext(os.path.basename(p))[0]
            with open(p, 'r', encoding='utf-8') as f:
                for ln, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.replace(',', ' ').split()
                    try:
                        c1, c2 = int(parts[0]), int(parts[1])
                    except (ValueError, IndexError):
                        print(f"⚠️ {p} 第 {ln} 行解析失败，跳过: {line!r}", file=sys.stderr)
                        continue
                    items.append({'col1': c1, 'col2': c2, 'cam': cam, 'row': ln,
                                  'fn': f'line{ln}', 'src': p, 'is_csv': True})
    items.sort(key=lambda x: (x['col1'], x['cam']))
    if not items:
        sys.exit("❌ 输入源无有效条目（csv 需每行 col1,col2；目录需 <ns>_<ns>.jpg）")
    return items


def fmt_base(b_ns):
    """base_time → 与 txt 第1列完全一致的 '秒.微秒' 文本。"""
    return f"{b_ns // 10**9}.{b_ns % 10**9 // 1000:06d}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('txt', help='livox_*.txt 路径')
    ap.add_argument('sources', nargs='+',
                    help='每相机一个：cameraX.csv 或图片目录（可多个）')
    ap.add_argument('--out', default='match_result.csv', help='详细诊断 csv 路径')
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

    items = load_sources(args.sources)
    warmup, unmatched, samples = [], [], []
    for it in items:
        if it['col2'] == 0:
            warmup.append(it)
            continue
        i = u_map.get((it['col2'] // 1000) * 1000)
        if i is None:
            unmatched.append(it)
            continue
        it.update({'i': i, 'u': rows[i][1], 'delta': rows[i][1] - it['col1']})
        samples.append(it)

    print(f"📷 共 {len(items)} 条 | 精确命中 {len(samples)} | 热身(col2=0) {len(warmup)} | "
          f"未命中 {len(unmatched)}")
    if unmatched:
        print(f"   未命中示例: {[x['fn'] for x in unmatched[:3]]}")
    if not samples:
        sys.exit("❌ 无可匹配条目（检查 txt 与录制是否同一时段）")

    # ---- θ 自标定：网格限定 [T, 2T)（= T + L_proc，L_proc 为组帧处理延迟 ∈[0,T)）， ----
    # ---- 按相机流内窗号增量一致性评分。范围不限定则 θ 与 θ±T 同分、解不唯一。    ----
    grid = [period_ns + x * 500_000 for x in range(0, int(period_ns / 500_000) + 1)]  # 0.5ms 步
    best_theta, best_score = grid[0], -1.0
    for theta in grid:
        seqs = {}
        for s in samples:
            e = theta - s['delta']
            shift = e // period_ns  # floor：e<0 → −1（上一窗）；e∈[0,T) → 0；e≥T → +1（下一窗）
            seqs.setdefault(s['cam'], []).append((s['col1'], s['i'] + shift))
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
    out_rows, residuals, overflow, assigned = [], [], [], []
    seqs = {}
    for s in samples:
        e = theta - s['delta']
        shift = e // period_ns
        k = s['i'] + shift
        if not (0 <= k < len(rows)):
            # 归属窗超出 txt 范围（典型：录像尾段雷达先停，本帧所属窗尚未/不再发布）。
            # 真 tick 不在 txt 里，退回命中行会给出错一个周期的 base —— 剔除。
            overflow.append(s)
            continue
        base = rows[k][0]
        r = e - shift * period_ns  # 曝光在归属窗内的位置，应落在 [0, T)
        residuals.append(r)
        seqs.setdefault(s['cam'], []).append((s['col1'], k))
        s['base'] = base
        s['shift'] = shift
        s['in_window_ms'] = round(r / MS, 3)
        assigned.append(s)
        out_rows.append({'row': s['row'], 'image': s['fn'], 'camera': s['cam'],
                         'col1_ns': s['col1'], 'col2_ns': s['col2'],
                         'hit_row': s['i'] + 1, 'shift': shift,
                         'delta_ms': round(s['delta'] / MS, 3),
                         'in_window_ms': round(r / MS, 3),
                         'base_same_tick_ns': base,
                         'base_same_tick': fmt_base(base)})

    # ---- 偏移 tick 打印：归属窗相对精确命中行的偏移数 ----
    # shift=0 → 同 tick 即命中行；+1 → 曝光晚于命中窗结束（该帧到达时尚未发布）；
    # −1 → 曝光早于命中窗起点。分布应集中在 0，±1 少量属边界相位滑动的正常现象。
    for cam in sorted({s['cam'] for s in assigned}):
        grp = [s for s in assigned if s['cam'] == cam]
        sc = Counter(s['shift'] for s in grp)
        dist = ' '.join(f"shift{k:+d}:{v}({v * 100 / len(grp):.1f}%)" for k, v in sorted(sc.items()))
        print(f"   [{cam}] 偏移tick分布 → {dist}")
        for s in [x for x in grp if x['shift'] != 0][:5]:
            print(f"      例: 行{s['row']} shift={s['shift']:+d} "
                  f"Δ={s['delta'] / MS:.1f}ms 窗内位置={s['in_window_ms']:.1f}ms")

    if overflow:
        print(f"⚠️ {len(overflow)} 条归属窗超出 txt 范围（多为尾段雷达先停），已剔除："
              f"{[s['fn'] for s in overflow[:6]]}{'...' if len(overflow) > 6 else ''}")

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

    # ---- 左右同 tick 对齐：tick 交集 → 各相机 <stem>_aligned.csv ----
    cam_srcs = {}
    tick_of_cam = defaultdict(set)
    for s in assigned:
        cam_srcs.setdefault(s['cam'], s['src'])
        tick_of_cam[s['cam']].add(s['base'])
    cams = sorted(tick_of_cam)
    if len(cams) < 2:
        print("⚠️ 只有一个相机流，无法做左右 tick 交集，跳过 _aligned.csv 输出")
    else:
        aligned = set.intersection(*(tick_of_cam[c] for c in cams))
        print(f"🔗 同 tick 对齐：相机间 tick 交集 {len(aligned)} 个"
              f"（各相机 tick 数：{'/'.join(str(len(tick_of_cam[c])) for c in cams)}）")
        for cam in cams:
            kept = [s for s in assigned if s['cam'] == cam and s['base'] in aligned]
            kept.sort(key=lambda s: s['row'])
            dup = len(kept) - len({s['base'] for s in kept})
            if dup:
                print(f"   ⚠️ [{cam}] 有 {dup} 张与同相机其他图片同 tick（均保留）")
            if not cam_srcs[cam].endswith('.csv'):
                print(f"   ⚠️ [{cam}] 输入是图片目录，_aligned.csv 写到当前目录")
                out_path = f"{cam}_aligned.csv"
            else:
                out_path = os.path.splitext(cam_srcs[cam])[0] + '_aligned.csv'
            with open(out_path, 'w', encoding='utf-8') as f:
                for s in kept:
                    f.write(f"{s['row']},{fmt_base(s['base'])}\n")
            print(f"📝 [{cam}] {out_path}：{len(kept)} 行（原行号,同tick GPS时间戳）")

    with open(args.out, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        w.writeheader()
        w.writerows(out_rows)
    print(f"📝 详细诊断: {args.out}（{len(out_rows)} 行）")


if __name__ == '__main__':
    main()
