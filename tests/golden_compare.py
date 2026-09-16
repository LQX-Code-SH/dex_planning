#!/usr/bin/env python3
"""golden 基线比对（入口无关判据，N8）：golden_compare.py <基准目录> <对照目录>

判据：
- NPZ 键集一致；Q / points 逐元素全等（atol 1e-12）；ts 相对容差 1e-6（重建
  program 路径的放宽量，单独标注）
- manifest：ACM hash 逐组合一致、判据阈值一致、误差统计 atol 1e-9、
  单形状耗时 ≤ 基线 1.5×（N4 性能守护）
"""
import json
import os
import sys

import numpy as np

TS_RTOL = 1e-6
POS_ATOL = 1e-12
STAT_ATOL = 1e-9
TIME_FACTOR = 1.5


def main(ref_dir, new_dir):
    ref_mf = json.load(open(os.path.join(ref_dir, "manifest.json")))
    new_mf = json.load(open(os.path.join(new_dir, "manifest.json")))
    fails = []

    if set(ref_mf["entries"]) != set(new_mf["entries"]):
        fails.append(f"组合集不一致: {set(ref_mf['entries']) ^ set(new_mf['entries'])}")

    for label, ref_e in ref_mf["entries"].items():
        new_e = new_mf["entries"].get(label)
        if new_e is None:
            continue
        if ref_e["acm_hash"] != new_e["acm_hash"]:
            fails.append(f"{label}: ACM hash 漂移")
        if ref_e["thresholds"] != new_e["thresholds"]:
            fails.append(f"{label}: 判据阈值漂移")
        for shape, ref_s in ref_e.items():
            if not isinstance(ref_s, dict) or "plan_time_s" not in ref_s:
                continue
            new_s = new_e[shape]
            if new_s["plan_time_s"] > TIME_FACTOR * max(ref_s["plan_time_s"], 1e-3):
                fails.append(f"{label}/{shape}: 耗时 {new_s['plan_time_s']}s > "
                             f"{TIME_FACTOR}×基线 {ref_s['plan_time_s']}s")
            for side, ref_d in ref_s.items():
                if not isinstance(ref_d, dict) or "perr_mm" not in ref_d:
                    continue
                new_d = new_s[side]
                for k in ("perr_mm", "offplane_mm", "frames", "joint_margin_rad"):
                    if abs(ref_d[k] - new_d[k]) > STAT_ATOL:
                        fails.append(f"{label}/{shape}/{side}: {k} "
                                     f"{ref_d[k]} -> {new_d[k]}")
                for tf, sv in ref_d.get("secondary_mm", {}).items():
                    if abs(sv - new_d["secondary_mm"][tf]) > STAT_ATOL:
                        fails.append(f"{label}/{shape}/{side}: 副指 {tf} {sv} -> "
                                     f"{new_d['secondary_mm'][tf]}")
            if ref_s.get("transition_frames") != new_s.get("transition_frames"):
                fails.append(f"{label}/{shape}: 过渡帧数 "
                             f"{ref_s.get('transition_frames')} -> "
                             f"{new_s.get('transition_frames')}")

    ref_npz = {f for f in os.listdir(ref_dir) if f.endswith(".npz")}
    new_npz = {f for f in os.listdir(new_dir) if f.endswith(".npz")}
    if ref_npz != new_npz:
        fails.append(f"NPZ 文件集不一致: {ref_npz ^ new_npz}")
    for f in sorted(ref_npz & new_npz):
        a = np.load(os.path.join(ref_dir, f))
        b = np.load(os.path.join(new_dir, f))
        if set(a.files) != set(b.files):
            fails.append(f"{f}: 键集不一致 {set(a.files) ^ set(b.files)}")
            continue
        for k in a.files:
            x, y = a[k], b[k]
            if k.endswith("_ts"):
                ok = np.allclose(x, y, rtol=TS_RTOL, atol=0.0)
            else:
                ok = np.allclose(x, y, rtol=0.0, atol=POS_ATOL)
            if not ok:
                d = np.max(np.abs(x - y))
                fails.append(f"{f}:{k} 数值漂移 max|Δ|={d:.3e}")

    if fails:
        print(f"BASELINE COMPARE: FAILED（{len(fails)} 项）")
        for s in fails[:30]:
            print("  " + s)
        return 1
    print("BASELINE COMPARE: ALL MATCH")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1], sys.argv[2]))
