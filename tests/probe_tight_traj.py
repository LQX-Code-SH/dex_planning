#!/usr/bin/env python3
"""列出左臂贴限位的具体轨迹（组合 × 形状 × 最紧关节）。只读不改。"""
import glob
import os
import xml.etree.ElementTree as ET

import numpy as np

URDF = "tienkung_dex/urdf/tienkung_dex.hand.urdf"
lim = {}
for j in ET.parse(URDF).getroot().findall("joint"):
    l = j.find("limit")
    if l is not None:
        lim[j.get("name")] = (float(l.get("lower")), float(l.get("upper")))

ARM_L = [f"arm_1{i}" for i in range(1, 8)]
ARM_R = [f"arm_2{i}" for i in range(1, 8)]
WAIST = ["waist_33", "waist_32", "waist_31"]


def cols_of(n):
    return {7: (0, 7, 0), 9: (0, 7, 2), 12: (3, 7, 2)}.get(n, (0, 0, 0))


def names_for(q, side):
    nw, na, nh = cols_of(q.shape[1])
    if na == 0:
        return None
    arm = ARM_L if side == "left" else ARM_R
    nm = (WAIST if nw else []) + arm + (["<prox>", "<distal>"] if nh else [])
    return nm if len(nm) == q.shape[1] else None


def tightest(q, side):
    names = names_for(q, side)
    if names is None:
        return None
    best = None
    for i, nm in enumerate(names):
        if nm in lim:
            lo, hi = lim[nm]
            md = float(np.degrees(np.min(np.minimum(q[:, i] - lo, hi - q[:, i]))))
            if best is None or md < best[0]:
                best = (md, nm)
    return best


print("=== 左臂贴限位轨迹（最紧关节 < 5 deg）===")
print(f"{'组合':<28} {'形状':<10} {'最紧关节':<10} {'裕度(deg)':>9}")
for p in sorted(glob.glob("tests/golden/*.npz")):
    lbl = os.path.basename(p)[:-4]
    for k in np.load(p).files:
        if not k.endswith("_left_Q"):
            continue
        q = np.load(p)[k]
        shape = k[:-len("_left_Q")]
        w = tightest(q, "left")
        if w and w[0] < 5:
            print(f"{lbl:<28} {shape:<10} {w[1]:<10} {w[0]:>9.2f}")

print("\n=== 左臂 7 关节全局最紧（含腰，min < 12 deg）===")
per = {}
for p in sorted(glob.glob("tests/golden/*.npz")):
    lbl = os.path.basename(p)[:-4]
    for k in np.load(p).files:
        if not k.endswith("_left_Q"):
            continue
        q = np.load(p)[k]
        names = names_for(q, "left")
        if names is None:
            continue
        for i, nm in enumerate(names):
            if nm in lim:
                lo, hi = lim[nm]
                md = float(np.degrees(np.min(np.minimum(q[:, i] - lo, hi - q[:, i]))))
                per.setdefault(nm, []).append((md, lbl, k[:-len("_left_Q")]))
for nm in sorted(per, key=lambda n: min(x[0] for x in per[n])):
    v = per[nm]
    if min(x[0] for x in v) < 12:
        top = sorted(v)[:3]
        print(f"  {nm:<10} min={min(x[0] for x in v):6.2f}  "
              f"出现在: " + ", ".join(f"{l}/{s}({md:.1f})" for md, l, s in top))
