#!/usr/bin/env python3
"""W5 离线量化：从 golden 基线的 Q + URDF 限位，算各关节到限位的最小裕度。

腕关节贴限位是纯运动学现象，无需真机：Q 已在 tests/golden/*.npz，限位在 URDF。
只读不改（不写基线、不改模块）。

列布局（由实际列数与内置校验确定）：
    7  = arm7
    9  = arm7 + 手2
    12 = 腰3 + arm7 + 手2
校验项：腰列左右应逐元素相等（W2 后物理腰共享、不镜像）；手指两列应成比例。
"""
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
HAND = ["<prox>", "<distal>"]

print("左臂关节限位（deg）:")
for n in ARM_L:
    lo, hi = lim[n]
    print(f"   {n:10s} [{np.degrees(lo):8.2f}, {np.degrees(hi):8.2f}]")


def cols_of(n):
    """列数 -> (腰数, 臂数, 手数)。"""
    return {7: (0, 7, 0), 9: (0, 7, 2), 12: (3, 7, 2)}.get(n, (0, 0, 0))


def names_for(q, side):
    nw, na, nh = cols_of(q.shape[1])
    if na == 0:
        return None
    nm = (WAIST if nw else []) + (ARM_L if side == "left" else ARM_R) \
        + (HAND if nh else [])
    return nm if len(nm) == q.shape[1] else None


# ---- 映射校验 ----
print("\n[校验] 左右腰列是否逐元素相等（W2 后物理腰共享、不镜像）：")
for p in sorted(glob.glob("tests/golden/*.npz")):
    lbl = os.path.basename(p)[:-4]
    d = np.load(p)
    for k in d.files:
        if not k.endswith("_left_Q"):
            continue
        ql = d[k]
        qr = d.get(k.replace("_left_", "_right_"))
        if qr is None or cols_of(ql.shape[1])[0] == 0:
            continue
        dmax = float(np.abs(np.asarray(ql[:, :3]) - np.asarray(qr[:, :3])).max())
        print(f"   {lbl:<30} {k[:-2]:<10} max|左腰-右腰| = {dmax:.2e}")
        break

print("\n[校验] 手指两列是否成比例（distal = mult*prox）：")
for p in sorted(glob.glob("tests/golden/*.npz")):
    lbl = os.path.basename(p)[:-4]
    d = np.load(p)
    for k in d.files:
        if not k.endswith("_left_Q"):
            continue
        q = np.asarray(d[k])
        nw, na, nh = cols_of(q.shape[1])
        if nh == 0:
            continue
        a, b = q[:, nw + 6], q[:, nw + 7]
        r = b / np.where(np.abs(a) > 1e-6, a, np.nan)
        print(f"   {lbl:<30} {k[:-2]:<10} cols{nw + 6},{nw + 7} 比值 "
              f"{np.nanmin(r):.3f}..{np.nanmax(r):.3f}")
        break

# ---- 裕度统计 ----
rows, per_combo = [], {}
for p in sorted(glob.glob("tests/golden/*.npz")):
    lbl = os.path.basename(p)[:-4]
    for k in np.load(p).files:
        if not k.endswith("_Q"):
            continue
        q = np.load(p)[k]
        shape, side = k[:-2].rsplit("_", 1)
        names = names_for(q, side)
        if names is None:
            print(f"   [跳过] {lbl} {k} 列数 {q.shape[1]} 未识别")
            continue
        for i, nm in enumerate(names):
            if nm not in lim:
                continue
            lo, hi = lim[nm]
            md = float(np.degrees(np.min(np.minimum(q[:, i] - lo, hi - q[:, i]))))
            rows.append((md, lbl, shape, side, nm))
            per_combo.setdefault((lbl, side, nm), []).append(md)

rows.sort()
print(f"\n共 {len(rows)} 项（覆盖 {len({r[1] for r in rows})} 个组合），最小裕度前 15：")
print(f"{'裕度(deg)':>10}  {'组合':<30} {'形状':<9} {'侧':<6} 关节")
for md, lbl, shape, side, nm in rows[:15]:
    print(f"{md:10.2f}  {lbl:<30} {shape:<9} {side:<6} {nm}")

print("\n各关节全局最小裕度（<12 deg 才列）：")
per = {}
for md, lbl, shape, side, nm in rows:
    per.setdefault((side, nm), []).append(md)
for (side, nm), v in sorted(per.items(), key=lambda x: min(x[1])):
    if min(v) < 12:
        print(f"   {side:<6} {nm:<10} min={min(v):6.2f}  中位={np.median(v):6.2f}  "
              f"共 {len(v)} 项")

print("\n左臂 7 关节逐组合最小裕度（deg）：")
print(f"{'组合':<30} " + " ".join(f"{n:>8}" for n in ARM_L))
for p in sorted(glob.glob("tests/golden/*.npz")):
    lbl = os.path.basename(p)[:-4]
    best = {}
    for k in np.load(p).files:
        if not k.endswith("_left_Q"):
            continue
        q = np.load(p)[k]
        names = names_for(q, "left")
        if names is None:
            continue
        for i, nm in enumerate(names):
            if nm in ARM_L:
                lo, hi = lim[nm]
                md = float(np.degrees(np.min(np.minimum(q[:, i] - lo, hi - q[:, i]))))
                best[nm] = min(best.get(nm, 1e9), md)
    if best:
        print(f"{lbl:<30} " + " ".join(
            ("%8.2f" % best[n]) if n in best else "       -" for n in ARM_L))
