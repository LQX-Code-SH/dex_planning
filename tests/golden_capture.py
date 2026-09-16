#!/usr/bin/env python3
"""步 0 golden 基线捕捉（单组合）：<arm> <mode> [waist] [fingers]

对给定组合逐形状规划，落 tests/golden/<label>.npz（Q/ts/points 逐元素）+
tests/golden/entries/<label>.json（误差统计、耗时、ACM hash、裕度）。
判据口径见 docs/实施计划.md 步 0（入口无关判据，N8）。

过渡期通过 sys.argv 进入模块（步 0.5 改为真 API 调用后重写本段）。
"""
import hashlib
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
arm, mode = sys.argv[1], sys.argv[2]
waist = sys.argv[3] if len(sys.argv) > 3 else "fixed"
fingers = sys.argv[4] if len(sys.argv) > 4 else None
_argv = ["gold", "--arm", arm, "--mode", mode, "--waist", waist]
if fingers:
    _argv += ["--fingers", fingers]
import numpy as np
import circle_planning_7axis as m

# 步 0.5：真 API 配置注入，不再篡改 sys.argv（import 无副作用）
m.configure(_argv[1:])

GOLDEN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden")
os.makedirs(os.path.join(GOLDEN_DIR, "entries"), exist_ok=True)

THRESHOLDS = {"perr_mm": 0.1, "offplane_mm": 0.5, "secondary_mm": 0.5}

# ---- 与 regression.py 相同的进入路径 ----
robot = m.Robot.from_files(m.URDF, m.SRDF)
cfgs = {}
for side in m.SIDES:
    seed = m.find_seed(robot, side)
    if len(m.FINGER_LIST) > 1:
        cfg = m.build_cfg_multi(robot, side, m.FINGER_LIST, m.MODE, seed)
    else:
        cfg = m.build_cfg(robot, side, m.FINGER_NAME, m.MODE, seed)
    # 限位已由 build_cfg 共享装配统一填充（_finalize_limits）
    if m.MODE != "tcp":
        cfg.other_pos = {**cfg.other_pos, **m.display_other_pos(side, m.FINGER_LIST)}
    cfgs[side] = cfg
for side in m.SIDES:
    op = cfgs[side].other_pos
    if op:
        robot.env.setState(list(op), np.array([op[n] for n in op], float))
robot.set_collision_margin(m.COLLISION_MARGIN)
m.fix_allowed_collisions(robot, list(cfgs.values()))

# ---- ACM 内容 hash（B4/N3）----
acm = robot.env.getAllowedCollisionMatrix().getAllAllowedCollisions()
acm_hash = hashlib.sha256(
    "\n".join(f"{a}|{b}|{r}" for (a, b), r in sorted(acm.items())).encode()
).hexdigest()

u = m.START_DIR / np.linalg.norm(m.START_DIR)
v = np.cross(m.PLANE_NORMAL, u)

# A3 预检
for side, cfg in cfgs.items():
    if cfg.tip_offset is not None:
        R, p_off = cfg.plan_rotation, cfg.tip_offset
        pose = lambda p: m.Pose.from_matrix_position(R, list(p - R @ p_off))
    else:
        R = cfg.rotation
        pose = lambda p: m.Pose.from_matrix_position(R, list(p))
    for d in (u, v, -u, -v):
        assert cfg.ik(pose(cfg.center + m.SIZE * d), cfg.seed_plan) is not None, \
            f"预检误报: {side} {d}"

primary = m.FINGER_LIST[0]
arrays = {}          # npz 内容
entry = {"thresholds": THRESHOLDS, "acm_hash": acm_hash, "precheck": "pass"}
waist_span = {}
ok = True
prev_end = None

for shape in m.SHAPES:
    results = {}
    shape_entry = {}
    order = list(m.SIDES)
    if len(m.SIDES) == 2 and m.WAIST_FREE:
        order = ["right", "left"]
    t0 = time.perf_counter()
    for side in order:
        tag = side if len(m.SIDES) > 1 else ""
        points, Q, ts, reason = m.plan_shape(robot, cfgs[side], shape, u, v, tag=tag)
        if points is None or ts is None:
            print(f"FAIL {shape} {side}: {reason or 'TOTG 未产出时间戳'}")
            ok = False
            continue
        key = f"{shape}_{side}"
        arrays[key + "_Q"] = Q
        arrays[key + "_ts"] = ts
        arrays[key + "_points"] = points
        tip = np.array([m.tip_pose(robot, cfgs[side], qq)[0] for qq in Q])
        n = min(len(Q), len(points))
        perr = float(np.linalg.norm(tip[:n] - points[:n], axis=1).max() * 1000)
        dev = float(np.abs((tip[:n] - cfgs[side].center) @ m.PLANE_NORMAL).max() * 1000)
        sub_errs = {}
        for tf, d in (cfgs[side].deltas or []):
            sub = []
            for qq in Q[:n]:
                cfgs[side].set_state(qq)
                sub.append(np.array(
                    robot.env.getState().link_transforms[tf].translation, float))
            serr = float(np.linalg.norm(
                np.asarray(sub) - (points[:n] + d), axis=1).max() * 1000)
            sub_errs[tf] = serr
            if serr > THRESHOLDS["secondary_mm"]:
                print(f"FAIL {shape} {side}: 副指 {tf} 随动误差 {serr:.4f}mm")
                ok = False
        if perr > THRESHOLDS["perr_mm"] or dev > THRESHOLDS["offplane_mm"]:
            print(f"FAIL {shape} {side}: 路径误差 {perr:.4f}mm 离面 {dev:.4f}mm")
            ok = False
        lo, hi = cfgs[side].limits
        margin = float(min((Q - lo).min(), (hi - Q).min()))
        w = Q[:, :3] if cfgs[side].group.endswith(f"waist_{primary}") else None
        if w is not None:
            waist_span.setdefault(side, []).append(float(np.abs(w).max()))
        shape_entry[side] = {"perr_mm": perr, "offplane_mm": dev,
                             "secondary_mm": sub_errs, "joint_margin_rad": margin,
                             "frames": int(len(Q)), "totg": True}
        results[side] = (points, Q, ts)
        if (side == "right" and len(m.SIDES) == 2
                and cfgs["left"].frozen_waist_setter is not None):
            cfgs["left"].frozen_waist_setter(
                m.WAIST_MIRROR * Q[0][:len(m.WAIST_JOINTS)])
    shape_entry["plan_time_s"] = round(time.perf_counter() - t0, 3)
    if len(results) == len(m.SIDES) == 2 and not m.check_dual_collision(robot, cfgs, results):
        print(f"FAIL {shape}: 双臂碰撞")
        ok = False
        shape_entry["dual_collision"] = True
    if prev_end is not None and len(results) == len(m.SIDES):
        trans = m.plan_transition(robot, cfgs, prev_end,
                                  {s: results[s][1][0] for s in m.SIDES})
        if trans is None:
            print(f"FAIL {shape}: 过渡轨迹规划失败")
            ok = False
        else:
            shape_entry["transition_frames"] = int(len(trans[m.SIDES[0]]))
    entry[shape] = shape_entry
    prev_end = {s: results[s][1][-1] for s in results} if results else None

if waist_span:
    entry["waist_span_rad"] = {s: max(v) for s, v in waist_span.items()}
label = f"{arm}-{mode}-{waist}" + (f"-{fingers.replace(',', '_')}" if fingers else "")
entry["label"] = label
entry["result"] = "ALL PASS" if ok else "FAILED"
entry["nanobind"] = __import__("importlib.metadata", fromlist=["version"]) \
    .version("tesseract-robotics-nanobind")
entry["numpy"] = __import__("importlib.metadata", fromlist=["version"]) \
    .version("numpy")

np.savez_compressed(os.path.join(GOLDEN_DIR, label + ".npz"), **arrays)
with open(os.path.join(GOLDEN_DIR, "entries", label + ".json"), "w",
          encoding="utf-8") as f:
    json.dump(entry, f, ensure_ascii=False, indent=1)
print(f"GOLDEN {label}: {entry['result']}")
