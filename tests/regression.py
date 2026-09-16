#!/usr/bin/env python3
"""回归（headless）：单组合 = arm mode [waist] [fingers]，逐形状规划 + TOTG + 过渡 + 精度。

需 conda env `dex` + ROS Jazzy 环境（见仓库根 README 或 run_rviz_sim.sh）。

用法: phaseA_regression.py <arm> <mode> [waist] [fingers]
  fingers 形如 index,thumb（缺省 = 单指 --finger-joint 默认 index）
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
arm, mode = sys.argv[1], sys.argv[2]
waist = sys.argv[3] if len(sys.argv) > 3 else "fixed"
fingers = sys.argv[4] if len(sys.argv) > 4 else None
_argv = ["reg", "--arm", arm, "--mode", mode, "--waist", waist]
if fingers:
    _argv += ["--fingers", fingers]
import numpy as np
import circle_planning_7axis as m

# 步 0.5：真 API 配置注入，不再篡改 sys.argv（import 无副作用）
m.configure(_argv[1:])

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
        cfg["other_pos"] = {**cfg.get("other_pos", {}), **m.display_other_pos(side, m.FINGER_LIST)}
    cfgs[side] = cfg
for side in m.SIDES:
    op = cfgs[side].get("other_pos", {})
    if op:
        robot.env.setState(list(op), np.array([op[n] for n in op], float))
robot.set_collision_margin(m.COLLISION_MARGIN)
m.fix_allowed_collisions(robot, list(cfgs.values()))

u = m.START_DIR / np.linalg.norm(m.START_DIR)
v = np.cross(m.PLANE_NORMAL, u)

# A3 预检（正常 offset 应通过）
for side, cfg in cfgs.items():
    if "tip_offset" in cfg:
        R, p_off = cfg["plan_rotation"], cfg["tip_offset"]
        pose = lambda p: m.Pose.from_matrix_position(R, list(p - R @ p_off))
    else:
        R = cfg["rotation"]
        pose = lambda p: m.Pose.from_matrix_position(R, list(p))
    for d in (u, v, -u, -v):
        assert cfg["ik_fn"](pose(cfg["center"] + m.SIZE * d), cfg["seed_plan"]) is not None, \
            f"预检误报: {side} {d}"
print("PRECHECK pass")

primary = m.FINGER_LIST[0]
waist_span = {}

ok = True
prev_end = None
for shape in m.SHAPES:
    results = {}
    order = list(m.SIDES)
    if len(m.SIDES) == 2 and m.WAIST_FREE:
        order = ["right", "left"]
    for side in order:
        tag = side if len(m.SIDES) > 1 else ""
        points, Q, ts, reason = m.plan_shape(robot, cfgs[side], shape, u, v, tag=tag)
        if points is None:
            print(f"FAIL {shape} {side}: {reason}")
            ok = False
            continue
        if ts is None:
            print(f"FAIL {shape} {side}: TOTG 未产出时间戳")
            ok = False
        w = Q[:, :3] if cfgs[side]["group"].endswith(f"waist_{primary}") else None
        if w is not None:
            waist_span.setdefault(side, []).append(float(np.abs(w).max()))
        tip = np.array([m.tip_pose(robot, cfgs[side], qq)[0] for qq in Q])
        n = min(len(Q), len(points))
        perr = np.linalg.norm(tip[:n] - points[:n], axis=1).max() * 1000
        dev = np.abs((tip[:n] - cfgs[side]["center"]) @ m.PLANE_NORMAL).max() * 1000
        if perr > 0.1 or dev > 0.5:
            print(f"FAIL {shape} {side}: 路径误差 {perr:.4f}mm 离面 {dev:.4f}mm")
            ok = False
        # B2 副指随动精度
        for tf, d in cfgs[side].get("deltas", []):
            sub = []
            for qq in Q[:n]:
                cfgs[side]["set_state"](qq)
                sub.append(np.array(
                    robot.env.getState().link_transforms[tf].translation, float))
            serr = np.linalg.norm(np.asarray(sub) - (points[:n] + d), axis=1).max() * 1000
            if serr > 0.5:
                print(f"FAIL {shape} {side}: 副指 {tf} 随动误差 {serr:.4f}mm")
                ok = False
        results[side] = (points, Q, ts)
        if (side == "right" and len(m.SIDES) == 2
                and cfgs["left"].frozen_waist_setter is not None):
            cfgs["left"].frozen_waist_setter(
                m.WAIST_MIRROR * Q[0][:len(m.WAIST_JOINTS)])
    if len(results) == len(m.SIDES) == 2 and not m.check_dual_collision(robot, cfgs, results):
        print(f"FAIL {shape}: 双臂碰撞")
        ok = False
    if prev_end is not None and len(results) == len(m.SIDES):
        trans = m.plan_transition(robot, cfgs, prev_end, {s: results[s][1][0] for s in m.SIDES})
        if trans is None:
            print(f"FAIL {shape}: 过渡轨迹规划失败")
            ok = False
        else:
            print(f"过渡 {list(prev_end)} -> {shape}: OK ({len(trans[m.SIDES[0]])} 帧)")
    prev_end = {s: results[s][1][-1] for s in results} if results else None

if waist_span:
    print("WAIST span:", {s: f"{max(v):.3f} rad" for s, v in waist_span.items()})
lbl = f"{arm}/{mode}/{waist}" + (f"/{fingers}" if fingers else "")
print(f"RESULT {lbl}: " + ("ALL PASS" if ok else "FAILED"))
