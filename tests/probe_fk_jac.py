#!/usr/bin/env python3
"""W3-① 探针 v2：验证「一次 calcFwdKin 构造解析雅可比」与现有数值雅可比等价。

只读不改：不写基线、不改模块。结论若等价，则 make_finger_ik 每次迭代可从
(1+n) 次 FK 降为 1 次 calcFwdKin（约 30x）。
"""
import os
import sys
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import circle_planning_7axis as m

# ---- URDF：joint -> (child link, axis) ----
root = ET.parse(m.URDF).getroot()
child_of, axis_of = {}, {}
for j in root.findall("joint"):
    name = j.get("name")
    child_of[name] = j.find("child").get("link")
    ax = j.find("axis")
    axis_of[name] = np.array([float(v) for v in ax.get("xyz").split()], float) \
        if ax is not None else np.array([1.0, 0.0, 0.0])

m.configure(["--arm", "both", "--mode", "fixed", "--waist", "all"])
robot = m.Robot.from_files(m.URDF, m.SRDF)
cfg = m.build_cfg(robot, "right", m.FINGER_NAME, m.MODE, m.find_seed(robot, "right"))

jn_state = list(cfg.state_joint_names)
var_names = list(cfg.var_names)
group = cfg.group
gj = list(robot.get_joint_names(group))
kg = robot.env.getKinematicGroup(group)
tip = cfg.tip_frame

print(f"group={group}  组内 {len(gj)} 关节  变量 {len(var_names)} 个: {var_names}")
for v in var_names:
    print(f"   {v:34s} -> child {child_of.get(v)}  axis {axis_of.get(v)}")


def fk_state(q_full):
    """现有做法：写入场景状态，取 tip 位姿。"""
    cfg.set_state(q_full)
    T = robot.env.getState().link_transforms[tip]
    return np.array(T.translation, float), np.array(T.rotation, float)


def num_J(q_full, var_names, h=1e-6):
    """现有做法：数值雅可比（每个变量一次 FK）。"""
    p, R = fk_state(q_full)
    J = np.zeros((6, len(var_names)))
    for i in range(len(var_names)):
        qp = q_full.copy()
        qp[jn_state.index(var_names[i])] += h
        pp, Rp = fk_state(qp)
        J[:3, i] = (pp - p) / h
        J[3:, i] = (m.rot_err(Rp, R) / h)
    return J


def ana_J(q_full, var_names):
    """新做法：一次 calcFwdKin 得全部连杆位姿，解析构造几何雅可比。"""
    d = dict(zip(jn_state, q_full))
    poses = kg.calcFwdKin(np.array([d[j] for j in gj], float))
    T = poses[tip]
    p_tip = np.array(T.translation, float)
    J = np.zeros((6, len(var_names)))
    for i, vn in enumerate(var_names):
        Tj = poses[child_of[vn]]
        Rj = np.array(Tj.rotation, float)
        pj = np.array(Tj.translation, float)
        z = Rj @ (axis_of[vn] / np.linalg.norm(axis_of[vn]))
        J[:3, i] = np.cross(z, p_tip - pj)
        J[3:, i] = z
    return J


st = robot.get_state(jn_state)
q0 = np.array([st[j] for j in jn_state], float)

print("\n[雅可比比对] 每个变量列的最大绝对差（数值 vs 解析）")
Jn, Ja = num_J(q0, var_names), ana_J(q0, var_names)
for i, vn in enumerate(var_names):
    print(f"   {vn:34s} dJv={np.abs(Jn[:3, i] - Ja[:3, i]).max():.2e}  "
          f"dJw={np.abs(Jn[3:, i] - Ja[3:, i]).max():.2e}")
print(f"   整体 max|Jn-Ja| = {np.abs(Jn - Ja).max():.3e}   量级 |Jn|max = {np.abs(Jn).max():.3e}")

# 多组随机配置抽验（含腰偏离中立位、手指弯曲）
rng = np.random.default_rng(1)
worst, worst_rel = 0.0, 0.0
for k in range(15):
    q = q0 + rng.normal(0, 0.15, len(q0))
    for i, vn in enumerate(var_names):
        lo, hi = cfg.limits[0][i], cfg.limits[1][i]
        q[jn_state.index(vn)] = np.clip(q[jn_state.index(vn)], lo, hi)
    Jn, Ja = num_J(q, var_names), ana_J(q, var_names)
    worst = max(worst, np.abs(Jn - Ja).max())
    worst_rel = max(worst_rel, np.abs(Jn - Ja).max() / max(np.abs(Jn).max(), 1e-12))
print(f"\n[随机抽验 15 组] max|Jn-Ja| = {worst:.3e}  相对 = {worst_rel:.3e}")

# ---- 端到端：单次 IK 调用耗时（现有 fk vs calcFwdKin fk）----
q_var = np.array([dict(zip(jn_state, q0))[v] for v in var_names], float)
pose_t, pose_R = fk_state(q0)
print(f"\n[配置] 变量 {len(var_names)} 个，迭代上限 {m.IK_ITERS}")

t0 = time.perf_counter()
for _ in range(200):
    num_J(q0, var_names)
t_num = (time.perf_counter() - t0) / 200 * 1e3
t0 = time.perf_counter()
for _ in range(200):
    ana_J(q0, var_names)
t_ana = (time.perf_counter() - t0) / 200 * 1e3
print(f"[速度] 数值 J = {t_num:.3f} ms/迭代    解析 J = {t_ana:.3f} ms/迭代   "
      f"加速 {t_num / t_ana:.1f}x")
print(f"[外推] 单次 IK 约 25 迭代: {t_num * 25:.2f} ms -> {t_ana * 25:.2f} ms")
