#!/usr/bin/env python3
"""E2 可行性判定：种子维度 / 腰维度能否把左臂 arm_16 的裕度抬起来。

只读不改（不写基线、不改模块）。取 both-fixed-free 的 circle 中最坏帧，反算目标
位姿，然后：
  1. 扫多种子（中性位/随机扰动），看能否找到裕度 ≥ 0.01 rad 且 perr ≤ 0.05 mm 的解；
  2. 腰 yaw 双侧扫（W4 判据：候选是否只缺了方向），看腰能否救。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import circle_planning_7axis as m

m.configure(["--arm", "both", "--mode", "fixed", "--waist", "free"])
robot = m.Robot.from_files(m.URDF, m.SRDF)
cfg = m.build_cfg(robot, "left", m.FINGER_NAME, m.MODE, m.find_seed(robot, "left"))
jn = list(cfg.state_joint_names)
ARM_L = [f"arm_1{i}" for i in range(1, 8)]
print("state_joint_names(%d):" % len(jn), jn)

Q = np.asarray(np.load("tests/golden/both-fixed-free.npz")["circle_left_Q"])
assert Q.shape[1] == len(jn), (Q.shape, len(jn))
lo, hi = cfg.limits[0], cfg.limits[1]


def margin(q):
    return float(min(min(q[jn.index(a)] - lo[jn.index(a)],
                         hi[jn.index(a)] - q[jn.index(a)]) for a in ARM_L))


def worst_joint(q):
    ms = [(min(q[jn.index(a)] - lo[jn.index(a)], hi[jn.index(a)] - q[jn.index(a)]), a)
          for a in ARM_L]
    return min(ms)


margins = [margin(q) for q in Q]
k = int(np.argmin(margins))
qrow = np.asarray(Q[k], float)
mj, mjn = worst_joint(qrow)
print(f"最坏帧 k={k}  min margin = {mj*180/np.pi:.3f} deg  @ {mjn}")

# 目标位姿（golden 解反算，perr 基线 ~0）
robot.env.setState(jn, qrow)
T = robot.env.getState().link_transforms[cfg.tip_frame]
t = np.array(T.translation, float)
R = np.array(T.rotation, float)
pose = m.Pose.from_matrix_position(R, list(t))

ia = {a: jn.index(a) for a in ARM_L}
arm_cols = [ia[a] for a in ARM_L]
base_arm = qrow[arm_cols]


def solve_report(seed):
    sol = cfg.ik(pose, seed)
    if sol is None:
        return None
    sol = np.asarray(sol, float)
    robot.env.setState(jn, sol)
    T2 = robot.env.getState().link_transforms[cfg.tip_frame]
    e = float(np.linalg.norm(np.array(T2.translation, float) - t) * 1000.0)
    return margin(sol), e


def make_seed(arm_vals):
    s = qrow.copy()
    s[arm_cols] = arm_vals
    return s


print("\n[种子扫描] 目标裕度 >= 0.01 rad(0.57deg) 且 perr <= 0.05mm")
rng = np.random.default_rng(1)
results = []
for name, arm in [("golden", base_arm), ("neutral", np.zeros(7))]:
    for i in range(25):
        pert = rng.normal(0, 0.25, 7) if name == "golden" else rng.normal(0, 0.6, 7)
        r = solve_report(make_seed(arm + pert))
        if r is not None:
            results.append((r[0], r[1], name))
results.sort()
ok = [(mg, e, nm) for mg, e, nm in results if mg >= 0.01 and e <= 0.05]
print(f"  共 {len(results)} 次求解成功；满足(裕度>=0.57deg & perr<=0.05mm)的有 {len(ok)} 个")
for mg, e, nm in results[:5]:
    print(f"    best: margin={mg*180/np.pi:6.2f}deg  perr={e:6.3f}mm  [{nm}]")
for mg, e, nm in results[-5:]:
    print(f"    worst:margin={mg*180/np.pi:6.2f}deg  perr={e:6.3f}mm  [{nm}]")

print("\n[腰 yaw 双侧扫] base 种子 = golden arm，腰 yaw 偏离 golden 值")
yaw0 = qrow[0]
for dy in np.linspace(-0.3, 0.3, 13):
    cfg.frozen_waist_setter(np.array([yaw0 + dy, qrow[1], qrow[2]]))
    r = solve_report(make_seed(base_arm))
    tag = "OK " if (r and r[0] >= 0.01 and r[1] <= 0.05) else "   "
    if r:
        print(f"   dy={dy:+5.2f}  {tag} margin={r[0]*180/np.pi:6.2f}deg perr={r[1]:6.3f}mm")
    else:
        print(f"   dy={dy:+5.2f}  无解")
