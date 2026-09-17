#!/usr/bin/env python3
"""A 系数实验：在 both-fixed-free 最坏帧上扫 IK_LIMIT_GAIN，看限位排斥能否把 arm_16
裕度抬起来、且不牺牲位姿误差。只读不改（不写基线）。"""
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
lo, hi = cfg.limits[0], cfg.limits[1]


def margin(q):
    return float(min(min(q[jn.index(a)] - lo[jn.index(a)],
                         hi[jn.index(a)] - q[jn.index(a)]) for a in ARM_L))


Q = np.asarray(np.load("tests/golden/both-fixed-free.npz")["circle_left_Q"])
assert Q.shape[1] == len(jn)
margins = [margin(q) for q in Q]
k = int(np.argmin(margins))
qrow = np.asarray(Q[k], float)
robot.env.setState(jn, qrow)
T = robot.env.getState().link_transforms[cfg.tip_frame]
pose = m.Pose.from_matrix_position(np.array(T.rotation, float),
                                   list(np.array(T.translation, float)))
seed = qrow.copy()

print(f"最坏帧 k={k}  min margin={margins[k]*180/np.pi:.3f} deg")
print(f"{'gain':>10} {'min margin(deg)':>16} {'perr(mm)':>10} {'arm_16 margin(deg)':>18}")
for g in [0, 1e-8, 1e-7, 3e-7, 1e-6, 3e-6, 1e-5, 3e-5, 1e-4, 3e-4, 1e-3]:
    m.IK_LIMIT_GAIN = g
    sol = cfg.ik(pose, seed)
    if sol is None:
        print(f"{g:10.0e}    None")
        continue
    sol = np.asarray(sol, float)
    robot.env.setState(jn, sol)
    T2 = robot.env.getState().link_transforms[cfg.tip_frame]
    e = float(np.linalg.norm(np.array(T2.translation, float)
                             - np.array(T.translation, float)) * 1000)
    i16 = jn.index("arm_16")
    a16m = min(sol[i16] - lo[i16], hi[i16] - sol[i16])
    print(f"{g:10.0e} {margin(sol)*180/np.pi:16.3f} {e:10.3f} {a16m*180/np.pi:18.3f}")
