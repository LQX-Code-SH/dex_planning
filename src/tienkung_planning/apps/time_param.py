"""time_param：对产物轨迹做 TOTG 时间参数化（Q4②，评审补项）。

用法:
    python -m tienkung_planning.apps.time_param -i in.traj [-o out.traj] [--force]

对 time_source="uniform" 的组重跑 TOTG（需要 URDF velocity 限位——本仓库
tienkung_dex.hand.urdf 已在阶段 0 补齐；边界：其他机型须自带 velocity）。
--force 可对已是 totg 的组重参数化。默认输出峰值速度供上机前核对
（URDF 速度限位边界由执行侧执行器兜底）。
"""
import argparse
import os
import sys

import numpy as np

from tienkung_planning.apps import _pipeline
from tienkung_planning.contracts import PlanArtifact
from tienkung_planning.core import tess


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--in", required=True, dest="inp")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args(argv)

    art = PlanArtifact.load(args.inp)
    _, robot = _pipeline.load_robot()
    frame = art.meta.options.get("frame", "pelvis")
    changed = False
    for side, gp in art.groups.items():
        if gp.time_source == "totg" and not args.force:
            print(f"SKIP {side}: 已是 totg（--force 重做）")
            continue
        info = robot.get_manipulator_info(gp.group_name,
                                          tcp_frame=gp.tip_frame,
                                          working_frame=frame)
        out = tess.totg_traj(gp.positions, gp.joint_names, robot.env, info,
                             dt=args.dt)
        if out is None:
            print(f"FAIL {side}: TOTG 失败，保留原时间戳")
            continue
        Q, ts = out
        gp.positions = Q
        gp.timestamps = ts
        gp.time_source = "totg"
        dt = np.diff(ts)
        peak = float((np.abs(np.diff(Q, axis=0)) / dt[:, None]).max()) \
            if len(Q) > 1 else 0.0
        print(f"OK {side}: 时长 {ts[-1]:.2f}s  帧数 {len(Q)}  "
              f"峰值关节速度 {peak:.2f} rad/s")
        changed = True

    if not changed:
        print("NOTHING CHANGED")
        return 0
    out = args.out or args.inp
    art.save(out)
    print(f"SAVED {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
