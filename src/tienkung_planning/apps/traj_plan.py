"""traj_plan：离线形状规划 -> PlanArtifact 产物（Q4①，封装方案 §13.1）。

用法:
    python -m tienkung_planning.apps.traj_plan --arm both --mode full \
        --waist free [--fingers index,thumb] [--shape circle|all] \
        [-o out_dir]

除 --shape/-o 外的参数原样交给管线 configure（与回归同一入口）。
产物: <out_dir>/<shape>.traj（kind="shape"，含逐侧 GroupPlan + 误差统计）。
"""
import argparse
import os
import sys

import numpy as np

from tienkung_planning.core.pipeline import (make_artifact, make_context,
                                             shape_to_group)
from tienkung_planning.contracts import artifact_hash


def main(argv=None):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--shape", default="all")
    ap.add_argument("-o", "--out", default=".")
    known, rest = ap.parse_known_args(argv)
    os.makedirs(known.out, exist_ok=True)

    ctx = make_context(rest)
    m, robot, cfgs = ctx.m, ctx.robot, ctx.cfgs
    u = m.START_DIR / np.linalg.norm(m.START_DIR)
    v = np.cross(m.PLANE_NORMAL, u)
    shapes = list(m.SHAPES) if known.shape == "all" else [known.shape]

    ok = True
    for shape in shapes:
        results = {}
        if len(m.SIDES) == 2 and m.WAIST_ON:
            # W2：腰归右臂解出，左臂逐帧跟随物理腰重解（编排内置双臂互碰校验）
            results, warns = m.plan_both_with_waist(robot, cfgs, shape, u, v)
            if results is None:
                print(f"FAIL {shape}: {warns}")
                ok = False
                continue
        else:
            for side in (["right", "left"] if len(m.SIDES) == 2
                         else list(m.SIDES)):
                tag = side if len(m.SIDES) > 1 else ""
                points, Q, ts, reason = m.plan_shape(robot, cfgs[side], shape,
                                                     u, v, tag=tag)
                if points is None or ts is None:
                    print(f"FAIL {shape} {side}: "
                          f"{reason or 'TOTG 未产出时间戳'}")
                    ok = False
                    continue
                results[side] = (points, Q, ts)
            if len(results) != len(m.SIDES):
                continue
            if len(m.SIDES) == 2 and not m.check_dual_collision(robot, cfgs,
                                                                results):
                print(f"FAIL {shape}: 双臂碰撞")
                ok = False
        # frozen_waist 记账退化：腰是两臂共享的同一组物理关节值（W2 后两臂腰列一致）
        groups = {side: shape_to_group(ctx, side, shape, *results[side],
                                       frozen=None)
                  for side in results}
        art = make_artifact(ctx, "shape", groups,
                            {s: g.ideal_path for s, g in groups.items()})
        out = os.path.join(known.out, f"{shape}.traj")
        art.save(out)
        print(f"SAVED {out}  sides={list(groups)}  "
              f"hash={artifact_hash(art)[:12]}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
