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
        order = list(m.SIDES)
        if len(m.SIDES) == 2 and m.WAIST_ON:
            order = ["right", "left"]      # 腰归属右臂，右先左后（B1 规则）
        for side in order:
            tag = side if len(m.SIDES) > 1 else ""
            points, Q, ts, reason = m.plan_shape(robot, cfgs[side], shape,
                                                 u, v, tag=tag)
            if points is None or ts is None:
                print(f"FAIL {shape} {side}: {reason or 'TOTG 未产出时间戳'}")
                ok = False
                continue
            results[side] = (points, Q, ts)
            if (side == "right" and len(m.SIDES) == 2
                    and cfgs["left"].frozen_waist_setter is not None):
                cfgs["left"].frozen_waist_setter(m.frozen_waist_from_vars(Q[0]))
        if len(results) != len(m.SIDES):
            continue
        if len(m.SIDES) == 2 and not m.check_dual_collision(robot, cfgs,
                                                            results):
            print(f"FAIL {shape}: 双臂碰撞")
            ok = False
        frozen = {}
        for side in m.SIDES:
            fz = None
            if len(m.SIDES) == 2 and side == "left" \
                    and cfgs["left"].frozen_waist_setter is not None:
                fz = m.frozen_waist_from_vars(results["right"][1][0])
            frozen[side] = fz
        groups = {side: shape_to_group(ctx, side, shape, *results[side],
                                       frozen=frozen[side])
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
