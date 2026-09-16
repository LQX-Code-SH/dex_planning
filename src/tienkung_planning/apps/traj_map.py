"""traj_map：产物关节空间重映射（Q4④，跨解释器/执行子集消费）。

用法:
    python -m tienkung_planning.apps.traj_map -i in.traj -o out.traj \
        [--sides right,left] [--joints j1,j2,...] [--every N]

- --joints：按给定顺序抽取/重排关节列（执行侧控制器顺序常与本仓库不同）
- --every N：等距抽稀（保留首末帧）
来源 provenance（原 hash + 变换参数）写入 meta.options["mapped"]。
"""
import argparse
import sys

import numpy as np

from tienkung_planning.contracts import PlanArtifact, artifact_hash


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--in", required=True, dest="inp")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--sides", default=None)
    ap.add_argument("--joints", default=None)
    ap.add_argument("--every", type=int, default=1)
    args = ap.parse_args(argv)

    art = PlanArtifact.load(args.inp)
    orig_hash = artifact_hash(art)      # 变换前先取（provenance，§4.6）
    sides = args.sides.split(",") if args.sides else list(art.groups)
    joints = args.joints.split(",") if args.joints else None
    every = max(1, args.every)

    for side in sides:
        if side not in art.groups:
            print(f"FAIL: 产物无侧 {side}（现有 {list(art.groups)}）")
            return 1
        gp = art.groups[side]
        Q = np.asarray(gp.positions)
        names = list(gp.joint_names)
        if joints:
            missing = [j for j in joints if j not in names]
            if missing:
                print(f"FAIL: {side} 缺关节 {missing}")
                return 1
            idx = [names.index(j) for j in joints]
            Q = Q[:, idx]
            gp.joint_names = tuple(joints)
        if every > 1:
            Q = np.concatenate([Q[::every], Q[-1:]])
            gp.timestamps = np.concatenate(
                [np.asarray(gp.timestamps)[::every],
                 np.asarray(gp.timestamps)[-1:]])
            gp.ideal_path = np.concatenate(
                [np.asarray(gp.ideal_path)[::every],
                 np.asarray(gp.ideal_path)[-1:]])
        gp.positions = Q
        print(f"  {side}: {Q.shape[0]} 帧 × {Q.shape[1]} 关节")

    art.meta.options["mapped"] = {
        "from_artifact_hash": orig_hash,
        "sides": sides, "joints": joints, "every": every,
    }
    art.save(args.out)
    print(f"SAVED {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
