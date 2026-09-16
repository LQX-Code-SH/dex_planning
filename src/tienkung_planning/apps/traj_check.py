"""traj_check：产物离线校验（Q4③；M1 对账口径：Biview 用本工具产出报告）。

用法:
    python -m tienkung_planning.apps.traj_check -i a.traj [b.traj ...]
        [--max-err-mm 0.1] [--max-offplane-mm 0.5] [--max-secondary-mm 0.5]
        [--contacts] [--stride 5]

不装 tesseract 也能跑结构/限位/误差校验；--contacts 追加全 mesh 碰撞扫描
（需 tesseract）。输出含输入 artifact_hash（N6：可复现报告）。
"""
import argparse
import sys

import numpy as np

from tienkung_planning.contracts import PlanArtifact, artifact_hash


def check_structure(art):
    errs = []
    for side, gp in art.groups.items():
        Q = np.asarray(gp.positions)
        if Q.ndim != 2 or Q.shape[1] != len(gp.joint_names):
            errs.append(f"{side}: positions {Q.shape} 与 joint_names "
                        f"{len(gp.joint_names)} 不对齐")
        ts = np.asarray(gp.timestamps)
        if len(ts) != len(Q):
            errs.append(f"{side}: timestamps {len(ts)} != 帧数 {len(Q)}")
        if len(ts) > 1 and not (np.diff(ts) > 0).all():
            errs.append(f"{side}: 时间戳非严格递增")
        if ts[0] < 0:
            errs.append(f"{side}: 首时间戳为负")
        if gp.time_source not in ("totg", "uniform"):
            errs.append(f"{side}: time_source 非法 {gp.time_source}")
        if art.kind == "transition" and art.transition_info is None:
            errs.append("transition 产物缺 transition_info")
        if art.kind == "shape" and len(gp.joint_names) == 0:
            errs.append(f"{side}: 空关节表")
        # 限位（meta.options.joint_limits，traj_plan 写入）
        jl = (art.meta.options.get("joint_limits") or {}).get(side)
        if jl:
            lo = np.array([jl[n][0] for n in gp.joint_names])
            hi = np.array([jl[n][1] for n in gp.joint_names])
            if Q.shape[1] == len(lo):
                over = float(min((Q - lo).min(), (hi - Q).min()))
                if over < 0:
                    errs.append(f"{side}: 关节越限位（最深 {-over:.4f} rad）")
                else:
                    print(f"  {side}: 关节裕度 +{over:.3f} rad")
    return errs


def check_errors(art, max_err, max_off, max_sec):
    errs = []
    for side, gp in art.groups.items():
        if gp.err_stats.max_mm > max_err:
            errs.append(f"{side}: 主指误差 {gp.err_stats.max_mm:.4f} > {max_err} mm")
        if gp.err_stats.max_offplane_mm > max_off:
            errs.append(f"{side}: 离面 {gp.err_stats.max_offplane_mm:.4f} > {max_off} mm")
        for name, sec in gp.secondary.items():
            if sec.max_err_mm > max_sec:
                errs.append(f"{side}: 副指 {name} 随动 {sec.max_err_mm:.4f} "
                            f"> {max_sec} mm")
    return errs


def check_contacts(art, stride):
    """全 mesh 碰撞扫描（N6：可复现接触对报告，附输入 hash）。"""
    from tienkung_planning.core import pipeline
    from tienkung_planning.core import tess
    _, robot = pipeline.load_robot()
    margin = art.meta.options.get("collision_margin", 0.02)
    mgr = tess.contact_manager(robot.env, margin)
    req = tess.contact_request()
    colliding = {}
    for side, gp in art.groups.items():
        hits = set()
        op = art.meta.options
        for i in range(0, len(gp.positions), max(1, stride)):
            q = gp.positions[i]
            robot.env.setState(list(gp.joint_names), np.asarray(q, float))
            for pair in tess.contact_test(mgr, req):
                hits.add(pair)
        colliding[side] = sorted(hits)
        print(f"  {side}: 碰撞对 {len(hits)} 个（裕度 {margin} m，步距 {stride}）")
        for a, b in colliding[side][:20]:
            print(f"    {a} | {b}")
    return colliding


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", nargs="+", required=True)
    ap.add_argument("--max-err-mm", type=float, default=0.1)
    ap.add_argument("--max-offplane-mm", type=float, default=0.5)
    ap.add_argument("--max-secondary-mm", type=float, default=0.5)
    ap.add_argument("--contacts", action="store_true")
    ap.add_argument("--stride", type=int, default=5)
    args = ap.parse_args(argv)

    all_ok = True
    for path in args.input:
        art = PlanArtifact.load(path)
        print(f"== {path}  hash={artifact_hash(art)[:12]}  "
              f"kind={art.kind}  profile={art.profile_id}")
        errs = check_structure(art) + check_errors(
            art, args.max_err_mm, args.max_offplane_mm, args.max_secondary_mm)
        if args.contacts:
            coll = check_contacts(art, args.stride)
            if any(coll.values()):
                errs.append("存在碰撞对（见上方清单）")
        if errs:
            all_ok = False
            for e in errs:
                print(f"  FAIL: {e}")
    print("TRAJ CHECK: " + ("ALL PASS" if all_ok else "FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
