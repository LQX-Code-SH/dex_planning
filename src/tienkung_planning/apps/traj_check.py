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

from tienkung_planning.contracts import PlanArtifact, artifact_hash
from tienkung_planning.core import checks


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
        errs = checks.check_structure(art) + checks.check_errors(
            art, args.max_err_mm, args.max_offplane_mm, args.max_secondary_mm)
        if args.contacts:
            from tienkung_planning.core import pipeline
            _, robot = pipeline.load_robot()
            coll = checks.check_contacts(art, robot, args.stride)
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
