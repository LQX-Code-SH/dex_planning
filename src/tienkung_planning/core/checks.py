"""core 产物校验三件（步 5 实现于 traj_check，v0.2 抽至 L2 供 facade.check 复用）。

check_structure / check_errors 纯 numpy + 契约层；check_contacts 需注入 Robot
（碰撞管理器走 L1 tess，ACM/裕度由 env 决定）。口径与 golden 一致。
"""
import numpy as np


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


def check_contacts(art, robot, stride):
    """全 mesh 碰撞扫描（N6：可复现接触对报告；robot 由调用方注入）。

    口径与 demo _scan_collision 一致：逐帧 setState 后必须
    setCollisionObjectsTransform 同步管理器变换缓存（否则扫的是陈旧
    状态），且只报告未被 ACM 允许的接触对（相邻指允许对不算碰撞）。
    """
    from tienkung_planning.core import tess
    margin = art.meta.options.get("collision_margin", 0.02)
    mgr = tess.contact_manager(robot.env, margin)
    req = tess.contact_request()
    acm = tess.acm(robot.env)
    colliding = {}
    for side, gp in art.groups.items():
        hits = set()
        for i in range(0, len(gp.positions), max(1, stride)):
            q = gp.positions[i]
            robot.env.setState(list(gp.joint_names), np.asarray(q, float))
            mgr.setCollisionObjectsTransform(robot.env.getState().link_transforms)
            for pair in tess.contact_test(mgr, req):
                if not acm.isCollisionAllowed(*pair):
                    hits.add(pair)
        colliding[side] = sorted(hits)
        print(f"  {side}: 碰撞对 {len(hits)} 个（裕度 {margin} m，步距 {stride}）")
        for a, b in colliding[side][:20]:
            print(f"    {a} | {b}")
    return colliding
