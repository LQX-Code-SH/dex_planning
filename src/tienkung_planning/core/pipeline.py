"""core 共享装配：管线模块定位 + 规划上下文构建 + 产物组装（步 5，v0.2 自 apps 下沉至 L2）。

管线（demo circle_planning_7axis.py）按 Q3 决策留在原仓库；定位顺序：
TIENKUNG_PLANNING_PIPELINE 显式路径 > description root 旁同目录。
产物组装只读 demo 的规划输出与既有辅助函数，不改其行为（golden 基线守护）。
"""
import datetime
import hashlib
import importlib.util
import os
import sys

from tienkung_planning.contracts import (
    GroupPlan, PathErrors, PlanArtifact, PlanMeta, SecondaryTrack,
    TransitionInfo, profile_hash)
from tienkung_planning.core.profile import (find_description_root,
                                            load_profile,
                                            resolve_description)


class Context:
    """一次 CLI 进程内的规划会话：管线模块 + 参数 + robot + 各侧 cfg。"""

    def __init__(self, m, args, robot, cfgs, profile):
        self.m, self.args, self.robot, self.cfgs = m, args, robot, cfgs
        self.profile = profile


def load_pipeline():
    """定位并导入管线模块 circle_planning_7axis。"""
    path = os.environ.get("TIENKUNG_PLANNING_PIPELINE")
    if not path:
        path = os.path.join(find_description_root(), "circle_planning_7axis.py")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"管线模块不存在: {path}（可设 TIENKUNG_PLANNING_PIPELINE 指定）")
    spec = importlib.util.spec_from_file_location("tienkung_pipeline", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def load_robot(profile=None):
    """按 profile 装载 Robot（time_param / traj_check --contacts 用）。"""
    profile = profile or load_profile()
    urdf, srdf = resolve_description(profile)
    from tienkung_planning.core.tess import Robot
    return profile, Robot.from_files(urdf, srdf)


def build_robot(profile_name=None):
    """profile 名/路径 -> (profile, Robot)；facade 能力件的独立装载入口。

    与 load_robot 同体；不经 demo 模块、不读 TIENKUNG_PLANNING_PIPELINE。
    """
    return load_robot(profile_name)


def make_context(argv):
    """复刻回归进入路径：configure → Robot（profile 装载）→ build_cfg → ACM 修复。"""
    import numpy as np
    m = load_pipeline()
    args = m.configure(argv)
    profile, robot = load_robot()

    cfgs = {}
    for side in m.SIDES:
        seed = m.find_seed(robot, side)
        if len(m.FINGER_LIST) > 1:
            cfg = m.build_cfg_multi(robot, side, m.FINGER_LIST, m.MODE, seed)
        else:
            cfg = m.build_cfg(robot, side, m.FINGER_NAME, m.MODE, seed)
        if m.MODE != "tcp":
            cfg.other_pos = {**cfg.other_pos,
                             **m.display_other_pos(side, m.FINGER_LIST)}
        cfgs[side] = cfg
    for side in m.SIDES:
        op = cfgs[side].other_pos
        if op:
            robot.env.setState(list(op), np.array([op[n] for n in op], float))
    robot.set_collision_margin(m.COLLISION_MARGIN)
    m.fix_allowed_collisions(robot, list(cfgs.values()))
    return Context(m, args, robot, cfgs, profile)


def acm_digest(robot):
    """ACM 内容 hash（N3：与 golden 同算法）。"""
    acm = robot.env.getAllowedCollisionMatrix().getAllAllowedCollisions()
    return hashlib.sha256(
        "\n".join(f"{a}|{b}|{r}" for (a, b), r in sorted(acm.items())).encode()
    ).hexdigest()


def _library_version():
    import importlib.metadata
    try:
        return importlib.metadata.version("tienkung-planning")
    except importlib.metadata.PackageNotFoundError:
        return "0.0.0.dev0+repo"


def shape_to_group(ctx, side, shape, points, Q, ts, frozen=None):
    """单侧单形状规划输出 -> GroupPlan（误差统计口径与 golden 一致）。"""
    import numpy as np
    m, robot, cfg = ctx.m, ctx.robot, ctx.cfgs[side]
    n = min(len(Q), len(points))
    tip = np.array([m.tip_pose(robot, cfg, qq)[0] for qq in Q])
    perr = np.linalg.norm(tip[:n] - points[:n], axis=1) * 1000
    dev = np.abs((tip[:n] - cfg.center) @ m.PLANE_NORMAL) * 1000
    secondary = {}
    for tf, d in (cfg.deltas or []):
        sub = []
        for qq in Q[:n]:
            cfg.set_state(qq)
            sub.append(np.array(
                robot.env.getState().link_transforms[tf].translation, float))
        serr = float(np.linalg.norm(
            np.asarray(sub) - (points[:n] + d), axis=1).max() * 1000)
        secondary[tf] = SecondaryTrack(tf, np.asarray(d, float), serr)
    return GroupPlan(
        group_name=cfg.group, joint_names=tuple(cfg.state_joint_names),
        positions=np.asarray(Q, float), timestamps=np.asarray(ts, float),
        time_source="totg", tip_frame=cfg.tip_frame,
        ideal_path=np.asarray(points, float),
        err_stats=PathErrors(float(perr.max()), float(perr.mean()),
                             float(dev.max())),
        secondary=secondary,
        frozen_waist=None if frozen is None else np.asarray(frozen, float))


def make_options(ctx):
    """会话配置 + 逐关节限位（traj_check 离线校验的数据源）+ ACM hash（N3）。"""
    import numpy as np
    m = ctx.m
    limits = {}
    for side, cfg in ctx.cfgs.items():
        if cfg.limits is None:
            continue
        lo, hi = cfg.limits
        limits[side] = {n: [float(lo[i]), float(hi[i])]
                        for i, n in enumerate(cfg.state_joint_names)}
    return {
        "arm": ctx.args.arm, "mode": m.MODE, "waist": ctx.args.waist,
        "fingers": list(m.FINGER_LIST), "finger_value": m.FINGER,
        "size": m.SIZE, "shapes": list(m.SHAPES),
        "collision_margin": m.COLLISION_MARGIN, "frame": m.FRAME,
        "acm_hash": acm_digest(ctx.robot), "joint_limits": limits,
    }


def make_artifact(ctx, kind, groups, waypoints, transition_info=None):
    return PlanArtifact(
        schema_version=1, kind=kind, profile_id=ctx.profile["robot"],
        profile_hash=profile_hash(ctx.profile),
        meta=PlanMeta(
            library_version=_library_version(),
            created_at=datetime.datetime.now(datetime.timezone.utc)
            .isoformat(timespec="seconds"),
            mode=ctx.m.MODE, options=make_options(ctx)),
        groups=groups, waypoints=waypoints, transition_info=transition_info)


def make_transition_info(source_hash, target_hash, joint_limits):
    return TransitionInfo(source_artifact_hash=source_hash,
                          target_artifact_hash=target_hash,
                          joint_limits=joint_limits)


def now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="seconds")
