"""L3 门面：组合便捷入口 TienKungPlanner（封装方案 §7/§13.1 用法①）。

入参 = 几何 + 选项 dataclass；出参 = L0 PlanArtifact 纯数据；不抛 tesseract
类型。规划失败抛 contracts.PlannerError，选项非法抛 PlannerError（configure
的 SystemExit 在此转译）。

B6 决策（env 归属）：一个实例 = 一个 Robot/env = 一个线程；env 在
from_profile 时创建，实例内独占、无全局缓存、不跨线程共享。ACM 修复仅在
构建期执行一次（N3）。plan_a/plan_b 必须来自同一实例（profile_hash + 关节
表校验），跨实例过渡不在 v0.2 支持范围。
"""
from dataclasses import dataclass

import numpy as np

from tienkung_planning.contracts import (PlannerError, PlanArtifact,
                                         artifact_hash, profile_hash)
from tienkung_planning.core import checks
from tienkung_planning.core.pipeline import (make_artifact, make_context,
                                             make_transition_info,
                                             shape_to_group,
                                             transition_to_group)


@dataclass
class PlannerOptions:
    """会话选项（与管线 configure 的 argparse 一一对应）。"""

    arm: str = "right"                    # right | left | both
    mode: str = "full"                    # full | fixed | tcp
    waist: str = "fixed"                  # fixed | free
    finger: str = "index"                 # 主指尖（--finger-joint）
    finger_value: float = 0.6             # 主指弯曲值 (rad)（--finger）
    fingers: tuple = None                 # 多指，如 ("index", "thumb")（--fingers）
    offset: tuple = (0.0, 0.0, 0.0)       # 路径中心平移 (m)（--offset）

    def to_argv(self):
        argv = ["--arm", self.arm, "--mode", self.mode,
                "--waist", self.waist,
                "--finger-joint", self.finger,
                "--finger", repr(float(self.finger_value))]
        if self.fingers:
            argv += ["--fingers", ",".join(self.fingers)]
        if any(self.offset):
            argv += ["--offset", *[repr(float(v)) for v in self.offset]]
        return argv


@dataclass
class CheckReport:
    """planner.check 的结果：结构/误差/可选碰撞扫描的汇总。"""

    ok: bool
    errors: list
    contacts: dict = None                 # {side: [(link1, link2), ...]}，仅 contacts=True


class TienKungPlanner:
    """一键形状规划门面：几何入参 -> PlanArtifact 产物。"""

    def __init__(self, ctx, options):
        self._ctx = ctx
        self._options = options
        m = ctx.m
        # 会话几何状态（plan_shape kwargs 的持久值，确定性重置）
        self._size = float(m.SIZE)
        self._plane_normal = np.asarray(m.PLANE_NORMAL, float)
        # 手动微调：复刻 demo main 的 offset 应用（make_context 不含此步）
        if np.any(np.asarray(options.offset, float)):
            off = np.asarray(options.offset, float)
            for cfg in ctx.cfgs.values():
                cfg.center = cfg.center + off

    @classmethod
    def from_profile(cls, profile, options=None):
        """profile 名/路径 + PlannerOptions -> 规划会话。

        进程内一次一个 from_profile（demo 模块 import 期读取
        TIENKUNG_PLANNING_PROFILE，构造完成后恢复，不在会话期共享）。
        """
        options = options or PlannerOptions()
        import os
        prev = os.environ.get("TIENKUNG_PLANNING_PROFILE")
        try:
            os.environ["TIENKUNG_PLANNING_PROFILE"] = str(profile)
            try:
                ctx = make_context(options.to_argv())
            except SystemExit as e:      # configure 的 argparse/组合校验
                raise PlannerError(f"PlannerOptions 无效: {e}") from e
        finally:
            if prev is None:
                os.environ.pop("TIENKUNG_PLANNING_PROFILE", None)
            else:
                os.environ["TIENKUNG_PLANNING_PROFILE"] = prev
        return cls(ctx, options)

    # ---------------------------------------------------------------- 规划
    def plan_shape(self, name, *, size=None, plane_normal=None) -> PlanArtifact:
        """规划一种形状（circle/arc/triangle/line），返回 kind="shape" 产物。

        size (m) / plane_normal 为会话级几何：给定后成为后续调用的持久值
        （省略即沿用上次值）；plane_normal 改变时起始方向 u 向其投影正交化。
        默认 u=START_DIR 归一、v=cross(n, u)，与 demo main 逐位一致。
        """
        m, robot, cfgs = self._ctx.m, self._ctx.robot, self._ctx.cfgs
        if size is not None:
            self._size = float(size)
        if plane_normal is not None:
            n = np.asarray(plane_normal, float)
            self._plane_normal = n / np.linalg.norm(n)
        n = self._plane_normal
        m.SIZE = self._size
        m.PLANE_NORMAL = n

        u = np.asarray(m.START_DIR, float)
        u = u / np.linalg.norm(u)
        u = u - (u @ n) * n               # n ⊥ u；默认 n 下 u 数值不变
        u = u / np.linalg.norm(u)
        v = np.cross(n, u)

        results, frozen_right_q = {}, None
        order = list(m.SIDES)
        if len(m.SIDES) == 2 and m.WAIST_FREE:
            order = ["right", "left"]     # 腰归右臂：右先左后（B1 规则）
        for side in order:
            tag = side if len(m.SIDES) > 1 else ""
            points, Q, ts, reason = m.plan_shape(robot, cfgs[side], name,
                                                 u, v, tag=tag)
            if points is None or ts is None:
                raise PlannerError(
                    f"plan_shape({name!r}) {side} 失败: "
                    f"{reason or 'TOTG 未产出时间戳'}")
            results[side] = (points, Q, ts)
            if (side == "right" and len(m.SIDES) == 2
                    and cfgs["left"].frozen_waist_setter is not None):
                frozen_right_q = m.WAIST_MIRROR * Q[0][:len(m.WAIST_JOINTS)]
                cfgs["left"].frozen_waist_setter(frozen_right_q)
        if len(m.SIDES) == 2 and not m.check_dual_collision(robot, cfgs,
                                                            results):
            raise PlannerError(f"plan_shape({name!r}): 双臂碰撞")
        frozen = {}
        for side in m.SIDES:
            frozen[side] = (frozen_right_q
                            if len(m.SIDES) == 2 and side == "left"
                            and cfgs["left"].frozen_waist_setter is not None
                            else None)
        groups = {side: shape_to_group(self._ctx, side, name, *results[side],
                                       frozen=frozen[side])
                  for side in results}
        return make_artifact(self._ctx, "shape", groups,
                             {s: g.ideal_path for s, g in groups.items()})

    def plan_transition(self, plan_a, plan_b) -> PlanArtifact:
        """两个同源 shape 产物之间的过渡轨迹（kind="transition"，uniform 时间源）。"""
        m, robot, cfgs = self._ctx.m, self._ctx.robot, self._ctx.cfgs
        if plan_a.kind != "shape" or plan_b.kind != "shape":
            raise PlannerError("plan_transition 仅接受 kind='shape' 的产物")
        if (plan_a.profile_hash != profile_hash(self._ctx.profile)
                or plan_b.profile_hash != profile_hash(self._ctx.profile)):
            raise PlannerError("产物与本会话 profile 不一致（跨实例过渡不支持）")
        sides = list(m.SIDES)
        for tag, art in (("a", plan_a), ("b", plan_b)):
            if set(art.groups) != set(sides):
                raise PlannerError(f"产物 {tag} 的侧集合 {sorted(art.groups)} "
                                   f"与会话 {sorted(sides)} 不一致")
            for s in sides:
                if list(art.groups[s].joint_names) != list(
                        cfgs[s].state_joint_names):
                    raise PlannerError(
                        f"产物 {tag} {s} 关节表与会话 cfg 不一致")
        Q_from = {s: plan_a.groups[s].positions[-1] for s in sides}
        Q_to = {s: plan_b.groups[s].positions[0] for s in sides}
        frames = m.plan_transition(robot, cfgs, Q_from, Q_to)
        if frames is None:
            raise PlannerError("plan_transition: 过渡轨迹规划失败（直线与"
                               "随机路标点均不可行）")
        # 双侧帧数恒同（try_path 对各侧同步推进）；时间轴 uniform，无 TOTG 先例
        groups = {s: transition_to_group(
            self._ctx, s, frames[s],
            np.linspace(0.0, m.TRANSITION_SECONDS, len(frames[s])))
            for s in sides}
        # joint_limits 取右臂（首侧）；双侧 dof 可能不同，契约按单对存（R5 注记）
        first = cfgs[sides[0]].limits
        return make_artifact(
            self._ctx, "transition", groups, waypoints=None,
            transition_info=make_transition_info(
                artifact_hash(plan_a), artifact_hash(plan_b),
                (np.asarray(first[0], float), np.asarray(first[1], float))))

    # ---------------------------------------------------------------- 校验
    def check(self, plan, *, max_err_mm=0.1, max_offplane_mm=0.5,
              max_secondary_mm=0.5, contacts=False, stride=5) -> CheckReport:
        """离线校验（同 traj_check 口径）：结构/限位/误差（+可选全 mesh 碰撞）。"""
        errors = list(checks.check_structure(plan))
        errors += checks.check_errors(plan, max_err_mm, max_offplane_mm,
                                      max_secondary_mm)
        coll = None
        if contacts:
            coll = checks.check_contacts(plan, self._ctx.robot, stride)
            if any(coll.values()):
                errors.append("存在碰撞对（见 contacts）")
        return CheckReport(ok=not errors, errors=errors, contacts=coll)
