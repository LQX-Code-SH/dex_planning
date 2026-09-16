"""L3 门面：按能力取用之时间参数化 TimeParamApi（封装方案 §7/§13.1 用法②）。

B6 决策同 CollisionApi：一实例一 env 一线程，from_profile 时装载。

边界（§7/B6 显式化）：
- 需要 URDF 速度限位：任一关节 velocity <= 0 → UnavailableError
- TOTG 异常/空结果 → uniform 回退（time_source="uniform"，与管线双判定一致）
- group 缺省时按 profile 组模板 + 关节名集合精确匹配推断；无命中 → ValueError
  （能力存在、输入不可解析，不属 UnavailableError）
- 偏差记录：§7 文档写 2-tuple 返回，实现为 TimedTrajectory——uniform 回退
  必须携带 time_source，调用方才能区分时间轴来源
"""
from dataclasses import dataclass

import numpy as np

from tienkung_planning.contracts import UnavailableError
from tienkung_planning.core import tess
from tienkung_planning.core.pipeline import build_robot


@dataclass(frozen=True)
class TimedTrajectory:
    positions: np.ndarray        # (N, dof)——TOTG 会重采样，帧数可与输入不同
    timestamps: np.ndarray       # (N,) s，严格递增，首尾速度为零（totg 路径）
    time_source: str             # "totg" | "uniform"


class TimeParamApi:
    """独立时间参数化能力件：裸关节轨迹 -> 带时间戳轨迹。"""

    def __init__(self, profile, robot):
        self._profile, self._robot = profile, robot

    @classmethod
    def from_profile(cls, profile="tienkung_dex", description_root=None,
                     cache_dir=None):
        profile_obj, robot = build_robot(profile,
                                         description_root=description_root,
                                         cache_dir=cache_dir)
        return cls(profile_obj, robot)

    def _resolve_group(self, joint_names):
        """按 profile 组模板展开候选组，关节名集合精确匹配。"""
        prof = self._profile
        sides = ("right", "left")
        fingers = list(prof.get("fingers", {}))
        candidates = []
        g = prof.get("groups", {})
        if "arm" in g:
            candidates += [g["arm"].format(side=s) for s in sides]
        for key in ("full", "waist"):
            if key in g:
                candidates += [g[key].format(side=s, finger=f)
                               for s in sides for f in fingers]
        for name in candidates:
            try:
                if set(self._robot.get_joint_names(name)) == set(joint_names):
                    return name
            except Exception:        # 非组名/无插件 → 换下一个候选
                continue
        raise ValueError(
            f"joint_names 无法匹配任何组（{len(joint_names)} 关节）；"
            f"请显式传 group=")

    def parameterize(self, Q, joint_names, group=None, tip_frame=None,
                     working_frame="pelvis", dt=0.1) -> TimedTrajectory:
        """对 (N, dof) 关节轨迹做 TOTG 时间参数化。

        group 缺省 → 由 joint_names 推断；tip_frame 缺省 → 组末端链接自动
        探测。TOTG 不可用（限位缺失）抛 UnavailableError；计算失败回退
        uniform（timestamps = arange(N) * dt）。
        """
        Q = np.asarray(Q, float)
        joint_names = list(joint_names)
        if Q.ndim != 2 or Q.shape[1] != len(joint_names):
            raise ValueError(f"Q {Q.shape} 与 joint_names "
                             f"{len(joint_names)} 不对齐")
        if len(Q) < 2:
            raise ValueError("时间参数化至少需要两帧")
        robot = self._robot
        if group is None:
            group = self._resolve_group(joint_names)
        lims = robot.get_joint_limits(group)
        bad = [n for n, lim in lims.items() if lim.get("velocity", 0) <= 0]
        if bad:
            raise UnavailableError(
                f"组 {group} 的关节 {bad[:3]}{'…' if len(bad) > 3 else ''} "
                f"缺 URDF 速度限位，TOTG 不可用")
        info = robot.get_manipulator_info(group, tcp_frame=tip_frame,
                                          working_frame=working_frame)
        out = tess.totg_traj(Q, joint_names, robot.env, info, dt)
        if out is None:
            return TimedTrajectory(Q, np.arange(len(Q)) * dt, "uniform")
        Q2, ts = out
        return TimedTrajectory(np.asarray(Q2, float),
                               np.asarray(ts, float), "totg")
