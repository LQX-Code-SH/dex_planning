"""L3 门面：按能力取用之碰撞检查 CollisionApi（封装方案 §7/§13.1 用法②）。

B6 决策：一实例一 env 一线程；env 在 from_profile 时经 core.pipeline
装载（不经 demo 模块、不读 TIENKUNG_PLANNING_PIPELINE），无全局缓存。

口径：contact_test 原样返回全部 <裕度 接触对，不做 ACM 过滤（调用方自行
判断语义）；裕度默认 0.02 m 与管线 COLLISION_MARGIN 对齐。

已知事实（v0.2 实测）：tesseract contactTest 并不抑制 ACM 允许对；demo
_scan_collision 的 isCollisionAllowed 过滤方向与其 docstring 相反（登记为
demo 待修项），同手相邻指在 2cm 裕度下天然进入报告（非物理接触）——需要
"碰撞"语义（未被 ACM 允许的对）请用 core.checks.check_contacts。
"""
import numpy as np

from tienkung_planning.core import tess
from tienkung_planning.core.pipeline import build_robot


class CollisionApi:
    """独立碰撞检查能力件：给定关节状态，返回接触对。"""

    def __init__(self, profile, robot):
        self._profile, self._robot = profile, robot

    @classmethod
    def from_profile(cls, profile="tienkung_dex", description_root=None,
                     cache_dir=None):
        profile_obj, robot = build_robot(profile,
                                         description_root=description_root,
                                         cache_dir=cache_dir)
        return cls(profile_obj, robot)

    def check(self, states, joint_names, margin=0.02):
        """检查关节状态是否碰撞。

        states: 单状态 (dof,) 或状态序列 (N, dof)；joint_names 与列对齐。
        返回：单状态 -> [(link1, link2), ...]；序列 -> 逐帧列表
        [[(link1, link2), ...], ...]（空列表 = 无碰撞）。
        """
        states = np.asarray(states, float)
        single = states.ndim == 1
        if states.ndim == 1:
            states = states[None, :]
        if states.ndim != 2 or states.shape[1] != len(joint_names):
            raise ValueError(
                f"states {states.shape} 与 joint_names {len(joint_names)} 不对齐")
        env = self._robot.env
        names = list(joint_names)
        mgr = tess.contact_manager(env, margin)
        req = tess.contact_request()
        out = []
        for q in states:
            env.setState(names, np.asarray(q, float))
            out.append(sorted(tess.contact_test(mgr, req)))
        return out[0] if single else out
