"""L2 模式契约：ModeConfig（字段集即契约，N1）与 ModeStrategy 协议（B7）。

ModeConfig 取代散装 cfg dict——模式差异的全部载体在此结构化：变量空间、
限位、IK 装配链（N5：DLS 主 + 回退，回退前重置状态）、ACM 采样规格
（B4 缓存键成分）、tcp/tip 帧、副指随动记录（B2）与冻腰注入（§5 会话状态）。

字段集即契约：新增/删改字段须走评审，不得静默加键。
策略只产出本结构；共享装配（限位填充等）在策略之外统一执行。
"""
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol, runtime_checkable

import numpy as np


@dataclass
class ModeConfig:
    # --- 身份 ---
    mode_name: str                        # full / fixed / tcp
    side: str                             # right / left
    group: str                            # 求解组（多指模式组名只由主指尖决定，D3）
    mode: str = ""                        # 兼容别名 = mode_name（步 4 移除）

    # --- 帧与关节 ---
    tcp_frame: str = ""                   # Descartes 跟踪帧——与 tip_frame 不同（tcp 模式二者相等）
    tip_frame: str = ""                   # 跟踪的指尖链接
    state_joint_names: list = field(default_factory=list)   # 完整状态关节名（positions 对齐的权威）
    joint_names: list = field(default_factory=list)         # 兼容别名 = state_joint_names（步 4 移除）

    # --- 变量空间 ---
    var_names: Optional[list] = None      # 变量名序列（含腰？含 prox？）
    lo: Optional[np.ndarray] = None       # 变量空间下限
    hi: Optional[np.ndarray] = None       # 变量空间上限
    seed_plan: Optional[np.ndarray] = None    # 规划种子（变量空间或全状态，随模式）

    # --- 运行时绑定（闭包工厂产物） ---
    set_state: Optional[Callable] = None      # 写入 env 的完整状态
    sample_spec: str = ""                     # ACM 采样规格标识（B4：进缓存键）
    sample_state: Optional[Callable] = None   # ACM 采样器
    ik: Optional[Callable] = None             # IK 装配语义（N5：含回退链）
    target_pose: Optional[Callable] = None    # fixed 的 tcp 换算（恒等 = None）

    # --- 几何 ---
    center: Optional[np.ndarray] = None       # 形状中心（A3 预检与 RViz marker 依赖）
    rotation: Optional[np.ndarray] = None     # (3,3) 形状朝向（指尖/ tcp 目标姿态）
    plan_rotation: Optional[np.ndarray] = None    # fixed 专属：tcp 目标姿态
    tip_offset: Optional[np.ndarray] = None       # fixed 专属：指尖相对 tcp 的常量偏移

    # --- 限位与手部 ---
    limits: Optional[tuple] = None        # (lo, hi) 全状态限位；共享装配阶段填充
    hand_joints: dict = field(default_factory=dict)   # 写入 env 的手部关节值
    other_pos: dict = field(default_factory=dict)     # 显示/固定关节值（不参与规划）

    # --- 双臂与多指扩展 ---
    frozen_waist_setter: Optional[Callable] = None    # both+free 左臂：注入冻结腰值（会话状态）
    deltas: Optional[list] = None         # B2 副指随动 [(tip_frame, delta(3,))]
    tip_frames: Optional[dict] = None     # B2 副指 tip_frame 表
    extra_fingers: Optional[list] = None  # B2 副指列表

    def __post_init__(self):
        # 兼容别名：旧消费方仍按 dict 键名访问的字段（步 4 统一移除）
        if not self.mode:
            self.mode = self.mode_name
        if not self.joint_names:
            self.joint_names = self.state_joint_names

    def get(self, key, default=None):
        """过渡期 dict 兼容：cfg.get("x") -> 属性访问；None 视为键不存在
        （可选字段在 dataclass 里以 None 表达，旧 dict 里是键缺席）（步 4 移除）。"""
        v = getattr(self, self._legacy(key), default)
        return default if v is None else v

    def __contains__(self, key):
        """过渡期 dict 兼容：`"x" in cfg` -> 属性非 None（步 4 移除）。"""
        return getattr(self, self._legacy(key), None) is not None

    def __getitem__(self, key):
        """过渡期 dict 兼容：cfg["x"] -> cfg.x（步 4 移除）。"""
        try:
            return getattr(self, self._legacy(key))
        except AttributeError:
            raise KeyError(key)

    def __setitem__(self, key, value):
        """过渡期 dict 兼容：cfg["x"] = v -> setattr（步 4 移除）。"""
        setattr(self, self._legacy(key), value)

    @staticmethod
    def _legacy(key):
        """旧 cfg dict 键名 -> ModeConfig 字段名（步 4 统一移除）。"""
        return {"ik_fn": "ik", "tcp": "tcp_frame",
                "set_frozen_waist": "frozen_waist_setter"}.get(key, key)


@runtime_checkable
class ModeStrategy(Protocol):
    """模式策略协议（B7）：策略产出结构化 ModeConfig，而非方法组。"""

    name: str

    def build(self, robot, side: str, finger: str, arm_seed: np.ndarray) -> ModeConfig:
        ...
