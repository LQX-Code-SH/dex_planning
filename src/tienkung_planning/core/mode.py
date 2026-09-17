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

    # --- 帧与关节 ---
    tcp_frame: str = ""                   # Descartes 跟踪帧——与 tip_frame 不同（tcp 模式二者相等）
    tip_frame: str = ""                   # 跟踪的指尖链接
    state_joint_names: list = field(default_factory=list)   # 完整状态关节名（positions 对齐的权威）

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
    frozen_waist_setter: Optional[Callable] = None    # 逐帧注入物理腰（W2 左臂跟随；腰为共享关节，不镜像）
    deltas: Optional[list] = None         # B2 副指随动 [(tip_frame, delta(3,))]
    tip_frames: Optional[dict] = None     # B2 副指 tip_frame 表
    extra_fingers: Optional[list] = None  # B2 副指列表


@runtime_checkable
class ModeStrategy(Protocol):
    """模式策略协议（B7）：策略产出结构化 ModeConfig，而非方法组。"""

    name: str

    def build(self, robot, side: str, finger: str, arm_seed: np.ndarray) -> ModeConfig:
        ...
