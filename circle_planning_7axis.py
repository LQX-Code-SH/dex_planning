#!/usr/bin/env python3
"""
天工 Dex 双臂双手 —— 垂直平面内走形状 demo：tesseract 实时规划 + RViz2 显示
==========================================================================
支持形状：圆 / 圆弧 / 三角形 / 直线（末端姿态固定，尺寸 10cm）。

手臂选择（--arm）：right（默认）/ left / both。
    --arm both：双臂镜像协同——两手在同一个 y-z 竖直平面各画一份，左手画
    右手的镜像（y→−y），各臂独立规划，规划完做左右臂碰撞校验。左臂的种子
    与路径中心不手工标定：枚举 2^7 符号模式用 FK 验证"左臂位姿 ≈ M·右臂位姿"
    （M = diag(1,−1,1)）求出种子，中心/姿态按镜像变换得出。

手指选择（--finger-joint）：index（默认）/ middle / ring / pinky / thumb。
    手指都是欠驱动耦合指：1 个主动自由度（proximal），distal 按 URDF
    <mimic> 系数跟随（与 hand_bridge._FOLLOWER 一致：四指 1.155、拇指 1.0；
    thumb_rot 无耦合、是独立关节，规划时固定在 0）。tesseract 不认 mimic：
    KDL 求解器把 distal 当独立关节，env.setState 也不会自动驱动它，所以
    IK 里把耦合显式参数化（臂 7 + 指 1 变量，distal ≡ mult*proximal），
    并且所有状态写入都显式带上 distal。

模式三选一（--mode）：
    full   臂 7 关节 + 所选手指 1 个耦合自由度全链规划，指尖走轨迹。
    fixed  手指先固定到 --finger 值，手臂 7-DOF 规划。指尖相对 tcp 的偏移
           在手指固定后是常量，目标位姿按 tcp_target = tip_target ∘ T_off⁻¹
           换算后走原 {side}_arm 规划链路。
    tcp    原 demo 行为：{side}_tcp_link 走轨迹（回归用）。

腰部冗余（--waist，B1）：fixed（默认）/ free。
    free 时腰 3 关节并入变量空间（full/fixed 模式；tcp 恒为 fixed），改善
    关节裕度与 --offset 可达范围。腰 3 关节是左右臂共享的同一组物理关节，
    --arm both 时腰只归属右臂变量空间，左臂在右臂解出的腰值（镜像换算）上
    冻结规划。

多指联动（--fingers index,thumb，B2）：手型刚体随动。
    参与手指（第一个是主指尖）prox 全部固定在 --finger 值、distal 按耦合跟随，
    手是刚体，由臂带着走主指尖路径；目标姿态固定 ⇒ 副指尖相对主指尖的世界系
    偏移是常量，"手型保持、平行随动"严格成立。灵巧手自由度有限，副指不做
    位置约束（超定大概率无解），只做 <0.5mm 随动验证。多指时统一按 fixed
    手指语义规划（full 的主指 prox 变量会破坏刚体假设），仅支持 full/fixed
    模式。

单进程跑完：tesseract 现场规划笛卡尔轨迹 → RViz2 里让机器人动起来，
并边动边画出指尖的历史轨迹。**不生成任何中间文件**。

启动方式（脚本自己管 ROS2 环境，见 run_rviz_sim.sh）：

    ./run_rviz_sim.sh                              # 右臂食指，full 模式，circle
    ./run_rviz_sim.sh triangle --mode fixed        # fixed / tcp；形状 arc / line
    ./run_rviz_sim.sh --arm both --mode full       # 双臂镜像协同
    ./run_rviz_sim.sh --arm left --finger-joint thumb   # 左手拇指
    ./run_rviz_sim.sh --finger 0.9                 # 手指弯曲程度 (rad)
    ./run_rviz_sim.sh --waist free                 # 腰 3 关节并入变量空间
    ./run_rviz_sim.sh --fingers index,thumb        # 多指刚体联动（手型保持平行随动）

发布的话题：
    /joint_states            关节状态（驱动 robot_state_publisher 的 TF）
    /tcp_history[_left]      历史轨迹（每帧累加，每轮重置；--arm both 时左臂
                             用 _left 后缀，单臂模式一律用无后缀话题）
    /path_ideal[_left]       理想路径
    /path_markers[_left]     中心点 / 平面法向 / 文字 / 失败警告
"""

import argparse
import itertools
import math
import xml.etree.ElementTree as ET

import numpy as np

from tesseract_robotics.planning import Robot
from tesseract_robotics.planning.transforms import Pose

# ROS 2（消息类不含 C 扩展，可以放心放模块顶层；rclpy 的 C 扩展在 tesseract 之后导入）
import rclpy
from geometry_msgs.msg import Point, Pose as PoseMsg, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------------------------------------------------- 参数
URDF = "/home/qingxiangliu/work/dex_planning/tienkung_dex/urdf/tienkung_dex.hand.urdf"
SRDF = "/home/qingxiangliu/work/dex_planning/tienkung_dex/urdf/tienkung_dex.srdf"

FRAME = "pelvis"          # tesseract 的 KinematicGroup 把 base 上溯到机器人根链接

SIZE = 0.10               # 形状尺寸 (m)：圆/圆弧半径、三角形外接圆半径、直线半长
N_POINTS = 72             # 路径采样点数

# 起始角度方向 θ=0 指向 +Y，路径所在的竖直平面法向 +X（即 y-z 平面）
START_DIR = np.array([0.0, 1.0, 0.0])
PLANE_NORMAL = np.array([1.0, 0.0, 0.0])

# 右臂 tcp 模式（原 demo）的路径中心与固定末端姿态（pelvis 系），以及落在 tcp 路径
# 起点上的起始关节位姿。这是整套标定的基准：左臂全部量由它镜像得出，full/fixed
# 模式的中心/姿态改由种子位姿的指尖 FK 得出，保证起点本身可达。
CENTER_TCP = np.array([0.12739162693933648, -0.33978233066150054, 0.04771404763425319])
ROTATION_TCP = np.array([
    [0.638872841814646, 0.4856700875016278, -0.5966289115504204],
    [-0.7527020228476247, 0.23432822811083073, -0.6152478738781081],
    [-0.15900049305826772, 0.8421489462183037, 0.515270798309615],
])
SEED_ARM = np.array([0.9848669018707299, -0.864589217331047, -0.3953022998890723,
                     -2.133457534342359, -2.4875267243865338, 0.5466924922330669,
                     -0.2419232346704452])

# 左右镜像：位置 p' = M·p，旋转 R' = M·R·M（机器人左右对称，左臂 = 右臂镜像）
MIRROR = np.diag([1.0, -1.0, 1.0])
CENTER_TCP_L = MIRROR @ CENTER_TCP
ROTATION_TCP_L = MIRROR @ ROTATION_TCP @ MIRROR

# ---------------------------------------------------------------- 侧 / 手指表
# 臂组名是 SRDF 里定义的 chain 组；全链组 {side}_arm_{finger} 的插件配置见
# tienkung_dex_plugins.yaml。左臂种子不用手工标定，见 find_seed()。
ARM_GROUP = {"right": "right_arm", "left": "left_arm"}
TCP_LINK = {"right": "right_tcp_link", "left": "left_tcp_link"}

# 手指耦合表（与 hand_bridge._FOLLOWER / joint_state_frontend._FOLLOWER 同一套系数）：
# mult = distal/proximal（URDF <mimic multiplier>），upper = proximal 限位 (rad)。
# thumb 的 metacarpal（thumb_rot）无耦合、是独立关节，规划时固定在 0。
FINGERS = {
    "index":  dict(mult=1.155, upper=1.41),
    "middle": dict(mult=1.155, upper=1.41),
    "ring":   dict(mult=1.155, upper=1.41),
    "pinky":  dict(mult=1.155, upper=1.41),
    "thumb":  dict(mult=1.0,   upper=1.03, mc_joint="{side}_thumb_metacarpal_joint"),
}


def prox_joint(side, finger):
    return f"{side}_{finger}_proximal_joint"


def distal_joint(side, finger):
    return f"{side}_{finger}_distal_joint"


def tip_link(side, finger):
    return f"{side}_{finger}_tip_link"


def full_group(side, finger):
    return f"{side}_arm_{finger}"


def waist_group(side, finger):
    """含腰 3 关节的全链组（--waist free 用，SRDF/插件见 tienkung_dex.srdf）。"""
    return f"{side}_arm_waist_{finger}"


DURATION = 8.0            # 每种形状的播放时长 (s)：TOTG 真实时长按比例映射到此窗口
RATE = 50.0               # 发布频率 (Hz)
SHAPE_REPEAT = 2          # 每种形状连续播放次数（TOTG 真实时长 < DURATION，多播几遍）
TRANSITION_SECONDS = 2.0  # 形状间过渡轨迹时长 (s)
COLLISION_MARGIN = 0.02   # 碰撞安全裕度 (m)

# 耦合 IK（DLS）参数
IK_ITERS = 80
IK_LAM = 1e-2             # 阻尼系数
IK_TOL = 1e-5             # 收敛残差 (位置 m / 姿态 rad 混合范数)
IK_TOL_MAX = 5e-4         # 迭代用尽时的接受上限
IK_MAX_STEP = 0.3         # 单步关节增量限幅 (rad)


def _parse_args():
    ap = argparse.ArgumentParser(description="天工 Dex 双臂双手形状 demo")
    ap.add_argument("shapes", nargs="*", help="circle / arc / triangle / line，缺省全轮换")
    ap.add_argument("--arm", choices=["right", "left", "both"], default="right",
                    help="right=右臂(默认), left=左臂, both=双臂镜像协同")
    ap.add_argument("--finger-joint", choices=list(FINGERS), default="index",
                    help="参与规划的手指（默认食指）")
    ap.add_argument("--fingers", type=str, default=None,
                    help="逗号分隔多指（第一个为主指尖），如 index,thumb；"
                         "缺省 = --finger-joint 单指。多指为手型刚体联动：各指"
                         "（含主指）保持手型，由臂带主指尖走轨迹，副指随动"
                         "（不做位置约束，仅验证），仅支持 full/fixed 模式")
    ap.add_argument("--mode", choices=["full", "fixed", "tcp"], default="full",
                    help="full=臂+手指耦合全链(默认), fixed=手指定值+臂7DOF, tcp=原demo")
    ap.add_argument("--finger", type=float, default=0.6,
                    help="手指 proximal 目标 (rad)：full 的初值 / fixed 的固定值，"
                         "上限按手指（四指 1.41、拇指 1.03）自动裁剪")
    ap.add_argument("--offset", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                    metavar=("X", "Y", "Z"),
                    help="路径中心平移 (m, pelvis 系)，微调预览图形/轨迹位置")
    ap.add_argument("--waist", choices=["fixed", "free"], default="fixed",
                    help="fixed=腰固定(默认); free=腰3关节并入变量空间(full/fixed模式，"
                         "tcp 恒为 fixed)。--arm both 时腰只归属右臂，左臂在右臂"
                         "解出的腰值上冻结规划（腰是左右共享的物理关节）")
    return ap.parse_args()


ARGS = _parse_args()
MODE = ARGS.mode
FINGER_NAME = ARGS.finger_joint
FINGER_LIST = [s.strip() for s in ARGS.fingers.split(",")] \
    if ARGS.fingers else [FINGER_NAME]
if not FINGER_LIST or any(f not in FINGERS for f in FINGER_LIST) \
        or len(set(FINGER_LIST)) != len(FINGER_LIST):
    raise SystemExit(f"--fingers 无效: {ARGS.fingers}（应为逗号分隔的手指名，"
                     f"可选 {list(FINGERS)}，不可重复）")
if len(FINGER_LIST) > 1 and MODE == "tcp":
    raise SystemExit("--fingers 多指仅支持 full/fixed 模式（tcp 是回归基线）")
FINGER = float(np.clip(ARGS.finger, 0.0, FINGERS[FINGER_NAME]["upper"]))
SIDES = {"right": ("right",), "left": ("left",), "both": ("right", "left")}[ARGS.arm]
OFFSET = np.asarray(ARGS.offset, float)
SHAPES = ARGS.shapes or ["circle", "arc", "triangle", "line"]

# B1 腰部冗余：--waist free 时腰 3 关节并入变量空间（full/fixed 模式；tcp 恒 fixed）。
# 腰 3 关节是左右臂共享的同一组物理关节，--arm both 时只归属右臂，左臂在右臂解出
# 的腰值上冻结。镜像系数：绕 z(yaw)/x(roll) 的转角在 y 镜像下反号，绕 y(pitch) 不变。
WAIST_JOINTS = ["waist_33", "waist_32", "waist_31"]
WAIST_MIRROR = np.array([-1.0, -1.0, 1.0])
WAIST_FREE = ARGS.waist == "free" and MODE != "tcp"


def make_path(shape, u, v, center, size=SIZE, n=N_POINTS):
    """在 (u, v) 张成的竖直平面里生成路径点。"""
    if shape == "line":                                     # 直线：沿 u 方向
        t = np.linspace(-1.0, 1.0, n)[:, None]
        return center + t * size * u
    if shape == "triangle":                                 # 等边三角形（外接圆半径 size）
        corners = [center + size * (math.cos(a) * u + math.sin(a) * v)
                   for a in (0.0, 2 * math.pi / 3, 4 * math.pi / 3)]   # 第一个顶点在 u 方向
        pts = []
        for i in range(3):
            a, b = corners[i], corners[(i + 1) % 3]
            t = np.linspace(0.0, 1.0, n // 3, endpoint=False)[:, None]
            pts.append(a + t * (b - a))
        return np.vstack(pts)
    span = 2.0 * math.pi if shape == "circle" else math.pi   # 圆弧 = 半圆
    th = np.linspace(0.0, span, n, endpoint=False)
    return np.array([center + size * (math.cos(a) * u + math.sin(a) * v) for a in th])


def rot_err(Rt, Rc):
    """世界系旋转向误差 0.5*vee(Rt Rcᵀ)。"""
    M = Rt @ Rc.T
    return 0.5 * np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]])


# ---------------------------------------------------------------- 左臂镜像种子
def find_seed(robot, side):
    """返回该臂的种子关节向量（组关节顺序）。

    右臂直接用标定好的 SEED_ARM。左臂不手工标定：机器人左右对称，枚举 2^7 个
    符号模式 q_L = s ⊙ SEED_ARM（按组关节顺序逐位对应），用 FK 验证左 tcp 位姿
    ≈ M·(右 tcp 位姿)（M = diag(1,−1,1)），命中即种子。128 次 FK 成本可忽略，
    且不依赖对 URDF 轴向符号的人工推断。
    """
    if side == "right":
        return SEED_ARM.copy()
    jn_r, jn_l = robot.get_joint_names("right_arm"), robot.get_joint_names("left_arm")
    lim = robot.get_joint_limits("left_arm")
    robot.env.setState(jn_r, SEED_ARM)
    T = robot.env.getState().link_transforms[TCP_LINK["right"]]
    p_t = MIRROR @ np.asarray(T.translation, float)
    R_t = MIRROR @ np.asarray(T.rotation, float) @ MIRROR
    best = None                            # (dp+dr, dp, dr, q)
    for s in itertools.product((1.0, -1.0), repeat=len(jn_l)):
        q = np.asarray(s) * SEED_ARM
        if any(q[i] < lim[jn_l[i]]["lower"] or q[i] > lim[jn_l[i]]["upper"]
               for i in range(len(jn_l))):
            continue
        robot.env.setState(jn_l, q)
        T2 = robot.env.getState().link_transforms[TCP_LINK["left"]]
        dp = float(np.linalg.norm(np.asarray(T2.translation, float) - p_t))
        dr = float(np.linalg.norm(rot_err(R_t, np.asarray(T2.rotation, float))))
        if best is None or dp + dr < best[0]:
            best = (dp + dr, dp, dr, q)
    # URDF 左右不完全对称（实测残差 ~1mm / 0.003rad），阈值放宽到 5mm / 5mrad
    if best is not None and best[1] < 5e-3 and best[2] < 5e-3:
        return best[3]
    raise RuntimeError(f"左臂镜像种子求解失败：最优残差 dp={best[1]*1000:.1f}mm "
                       f"dr={best[2]*1000:.1f}mrad")


# ---------------------------------------------------------------- 耦合 IK
def make_finger_ik(robot, jn_state, lo, hi, assemble, tip_frame, full, to_vars=None):
    """手指耦合 DLS IK 工厂。

    变量 x：full 模式 = [arm7, prox]（8 个），返回完整状态向量；fixed 模式 =
    [arm7]（7 个），返回 q7。FK 时显式写入 distal = mult*prox（tesseract 不自动
    应用 URDF <mimic>，漏写 distal 就成了 distal 冻结的假机构，Descartes 内部
    校验必然对不上）。assemble(x) 把变量展开成与 jn_state 对齐的完整状态值
    （含固定的 metacarpal / 手指值）。作为 LMA 的后备——LMA 收敛域小，腕部奇异
    附近常失败，DLS + 限位投影能补上一部分。

    to_vars：把 IK 种子（可能是含腰前缀的全状态向量）投影到变量空间的函数；
    缺省直接取前 n 个（变量是状态前缀的情形，见 build_cfg 的腰部冻结分支）。
    """
    n = len(lo)

    def fk(x):
        robot.env.setState(jn_state, assemble(x))
        T = robot.env.getState().link_transforms[tip_frame]
        return np.array(T.translation, float), np.array(T.rotation, float)

    def ik(pose, seed):
        sv = np.asarray(seed, float) if to_vars is None else to_vars(seed)
        x = np.clip(sv[:n].copy(), lo, hi)
        p_t = np.array([pose.x, pose.y, pose.z])
        R_t = pose.rotation_matrix
        e_norm = None
        for _ in range(IK_ITERS):
            p, R = fk(x)
            e = np.concatenate([p_t - p, rot_err(R_t, R)])
            e_norm = float(np.linalg.norm(e))
            if e_norm < IK_TOL:
                return assemble(x) if full else x
            J = np.zeros((6, n))
            h = 1e-6
            for i in range(n):
                xp = x.copy()
                xp[i] += h
                pp, Rp = fk(xp)
                J[:3, i] = (pp - p) / h
                M = ((Rp - R) / h) @ R.T
                J[3:, i] = rot_err(Rp, R) / h
            dx = np.linalg.solve(J.T @ J + (IK_LAM ** 2) * np.eye(n) + 1e-10 * np.eye(n), J.T @ e)
            step = float(np.linalg.norm(dx))
            if step > IK_MAX_STEP:
                dx *= IK_MAX_STEP / step
            x = np.clip(x + dx, lo, hi)
        if e_norm >= IK_TOL_MAX:
            return None
        return assemble(x) if full else x

    return ik


# ---------------------------------------------------------------- tesseract 规划
def tip_pose(robot, cfg, q):
    """pelvis 系下 cfg['tip_frame'] 的位置与姿态。q 是规划空间向量。"""
    cfg["set_state"](q)
    T = robot.env.getState().link_transforms[cfg["tip_frame"]]
    return np.array(T.translation, float), np.array(T.rotation, float)


def fix_allowed_collisions(robot, cfgs):
    """修 ACM。

    SRDF 里只禁用了躯干/右臂链的相邻对，其余（腿、头、相机、左手、其余手指）
    全都保留，于是任何位形都会报几十个“碰撞”，规划必然失败。这里禁用 URDF 里
    所有父子链接对，再用随机采样找出并禁用那些网格本身就互相穿插的“结构性
    重叠”对。采样轮流经过各侧 cfg['sample_state']，手指 distal 始终按耦合写入。
    """
    from tesseract_robotics.tesseract_collision import (
        ContactRequest, ContactResultMap, ContactResultVector, ContactTestType_ALL)

    tree = ET.parse(URDF)
    for j in tree.getroot().findall("joint"):
        p, c = j.find("parent"), j.find("child")
        if p is not None and c is not None:
            robot.add_allowed_collision(p.get("link"), c.get("link"), "Adjacent")

    mgr = robot.env.getDiscreteContactManager()
    mgr.setActiveCollisionObjects(robot.env.getActiveLinkNames())
    mgr.setDefaultCollisionMargin(COLLISION_MARGIN)
    req = ContactRequest(ContactTestType_ALL)
    req.calculate_distance = True

    def pairs():
        for cfg in cfgs:
            cfg["sample_state"](rng)
        mgr.setCollisionObjectsTransform(robot.env.getState().link_transforms)
        res = ContactResultMap()
        mgr.contactTest(res, req)
        vec = ContactResultVector()
        res.flattenMoveResults(vec)
        return [tuple(sorted(vec[i].link_names)) for i in range(len(vec))]

    rng = np.random.default_rng(0)
    count = {}
    for _ in range(40):
        for pair in pairs():
            count[pair] = count.get(pair, 0) + 1
    for pair, n in count.items():
        if n >= 0.85 * 40:
            robot.add_allowed_collision(pair[0], pair[1], "Never")


def plan_path(robot, cfg, poses, seed, ik_fn, return_result=False):
    """Descartes 笛卡尔规划，返回关节轨迹 (N, len(cfg['joint_names']))。

    return_result=True 时返回 (Q, response)，response.results 可直接交给
    时间参数化（TOTG）做后处理。

    默认采样器用全零关节向量作为 IK 种子，而 KDLInvKinChainLMA 对本臂（限位高度
    不对称）从零种子不收敛，会报 "LadderGraphSolver failed to build graph"，
    所以这里自己写带热启动的采样器。

    三个 tesseract 绑定的坑（都已规避）：
      1. Robot.ik 默认 tip_link 是 getActiveLinkNames() 的末元素（实测
         right_index_touch_link），必须显式传 TCP，否则解出来的位姿完全不对。
      2. 必须直接继承 DescartesMoveProfileD —— nanobind 的 trampoline 注册在这个
         类上；继承 DescartesDefaultMoveProfileD 的 Python 子类不会把虚函数派发回
         Python（createWaypointSampler 一次都不会被调用）。
      3. Robot.ik/fk 的坐标系是 pelvis（组 base 被上溯到根链接），不是 SRDF 里写的
         waist_pitch_link。
    """
    from tesseract_robotics.planning.profiles import DESCARTES_DEFAULT_NAMESPACE
    from tesseract_robotics.tesseract_command_language import (
        CartesianWaypoint, CartesianWaypointPoly_wrap_CartesianWaypoint, CompositeInstruction,
        InstructionPoly_as_MoveInstructionPoly, MoveInstruction,
        MoveInstructionPoly_wrap_MoveInstruction, MoveInstructionType_LINEAR, ProfileDictionary,
        WaypointPoly_as_CartesianWaypointPoly, WaypointPoly_as_StateWaypointPoly)
    from tesseract_robotics.tesseract_motion_planners import PlannerRequest
    from tesseract_robotics.tesseract_motion_planners_descartes import (
        DescartesDefaultMoveProfileD, DescartesMotionPlannerD, DescartesMoveProfileD,
        DescartesStateD, DescartesStateSampleD, DescartesWaypointSamplerD,
        cast_DescartesMoveProfileD)
    from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

    class FixedSampler(DescartesWaypointSamplerD):
        def __init__(self, samples):
            super().__init__()
            self._samples = samples

        def sample(self):
            return self._samples

    class WarmStartProfile(DescartesMoveProfileD):
        def __init__(self):
            super().__init__()
            self._inner = DescartesDefaultMoveProfileD()
            self._last = np.asarray(seed, float).copy()
            self.ok = self.fail = 0

        def createWaypointSampler(self, mi, info, env):
            wp = mi.getWaypoint()
            if not wp.isCartesianWaypoint():
                return self._inner.createWaypointSampler(mi, info, env)
            pose = Pose(WaypointPoly_as_CartesianWaypointPoly(wp).getTransform())
            sol = ik_fn(pose, self._last)
            if sol is None:
                sol = ik_fn(pose, seed)
            if sol is None:
                self.fail += 1
                return FixedSampler([])
            self._last = np.asarray(sol, float)
            self.ok += 1
            return FixedSampler([DescartesStateSampleD(DescartesStateD(self._last), 0.0)])

        def createEdgeEvaluator(self, mi, info, env):
            return self._inner.createEdgeEvaluator(mi, info, env)

        def createStateEvaluator(self, mi, info, env):
            return self._inner.createStateEvaluator(mi, info, env)

    info = robot.get_manipulator_info(cfg["group"], tcp_frame=cfg["tcp"], working_frame=FRAME)
    program = CompositeInstruction()
    program.setManipulatorInfo(info)
    for pose in poses:
        mi = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(pose)),
                             MoveInstructionType_LINEAR, "WARMSTART")
        mi.setManipulatorInfo(info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

    # 航点已足够密，阈值设成略大于间距即可避免插值产生新的、可能无解的位姿
    spacing = 2.0 * math.pi * SIZE / N_POINTS
    interp = generateInterpolatedProgram(program, robot.env, 0.1, spacing * 1.05, 0.1, 1)

    profile = WarmStartProfile()
    profiles = ProfileDictionary()
    profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, "WARMSTART",
                        cast_DescartesMoveProfileD(profile))

    request = PlannerRequest()
    request.instructions = interp
    request.env = robot.env
    request.profiles = profiles
    response = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE).solve(request)
    print(f"  Descartes: {'成功' if response.successful else '失败: ' + response.message}"
          f"  (航点 IK {profile.ok}/{profile.ok + profile.fail})")
    if not response.successful:
        return None

    Q = []
    for instr in response.results:
        if instr.isMoveInstruction():
            wp = InstructionPoly_as_MoveInstructionPoly(instr).getWaypoint()
            if wp.isStateWaypoint():
                Q.append(np.asarray(WaypointPoly_as_StateWaypointPoly(wp).getPosition(), float))
    Q = np.asarray(Q)
    return (Q, response, (profile.ok, profile.ok + profile.fail)) if return_result else Q


# ---------------------------------------------------------------- 时间参数化 / 过渡
def extract_timed_trajectory(robot, results):
    """对规划结果做时间参数化（TOTG，需要 URDF velocity 限位），返回 (Q, ts)。

    ts 为各航点的 time_from_start (s)，轨迹首尾速度为零。失败返回 ts=None
    （调用方回退匀速播放）。探针已验证：72 点 circle 2.66s、峰值 2.6 rad/s。
    """
    from tesseract_robotics.tesseract_command_language import (
        InstructionPoly_as_MoveInstructionPoly, ProfileDictionary,
        WaypointPoly_as_StateWaypointPoly)
    from tesseract_robotics.tesseract_time_parameterization import (
        TimeOptimalTrajectoryGeneration)

    try:
        if not TimeOptimalTrajectoryGeneration().compute(results, robot.env, ProfileDictionary()):
            return None
    except Exception as e:  # noqa: BLE001
        print(f"  TOTG 失败({e})，回退匀速播放")
        return None

    Q, ts = [], []
    for instr in results:
        if instr.isMoveInstruction():
            wp = InstructionPoly_as_MoveInstructionPoly(instr).getWaypoint()
            if wp.isStateWaypoint():
                swp = WaypointPoly_as_StateWaypointPoly(wp)
                Q.append(np.asarray(swp.getPosition(), float))
                ts.append(float(swp.getTime()))
    if len(Q) < 2 or ts[-1] <= 0:
        return None
    return np.asarray(Q), np.asarray(ts)


def _scan_collision(robot, cfgs, qsides, mgr, req, acm):
    """设置各侧状态后做一次碰撞扫描，返回首个未被 ACM 允许的碰撞对或 None。"""
    from tesseract_robotics.tesseract_collision import (
        ContactResultMap, ContactResultVector)

    for side, cfg in cfgs.items():
        cfg["set_state"](qsides[side])
    mgr.setCollisionObjectsTransform(robot.env.getState().link_transforms)
    res = ContactResultMap()
    mgr.contactTest(res, req)
    vec = ContactResultVector()
    res.flattenMoveResults(vec)
    for i in range(len(vec)):
        l1, l2 = vec[i].link_names[0], vec[i].link_names[1]
        if acm.isCollisionAllowed(l1, l2):
            return (l1, l2)
    return None


def plan_transition(robot, cfgs, Q_from, Q_to, n=32, tries=8):
    """两段关节位形之间的过渡轨迹（A2 起始归位）。

    Q_from/Q_to 是 {side: 关节向量}。先试直线插值（smoothstep 起停）+ 逐点
    碰撞扫描；失败再试带随机中间路标点的分段插值。返回 {side: (n+1, dof)}
    帧序列，失败返回 None。按 cfg['joint_names'] 泛化维度，不依赖具体模式。
    """
    from tesseract_robotics.tesseract_collision import ContactRequest, ContactTestType_ALL

    mgr = robot.env.getDiscreteContactManager()
    mgr.setActiveCollisionObjects(robot.env.getActiveLinkNames())
    mgr.setDefaultCollisionMargin(COLLISION_MARGIN)
    req = ContactRequest(ContactTestType_ALL)
    acm = robot.env.getAllowedCollisionMatrix()
    lo = {s: c["limits"]["lo"] for s, c in cfgs.items()}
    hi = {s: c["limits"]["hi"] for s, c in cfgs.items()}

    def smooth(t):
        return t * t * (3.0 - 2.0 * t)

    def try_path(waypoints):
        """waypoints: [q dict, ...]，段间直线插值，逐点扫描。"""
        frames = {s: [] for s in cfgs}
        for w0, w1 in zip(waypoints[:-1], waypoints[1:]):
            for k in range(n + 1):
                t = smooth(k / n)
                qs = {s: (1.0 - t) * w0[s] + t * w1[s] for s in cfgs}
                if _scan_collision(robot, cfgs, qs, mgr, req, acm) is not None:
                    return None
                for s in cfgs:
                    frames[s].append(qs[s])
        return {s: np.asarray(fr) for s, fr in frames.items()}

    # 1) 直线
    frames = try_path([Q_from, Q_to])
    if frames is not None:
        return frames

    # 2) 随机中间路标点
    rng = np.random.default_rng(0)
    for _ in range(tries):
        qm = {s: rng.uniform(lo[s], hi[s]) for s in cfgs}
        frames = try_path([Q_from, qm, Q_to])
        if frames is not None:
            print("  过渡：直线不可行，经随机路标点绕行成功")
            return frames
    return None


# ---------------------------------------------------------------- 规划一种形状
def plan_shape(robot, cfg, shape, u, v, tag=""):
    """现场规划一种形状。

    返回 (目标路径点, 关节轨迹, 时间戳, 失败原因)：成功时 reason=None，
    ts 是 TOTG 时间戳（失败则 None=匀速回退）；失败时 points/Q/ts 为 None，
    reason 给出分类（IK 失败点数等），供 A4 失败反馈显示。
    """
    label = f"[{tag}] " if tag else ""
    points = make_path(shape, u, v, cfg["center"])
    if "tip_offset" in cfg:
        # fixed 模式：手指固定后指尖相对 tcp 的偏移 p_off 是常量。tcp 目标姿态
        # 直接用原 demo 验证可行的固定姿态（若沿用"指尖姿态∘偏移"，腕关节
        # 要额外弯出手指弯曲角，7-DOF 下经常不可达），位置按 p_tcp = p_tip - R*p_off 换算。
        R = cfg["plan_rotation"]
        p_off = cfg["tip_offset"]
        poses = [Pose.from_matrix_position(R, list(p - R @ p_off)) for p in points]
    else:
        poses = [Pose.from_matrix_position(cfg["rotation"], list(p)) for p in points]

    cfg["set_state"](cfg["seed_plan"])
    Q, resp, (ik_ok, ik_total) = plan_path(robot, cfg, poses, cfg["seed_plan"],
                                           cfg["ik_fn"], return_result=True)
    if Q is None or len(Q) < 2:
        reason = (f"Descartes 失败（IK {ik_ok}/{ik_total}）" if ik_total else "Descartes 失败")
        print(f"  {label}{shape} 规划失败: {reason}")
        return None, None, None, reason

    # 时间参数化（TOTG）：失败则 ts=None，回退匀速播放
    timed = extract_timed_trajectory(robot, resp.results)
    if timed is not None:
        Q, ts = timed
    else:
        ts = None

    # 精度评估：规划出的关节轨迹做 FK，看指尖是否落在目标路径上
    tip = np.array([tip_pose(robot, cfg, qq)[0] for qq in Q])
    n = min(len(Q), len(points))
    pos_err = np.linalg.norm(tip[:n] - points[:n], axis=1)
    plane_dev = (tip[:n] - cfg["center"]) @ PLANE_NORMAL
    margin = min(float(np.min(np.minimum(qq - cfg["limits"]["lo"], cfg["limits"]["hi"] - qq)))
                 for qq in Q)
    dur = f"  时长 {ts[-1]:.2f}s" if ts is not None else ""
    print(f"  {label}{shape:9s} {len(Q)} 点  路径误差 {np.max(pos_err)*1000:.4f} mm  "
          f"离面 {np.max(np.abs(plane_dev))*1000:.4f} mm  关节裕度 {margin:+.3f} rad{dur}")

    # B2 副指验证（不做约束）：副指随主手刚体随动，指尖偏差应与主指同量级
    if cfg.get("deltas"):
        sub_err = 0.0
        for tf, d in cfg["deltas"]:
            sub = []
            for qq in Q[:n]:
                cfg["set_state"](qq)
                sub.append(np.array(
                    robot.env.getState().link_transforms[tf].translation, float))
            sub_err = max(sub_err, float(
                np.linalg.norm(np.asarray(sub) - (points[:n] + d), axis=1).max() * 1000))
        print(f"  {label}副指随动误差 {sub_err:.4f} mm")
        if sub_err > 0.5:
            reason = f"副指随动误差超差（{sub_err:.3f} mm）"
            print(f"  {label}{shape} 规划失败: {reason}")
            return None, None, None, reason
    return points, Q, ts, None


# ------------------------------------------------- 多指顺序规划（B2，自研）
# ---------------------------------------------------------------- 双臂碰撞校验
def _link_side(name):
    if "left" in name:
        return "left"
    if "right" in name:
        return "right"
    return None                      # 躯干/头/腿等中立链接


def check_dual_collision(robot, cfgs, results):
    """沿两臂轨迹逐点检查左右臂链接之间是否碰撞。

    每条单臂轨迹本身是规划器验证过无碰的，所以合并状态下的碰撞只可能来自
    左右臂互碰（或与躯干——那单臂规划时也会撞，不会到这里）。返回 True 表示
    通过；发现左右互碰则打印并返回 False。
    """
    from tesseract_robotics.tesseract_collision import (
        ContactRequest, ContactResultMap, ContactResultVector, ContactTestType_ALL)

    mgr = robot.env.getDiscreteContactManager()
    mgr.setActiveCollisionObjects(robot.env.getActiveLinkNames())
    mgr.setDefaultCollisionMargin(COLLISION_MARGIN)
    req = ContactRequest(ContactTestType_ALL)

    (pts_r, Q_r), (pts_l, Q_l) = results["right"][:2], results["left"][:2]
    jn_r, jn_l = cfgs["right"]["joint_names"], cfgs["left"]["joint_names"]
    n = min(len(Q_r), len(Q_l))
    for k in range(n):
        robot.env.setState(jn_r, Q_r[k])
        robot.env.setState(jn_l, Q_l[k])
        mgr.setCollisionObjectsTransform(robot.env.getState().link_transforms)
        res = ContactResultMap()
        mgr.contactTest(res, req)
        vec = ContactResultVector()
        res.flattenMoveResults(vec)
        for i in range(len(vec)):
            sides = {_link_side(nm) for nm in vec[i].link_names}
            if "left" in sides and "right" in sides:
                print(f"  双臂碰撞校验: 第 {k}/{n} 点左右臂相交 "
                      f"({vec[i].link_names[0]} <-> {vec[i].link_names[1]})")
                return False
    return True


# ---------------------------------------------------------------- 模式配置
def build_cfg(robot, side, finger, mode, arm_seed):
    """按 (侧, 手指, 模式) 组装规划所需的组/TCP/种子/IK 等配置。

    tesseract 的 env.setState 不会自动应用 URDF <mimic>（distal 漏写就冻在 0），
    所以手指 distal 一律按耦合系数显式写入——与 hand_bridge 在指令侧展开耦合
    是同一个道理。左臂的基准量（中心/姿态）一律由右臂标定值经 M 镜像得出。

    变量空间（B1）：--waist free 且非 tcp 模式时腰 3 关节并入变量空间，规划组
    换成含腰全链组（根 pelvis），变量 = [腰3?] + 臂7 + [指1?]，IK 返回组全状态
    向量（distal 仍按耦合显式写入）。--arm both 时腰只归右臂（waist_owned），
    左臂的腰从 frozen_waist 单元格读取，规划右臂后经 set_frozen_waist 注入。
    """
    spec = FINGERS[finger]
    mult, prox_j = spec["mult"], prox_joint(side, finger)
    distal_j, tip = distal_joint(side, finger), tip_link(side, finger)
    mc_j = spec["mc_joint"].format(side=side) if "mc_joint" in spec else None

    # 基准姿态/中心：右臂用标定值，左臂镜像
    center0 = CENTER_TCP if side == "right" else CENTER_TCP_L
    rot0 = ROTATION_TCP if side == "right" else ROTATION_TCP_L

    arm_names = robot.get_joint_names(ARM_GROUP[side])
    lo7 = np.array([robot.get_joint_limits(ARM_GROUP[side])[n]["lower"] for n in arm_names])
    hi7 = np.array([robot.get_joint_limits(ARM_GROUP[side])[n]["upper"] for n in arm_names])

    waist_free = WAIST_FREE and mode != "tcp"
    waist_owned = waist_free and not (len(SIDES) == 2 and side == "left")
    frozen_waist = {"w": np.zeros(len(WAIST_JOINTS))}

    if waist_free:
        group = waist_group(side, finger)
        jn_state = robot.get_joint_names(group)
        expect = (set(WAIST_JOINTS) | set(arm_names) | {prox_j, distal_j}
                  | ({mc_j} if mc_j else set()))
        assert set(jn_state) == expect, f"{group} 关节异常: {jn_state}"
        lims = robot.get_joint_limits(group)
    else:
        jn_state = robot.get_joint_names(full_group(side, finger))
        expect = set(arm_names) | {prox_j, distal_j} | ({mc_j} if mc_j else set())
        assert set(jn_state) == expect, f"{full_group(side, finger)} 关节异常: {jn_state}"

    def state_values(q_arm, prox):
        vals = {**dict(zip(arm_names, q_arm)), prox_j: prox, distal_j: mult * prox}
        if mc_j:
            vals[mc_j] = 0.0                     # thumb_rot 固定
        return np.array([vals[n] for n in jn_state])

    def sample_full(rng):
        x8 = np.append(rng.uniform(lo7, hi7),
                       rng.uniform(0.0, spec["upper"]))
        robot.env.setState(jn_state, state_values(x8[:7], x8[7]))

    def sample_fixed(rng):
        robot.env.setState(jn_state, state_values(rng.uniform(lo7, hi7), FINGER))

    def make_waist_state(var_names):
        """构造 waist 模式的变量->组全状态展开器（distal 按耦合显式写入）。"""
        lo_v = np.array([lims[n]["lower"] for n in var_names])
        hi_v = np.array([lims[n]["upper"] for n in var_names])
        if waist_owned:
            # 组链顺序 = [腰3, 臂7, (mc,) prox, distal]，变量必须是状态前缀
            # （warm-start 种子按前 n 个切片）
            assert list(jn_state[:len(var_names)]) == list(var_names), \
                f"{group} 变量前缀异常: {jn_state}"

        def state_full(x):
            d = dict(zip(var_names, np.asarray(x, float)))
            vals = {a: d[a] for a in arm_names}
            if mode == "full":
                vals[prox_j] = d[prox_j]
                vals[distal_j] = mult * d[prox_j]
            else:
                vals[prox_j] = FINGER
                vals[distal_j] = mult * FINGER
            if mc_j:
                vals[mc_j] = 0.0
            if waist_owned:
                vals.update({w: d[w] for w in WAIST_JOINTS})
            else:
                vals.update(zip(WAIST_JOINTS, frozen_waist["w"]))
            return np.array([vals[n] for n in jn_state])

        # 冻结腰时变量前面垫着腰 3 个状态位，warm-start 种子要跳过
        tv = None if waist_owned else (
            lambda s: np.asarray(s, float)[len(WAIST_JOINTS):
                                           len(WAIST_JOINTS) + len(var_names)])

        def sample(rng):
            robot.env.setState(jn_state, state_full(rng.uniform(lo_v, hi_v)))

        return lo_v, hi_v, state_full, tv, sample

    if mode == "tcp":
        return {
            "mode": mode, "side": side, "group": ARM_GROUP[side],
            "tcp": TCP_LINK[side], "tip_frame": TCP_LINK[side],
            "joint_names": arm_names,
            "set_state": lambda q: robot.env.setState(arm_names, np.asarray(q, float)),
            "sample_state": lambda rng: robot.env.setState(arm_names, rng.uniform(lo7, hi7)),
            "seed_plan": arm_seed.copy(),
            "center": center0.copy(), "rotation": rot0.copy(),
            "ik_fn": lambda pose, s: robot.ik(ARM_GROUP[side], pose, seed=s,
                                              tip_link=TCP_LINK[side]),
        }

    if mode == "full":
        if waist_free:
            var_names = (WAIST_JOINTS if waist_owned else []) + arm_names + [prox_j]
            lo_v, hi_v, state_full, tv, sample_w = make_waist_state(var_names)
            seed_vars = np.array(([0.0] * 3 if waist_owned else [])
                                 + list(arm_seed) + [FINGER])
            seed_state = state_full(seed_vars)
            robot.env.setState(jn_state, seed_state)
            T = robot.env.getState().link_transforms[tip]
            cfg = {
                "mode": mode, "side": side, "group": group,
                "tcp": tip, "tip_frame": tip,
                "joint_names": jn_state,
                "set_state": lambda q: robot.env.setState(jn_state, np.asarray(q, float)),
                "sample_state": sample_w,
                "seed_plan": seed_state,
                "center": np.array(T.translation, float),
                "rotation": np.array(T.rotation, float),
                "ik_fn": make_finger_ik(robot, jn_state, lo_v, hi_v, state_full, tip,
                                        full=True, to_vars=tv),
            }
            if not waist_owned:              # both 模式左臂：右臂规划后注入冻结腰值
                def set_frozen(w, cell=frozen_waist):
                    cell["w"] = np.asarray(w, float)
                cfg["set_frozen_waist"] = set_frozen
            return cfg

        seed_state = state_values(arm_seed, FINGER)
        robot.env.setState(jn_state, seed_state)
        T = robot.env.getState().link_transforms[tip]
        return {
            "mode": mode, "side": side, "group": full_group(side, finger),
            "tcp": tip, "tip_frame": tip,
            "joint_names": jn_state,
            "set_state": lambda q: robot.env.setState(jn_state, np.asarray(q, float)),
            "sample_state": sample_full,
            "seed_plan": seed_state,
            "center": np.array(T.translation, float),
            "rotation": np.array(T.rotation, float),
            "ik_fn": make_finger_ik(
                robot, jn_state, np.append(lo7, 0.0), np.append(hi7, spec["upper"]),
                lambda x: state_values(x[:7], x[7]), tip, full=True),
        }

    # fixed 模式：手指固定，指尖相对 tcp 的偏移 p_off 是常量。指尖路径中心取
    # 基准中心 + 基准姿态 ∘ p_off，使换算出的 tcp 路径恰好落在 tcp 模式已验证
    # 可行的中心圆上（若围绕种子指尖位置取中心，tcp 路径会偏出腕部可达姿态区，
    # LMA/DLS 都会失败）。左臂用镜像基准，手部镜像精确时结果与右臂严格镜像。
    if waist_free:
        var_names = (WAIST_JOINTS if waist_owned else []) + arm_names
        lo_v, hi_v, state_full, tv, sample_w = make_waist_state(var_names)
        seed_state = state_full(np.array(([0.0] * 3 if waist_owned else []) + list(arm_seed)))
        robot.env.setState(jn_state, seed_state)
        Ttip = robot.env.getState().link_transforms[tip]
        Ttcp = robot.env.getState().link_transforms[TCP_LINK[side]]
        T_tcp = np.eye(4); T_tcp[:3, :3] = np.array(Ttcp.rotation); T_tcp[:3, 3] = np.array(Ttcp.translation)
        T_tip = np.eye(4); T_tip[:3, :3] = np.array(Ttip.rotation); T_tip[:3, 3] = np.array(Ttip.translation)
        T_off = np.linalg.inv(T_tcp) @ T_tip
        p_off = np.array(T_off[:3, 3], float)
        R_off = np.array(T_off[:3, :3], float)
        dls = make_finger_ik(robot, jn_state, lo_v, hi_v, state_full, tip,
                             full=True, to_vars=tv)

        def ik_fn(pose, s):
            # 带腰 DLS 为主（腰是冗余自由度，可改善裕度/可达性）；失败退回
            # 臂 7-DOF LMA（腰保持种子值）。robot.ik 的组基座是 waist_pitch_link，
            # 目标位姿换算依赖当前腰状态——DLS 求解过程改写了腰，fallback 前必须
            # 先把腰重置回种子值，否则 LMA 解的是被腰偏移污染过的目标
            sol = dls(pose, s)
            if sol is not None:
                return sol
            sv = (np.asarray(s, float) if tv is None else tv(s))[:len(var_names)]
            w0 = sv[:len(WAIST_JOINTS)] if waist_owned else frozen_waist["w"]
            robot.env.setState(WAIST_JOINTS, np.asarray(w0, float))
            sol7 = robot.ik(ARM_GROUP[side], pose, seed=sv[-7:], tip_link=TCP_LINK[side])
            if sol7 is None:
                return None
            x = sv.copy()
            x[-7:] = np.asarray(sol7, float)
            return state_full(x)

        cfg = {
            "mode": mode, "side": side, "group": group,
            # 腰组链里没有 tcp_link，Descartes 直接跟踪指尖：位姿用指尖目标
            # （固定姿态 rot0∘R_off、位置即路径点），无需 fixed 模式的 tcp 换算
            "tcp": tip, "tip_frame": tip,
            "joint_names": jn_state,
            "set_state": lambda q: robot.env.setState(jn_state, np.asarray(q, float)),
            "sample_state": sample_w,
            "seed_plan": seed_state,
            "center": center0 + rot0 @ p_off,
            "rotation": rot0 @ R_off,
            "ik_fn": ik_fn,
            "other_pos": {prox_j: FINGER, distal_j: mult * FINGER},
        }
        if not waist_owned:
            def set_frozen(w, cell=frozen_waist):
                cell["w"] = np.asarray(w, float)
            cfg["set_frozen_waist"] = set_frozen
        return cfg

    robot.env.setState(jn_state, state_values(arm_seed, FINGER))
    Ttip = robot.env.getState().link_transforms[tip]
    Ttcp = robot.env.getState().link_transforms[TCP_LINK[side]]
    T_tcp = np.eye(4); T_tcp[:3, :3] = np.array(Ttcp.rotation); T_tcp[:3, 3] = np.array(Ttcp.translation)
    T_tip = np.eye(4); T_tip[:3, :3] = np.array(Ttip.rotation); T_tip[:3, 3] = np.array(Ttip.translation)
    T_off = np.linalg.inv(T_tcp) @ T_tip                      # tcp 系下指尖的常量位姿
    p_off = np.array(T_off[:3, 3], float)
    R_off = np.array(T_off[:3, :3], float)
    dls = make_finger_ik(robot, jn_state, lo7, hi7,
                         lambda x: state_values(x[:7], FINGER), tip, full=False)

    def ik_fn(pose, s):
        sol = robot.ik(ARM_GROUP[side], pose, seed=s, tip_link=TCP_LINK[side])
        if sol is not None:
            return np.asarray(sol, float)
        return dls(pose, s)

    return {
        "mode": mode, "side": side, "group": ARM_GROUP[side],
        "tcp": TCP_LINK[side], "tip_frame": tip,
        "joint_names": arm_names,
        "set_state": lambda q: robot.env.setState(jn_state, state_values(q, FINGER)),
        "sample_state": sample_fixed,
        "seed_plan": arm_seed.copy(),
        "center": center0 + rot0 @ p_off,
        "rotation": rot0 @ R_off,               # 指尖的固定姿态（仅记录用）
        "plan_rotation": rot0,                  # fixed 模式 tcp 目标姿态
        "tip_offset": p_off,
        "ik_fn": ik_fn,
        # distal 也要显式发布：部分 robot_state_publisher 版本不应用 URDF <mimic>，
        # 只发 proximal 的话渲染出的手指是直的、渲染指尖会偏离预览轨迹
        "other_pos": {prox_j: FINGER, distal_j: mult * FINGER},
    }


def display_other_pos(side, fingers):
    """未参与规划的四指也按 FINGER 弯曲（纯显示，避免其余手指僵直的怪相）；
    拇指保持 0。tcp 模式不调用（保持原 demo 全伸直行为）。"""
    out = {}
    for f in ("index", "middle", "ring", "pinky"):
        if f in fingers:
            continue
        out[prox_joint(side, f)] = FINGER
        out[distal_joint(side, f)] = FINGERS[f]["mult"] * FINGER
    return out


# ---------------------------------------------------------------- 多指配置
def build_cfg_multi(robot, side, fingers, mode, arm_seed):
    """多指（--fingers ≥2）配置（B2）：手型刚体联动。

    灵巧手自由度有限，副指尖与主指尖做共同位置约束（变量 9 个对误差 12 维）
    大概率无解——多指语义定为：参与手指（含主指）prox 全部固定在 --finger 值、
    distal 按耦合显式写入，手是刚体，由臂 7-DOF（+腰）带着走主指尖路径；
    所有模式指尖目标姿态固定，副指尖相对主指尖的世界系偏移因此是常量，
    "手型保持、平行随动"严格成立。副指位置只做验证（plan_shape 内 <0.5mm）
    不做约束。规划走与单指 fixed 相同的已验证管线，多指只增加副指的
    验证与历史轨迹显示。full 模式的主指 prox 变量会破坏刚体假设
    （随动误差 ~1-2mm），多指时统一按 fixed 手指语义规划。
    """
    cfg = build_cfg(robot, side, fingers[0], "fixed", arm_seed)

    sec_names, sec_pos = [], []
    for f in fingers[1:]:
        pj = float(np.clip(FINGER, 0.0, FINGERS[f]["upper"]))  # 拇指上限 1.03
        sec_names += [prox_joint(side, f), distal_joint(side, f)]
        sec_pos += [pj, FINGERS[f]["mult"] * pj]
        mc = FINGERS[f].get("mc_joint")
        if mc:
            sec_names.append(mc.format(side=side))
            sec_pos.append(0.0)

    # 副指固定手型并入 other_pos：RvizSim 经 /joint_states 发布渲染，env 状态
    # 在 main 里统一写入后不再变动——规划与碰撞扫描全程可见真实手型
    cfg.setdefault("other_pos", {})
    cfg["other_pos"].update(dict(zip(sec_names, sec_pos)))

    # 种子手型偏移（验证与副指尖历史显示用）：手是刚体，此偏移全程恒定
    robot.env.setState(sec_names, np.asarray(sec_pos, float))
    cfg["set_state"](cfg["seed_plan"])
    st = robot.env.getState().link_transforms
    p_main = np.array(st[cfg["tip_frame"]].translation, float)
    deltas, tip_frames = [], {}
    for f in fingers[1:]:
        tf = tip_link(side, f)
        deltas.append((tf, np.array(st[tf].translation, float) - p_main))
        tip_frames[f] = tf
    cfg.update({
        "deltas": deltas,
        "tip_frames": tip_frames,
        "extra_fingers": list(fingers[1:]),
    })
    return cfg


# ---------------------------------------------------------------- RViz 显示
class RvizSim:
    """把规划好的关节轨迹按分段序列回放，并画出指尖历史轨迹。

    回放序列 self.seq = [(kind, {side: 帧})]：
        ("trans", ...)  形状间过渡（A2，从上一段末帧平滑走到新形状首帧）；
        ("shape", ...)  形状本身，按 TOTG 时间戳插值（A1，保留真实速度曲线；
                        ts 缺失时回退关节空间弧长匀速），连续播 SHAPE_REPEAT 遍。
    规划失败（A4）：在路径中心下方发红色文字标记，继续尝试下一种形状。
    支持多侧：planning 关节取各侧并集；历史/理想路径/标记按侧分话题发布
    （--arm both 时左臂加 _left 后缀，单臂模式一律用无后缀话题）。
    """

    def __init__(self, robot, cfgs, u, v, plan_fn):
        self.robot = robot
        self.cfgs = cfgs
        self.sides = list(cfgs)
        self.u, self.v = u, v
        self.plan_fn = plan_fn
        self.shapes = SHAPES
        self.shape_idx = -1

        plan_joints = []
        for side in self.sides:
            plan_joints += cfgs[side]["joint_names"]
        self.plan_joints = plan_joints
        self.other_joints = [
            j.get("name") for j in ET.parse(URDF).getroot().findall("joint")
            if j.get("type") != "fixed" and j.get("name") not in plan_joints]
        self.other_pos = {}
        for side in self.sides:
            self.other_pos.update(cfgs[side].get("other_pos", {}))

        self.node = Node("tienkung_dex_shape_demo")
        self.pub_joints = self.node.create_publisher(JointState, "/joint_states", 10)
        self.pub = {}
        for side in self.sides:
            sfx = "" if len(self.sides) == 1 or side == "right" else "_left"
            self.pub[side] = (
                self.node.create_publisher(Path, f"/tcp_history{sfx}", 10),
                self.node.create_publisher(Path, f"/path_ideal{sfx}", 1),
                self.node.create_publisher(MarkerArray, f"/path_markers{sfx}", 1))

        self.history = {side: [] for side in self.sides}
        self.pub_x = {}                          # 副指尖历史话题（B2 多指）
        self.history_x = {}
        for side in self.sides:
            sfx = "" if len(self.sides) == 1 or side == "right" else "_left"
            self.pub_x[side] = {
                f: self.node.create_publisher(Path, f"/tcp_history_{f}{sfx}", 10)
                for f in self.cfgs[side].get("extra_fingers", [])}
            self.history_x[side] = {
                f: [] for f in self.cfgs[side].get("extra_fingers", [])}
        self.warn = {side: None for side in self.sides}
        self.points = {}
        self.shape = None
        self.seq = []
        self.seg_i = 0
        self.i = 0
        self.tick_n = 0
        self._next_shape()                      # 启动先规划第一种
        self.node.create_timer(1.0 / RATE, self._tick)

    # --- 换形状（现场重新规划） ---
    def _next_shape(self):
        for _ in range(len(self.shapes)):
            self.shape_idx = (self.shape_idx + 1) % len(self.shapes)
            shape = self.shapes[self.shape_idx]
            self.node.get_logger().info(f"规划 {shape} ...")
            out, warns = self.plan_fn(shape)
            if out is None:
                for side in self.sides:         # A4：失败原因显示在路径中心旁
                    if warns.get(side):
                        self.warn[side] = f"{shape}: {warns[side]}"
                self.node.get_logger().error(f"{shape} 规划失败: {warns}")
                self._publish_static()
                continue
            self.warn = {side: None for side in self.sides}
            self.shape = shape
            self.points = {side: out[side][0] for side in self.sides}
            seq = []
            if self.seq:                        # 过渡：上一段末帧 -> 新形状首帧
                last = {s: self.seq[-1][1][s][-1] for s in self.sides}
                first = {s: out[s][1][0] for s in self.sides}
                trans = plan_transition(self.robot, self.cfgs, last, first)
                if trans is not None:
                    seq.append(("trans", {s: self._resample(
                        trans[s], TRANSITION_SECONDS * RATE) for s in self.sides}))
                else:
                    self.node.get_logger().warning("过渡轨迹规划失败，直接跳变")
            seg = ("shape", {s: self._timed_frames(s, out[s][1], out[s][2])
                             for s in self.sides})
            seq += [seg] * SHAPE_REPEAT
            self.seq, self.seg_i, self.i = seq, 0, 0
            self._publish_static()
            self.node.get_logger().info(f"开始回放 {shape}")
            return
        raise RuntimeError("所有形状都规划失败")

    # --- 帧序列构造 ---
    def _resample(self, frames, n_target, s=None):
        """把 (m, dof) 帧序列按参数 s（缺省均匀）重采样到 n_target 帧播放网格。"""
        n = max(2, int(n_target))
        if s is None:
            s = np.linspace(0.0, 1.0, len(frames))
        s = np.asarray(s, float)
        s = s / s[-1]
        frac = np.linspace(0.0, 1.0, n, endpoint=False)
        return np.column_stack(
            [np.interp(frac, s, frames[:, j]) for j in range(frames.shape[1])])

    def _timed_frames(self, side, Q, ts):
        """形状轨迹 -> 播放帧：TOTG 时间戳优先（真实速度曲线），无 ts 回退弧长匀速。"""
        if ts is not None:
            s = ts
        else:
            seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)
            s = np.concatenate([[0.0], np.cumsum(seg)])
        return self._resample(Q, DURATION * RATE, s)

    def _publish_static(self):
        for side in self.sides:
            _, pub_ideal, pub_markers = self.pub[side]
            if side in self.points:
                pub_ideal.publish(self._path(side, self.points[side]))
            pub_markers.publish(self._markers(side))

    # --- 消息构造 ---
    def _pose(self, x, y, z):
        return PoseMsg(position=Point(x=float(x), y=float(y), z=float(z)),
                       orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def _path(self, side, pts):
        now = self._stamp()
        path = Path()
        path.header = Header(frame_id=FRAME, stamp=now)
        for p in pts:
            ps = PoseStamped()
            ps.header = Header(frame_id=FRAME, stamp=now)
            ps.pose = self._pose(*p)
            path.poses.append(ps)
        return path

    def _markers(self, side):
        now = self._stamp()
        cfg = self.cfgs[side]
        idx = 10 * self.sides.index(side)

        def marker(mid, mtype):
            m = Marker()
            m.header = Header(frame_id=FRAME, stamp=now)
            m.ns, m.id, m.type, m.action = f"path_{side}", idx + mid, mtype, Marker.ADD
            m.lifetime.sec = m.lifetime.nanosec = 0
            return m

        arr = MarkerArray()
        m = marker(0, Marker.SPHERE)                 # 路径中心
        m.pose, m.scale.x = self._pose(*cfg["center"]), 0.02
        m.scale.y = m.scale.z = 0.02
        m.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        arr.markers.append(m)
        m = marker(1, Marker.ARROW)                  # 平面法向
        m.pose, m.scale.x, m.scale.y, m.scale.z = self._pose(*cfg["center"]), 0.12, 0.012, 0.02
        m.color = ColorRGBA(r=0.9, g=0.6, b=0.1, a=1.0)
        arr.markers.append(m)
        m = marker(2, Marker.TEXT_VIEW_FACING)       # 文字
        m.pose, m.scale.z = self._pose(*(cfg["center"] + 0.06 * self.v)), 0.03
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = f"{self.shape}  size = {SIZE*1000:.0f} mm  [{cfg['mode']}:{side}]"
        arr.markers.append(m)
        m = marker(3, Marker.TEXT_VIEW_FACING)       # 规划失败警告（无警告时空文本不可见）
        m.pose, m.scale.z = self._pose(*(cfg["center"] - 0.08 * self.v)), 0.035
        m.color = ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0)
        m.text = self.warn.get(side) or ""
        arr.markers.append(m)
        return arr

    # --- 主循环 ---
    def _tick(self):
        if not self.seq:
            return
        kind, frames = self.seq[self.seg_i]
        if kind == "shape" and self.i == 0:      # 形状段开头清空历史（过渡不画）
            for side in self.sides:
                self.history[side] = []
                for f in self.history_x.get(side, {}):
                    self.history_x[side][f] = []

        js = JointState()
        js.header = Header(stamp=self._stamp())
        js.name = self.plan_joints + self.other_joints
        pos = []
        for side in self.sides:                  # 规划关节：按侧顺序拼接
            pos += [float(x) for x in frames[side][self.i]]
        pos += [self.other_pos.get(n, 0.0) for n in self.other_joints]
        js.position = pos
        js.velocity = js.effort = [0.0] * len(js.name)
        self.pub_joints.publish(js)

        if kind == "shape":
            for side in self.sides:
                cfg = self.cfgs[side]
                cfg["set_state"](frames[side][self.i])
                st = self.robot.env.getState().link_transforms
                self.history[side].append(
                    np.array(st[cfg["tip_frame"]].translation, float))
                pub_hist, _, _ = self.pub[side]
                pub_hist.publish(self._path(side, self.history[side]))
                for f, tf in cfg.get("tip_frames", {}).items():
                    self.history_x[side][f].append(
                        np.array(st[tf].translation, float))
                    self.pub_x[side][f].publish(
                        self._path(side, self.history_x[side][f]))

        # 静态内容每秒重发一次，晚加入的 RViz 订阅者才能收到（QoS 是 volatile）
        self.tick_n += 1
        if self.tick_n % int(RATE) == 0:
            self._publish_static()

        self.i += 1
        if self.i >= len(frames[self.sides[0]]):
            self.seg_i += 1
            self.i = 0
            if self.seg_i >= len(self.seq):      # 序列播完，现场规划下一种
                self._next_shape()


def main():
    print("=" * 60)
    print(f"天工 Dex 双臂双手形状 demo —— 臂 {ARGS.arm}  指 {'+'.join(FINGER_LIST)}  "
          f"模式 {MODE}  "
          f"腰 {ARGS.waist}  {SHAPES}  "
          f"(size = {SIZE*1000:.0f}mm, finger = {FINGER:.2f} rad)")
    print("=" * 60)

    print("\n加载机器人 ...")
    robot = Robot.from_files(URDF, SRDF)

    cfgs = {}
    for side in SIDES:
        seed = find_seed(robot, side)
        if len(FINGER_LIST) > 1:
            cfg = build_cfg_multi(robot, side, FINGER_LIST, MODE, seed)
        else:
            cfg = build_cfg(robot, side, FINGER_NAME, MODE, seed)
        if "limits" not in cfg:               # 多指 cfg 已自带按名取的限位
            limits = robot.get_joint_limits(cfg["group"])
            cfg["limits"] = {
                "lo": np.array([limits[j]["lower"] for j in cfg["joint_names"]]),
                "hi": np.array([limits[j]["upper"] for j in cfg["joint_names"]]),
            }
        if np.any(OFFSET):                    # 手动微调预览图形/轨迹位置
            cfg["center"] = cfg["center"] + OFFSET
            print(f"  [{side}] 路径中心平移 {np.round(OFFSET, 4)} m -> "
                  f"{np.round(cfg['center'], 4)}")
        if MODE != "tcp":                     # 显示细节：其余四指跟着弯
            cfg["other_pos"] = {**cfg.get("other_pos", {}),
                                **display_other_pos(side, FINGER_LIST)}
        cfgs[side] = cfg
        print(f"  [{side}] 组 {cfg['group']}  TCP {cfg['tip_frame']}  "
              f"关节 {len(cfg['joint_names'])}")

    # 把显示/固定的手部姿态写入环境：之后的 ACM 采样与规划碰撞检查都能看到
    # 真实的手部状态（其余手指的值此后不再被任何 setState 改动，保持不变）
    for side in SIDES:
        op = cfgs[side].get("other_pos", {})
        if op:
            robot.env.setState(list(op), np.array([op[n] for n in op], float))

    robot.set_collision_margin(COLLISION_MARGIN)
    fix_allowed_collisions(robot, list(cfgs.values()))

    u = START_DIR / np.linalg.norm(START_DIR)         # 起始角度方向 θ=0 → +Y
    v = np.cross(PLANE_NORMAL, u)                     # 平面内与 u 垂直的竖直方向 (+Z)

    # A3 可达性预检：四个形状采样点（±u/±v）先做单点 IK，偏移过大立即拒绝启动，
    # 而不是等 Descartes 规划中途失败
    for side, cfg in cfgs.items():
        if "tip_offset" in cfg:              # fixed 模式：实际规划的是 tcp 目标位姿
            R, p_off = cfg["plan_rotation"], cfg["tip_offset"]
            pose = lambda p: Pose.from_matrix_position(R, list(p - R @ p_off))
        else:
            R = cfg["rotation"]
            pose = lambda p: Pose.from_matrix_position(R, list(p))
        bad = [d for d in (u, v, -u, -v)
               if cfg["ik_fn"](pose(cfg["center"] + SIZE * d), cfg["seed_plan"]) is None]
        if bad:
            raise SystemExit(
                f"[{side}] 可达性预检失败：中心 {np.round(cfg['center'], 3)} 沿 "
                f"{np.round(bad[0], 2)} 方向的形状采样点不可达，"
                f"请用 --offset 平移中心或减小图形尺寸")
    print("  可达性预检通过（形状 ±u/±v 四点）")

    def plan_fn(shape):
        out, warns = {}, {}
        order = list(SIDES)
        if len(SIDES) == 2 and WAIST_FREE:
            order = ["right", "left"]       # 腰归右臂：先右后左，左臂冻结右臂腰值
        for side in order:
            tag = side if len(SIDES) > 1 else ""
            points, Q, ts, reason = plan_shape(robot, cfgs[side], shape, u, v, tag=tag)
            if points is None:
                warns[side] = reason
                if len(SIDES) == 2 and WAIST_FREE and side == "right":
                    warns["left"] = "依赖右臂（腰冻结）"
                break
            out[side] = (points, Q, ts)
            if (side == "right" and len(SIDES) == 2
                    and "set_frozen_waist" in cfgs["left"]):
                # 左臂腰 = 右臂首航点腰值的镜像（yaw/roll 反号，pitch 不变）
                cfgs["left"]["set_frozen_waist"](WAIST_MIRROR * Q[0][:len(WAIST_JOINTS)])
        if len(out) < len(SIDES):
            return None, warns
        if len(SIDES) == 2 and not check_dual_collision(robot, cfgs, out):
            return None, {side: "双臂碰撞" for side in SIDES}
        return out, warns

    print("启动 RViz 仿真（每种形状在切换时现场规划）...")
    rclpy.init()
    sim = RvizSim(robot, cfgs, u, v, plan_fn)
    try:
        rclpy.spin(sim.node)
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001
        # Ctrl-C 时 rclpy 信号处理器已把 context 关掉，随后任何一次发布/等待都会
        # 抛 RCLError（"publisher's context is invalid" 之类）。此时 rclpy.ok()
        # 已为 False，属正常退出；否则才是真错误，照常抛出。
        if rclpy.ok():
            raise
    finally:
        sim.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
