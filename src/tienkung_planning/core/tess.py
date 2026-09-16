"""L1 防腐层：tesseract nanobind 绑定细节的唯一居所（封装方案 §5，实施计划步 1）。

本模块是全库唯一允许 `import tesseract_robotics` 的地方（CI 红线：tesseract
只出现在 tess.py）。上层只看到 numpy / 本模块的纯函数与再导出的对象模型。

再导出对象模型（浅导出；深度包装随步 2+3 的 ModeConfig 收紧）：
    Robot   —— 描述加载、fk/ik、关节限位、manipulator info、ACM 写入
    Pose    —— 位姿构造（from_matrix_position）

适配函数（绑定陷阱的规避都收拢在这里）：
    contact_request() / contact_manager() / contact_test() / acm()   碰撞
    descartes_plan()   笛卡尔规划全流程（program 构造→加密→求解→状态提取）
    totg()             TOTG 时间参数化（回退判定：返回值 + 异常，双判定）

线程与状态契约（§5/N4）：非线程安全（env 是有状态单例，一个实例归一个线程）；
快照/恢复只发生在上层公开 API 边界，本模块内层实现不做快照。
ACM 修复仅在模型构建期执行一次（N3），结果即快照基线。
"""
import numpy as np

# ---------------------------------------------------------------- 对象模型
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning.transforms import Pose

__all__ = ["Robot", "Pose", "contact_request", "contact_manager", "contact_test",
           "acm", "descartes_plan", "totg", "totg_traj"]


# ---------------------------------------------------------------- 碰撞
def contact_request():
    """全接触对请求（ContactTestType_ALL）。"""
    from tesseract_robotics.tesseract_collision import (ContactRequest,
                                                        ContactTestType_ALL)
    return ContactRequest(ContactTestType_ALL)


def contact_manager(env, margin=None):
    """离散接触管理器：激活全部链接，可选设置碰撞裕度。"""
    mgr = env.getDiscreteContactManager()
    mgr.setActiveCollisionObjects(env.getActiveLinkNames())
    if margin is not None:
        mgr.setDefaultCollisionMargin(margin)
    return mgr


def contact_test(mgr, req):
    """执行一次接触检测，返回 [(link1, link2), ...]（保持检测顺序）。"""
    from tesseract_robotics.tesseract_collision import (ContactResultMap,
                                                        ContactResultVector)
    res = ContactResultMap()
    mgr.contactTest(res, req)
    vec = ContactResultVector()
    res.flattenMoveResults(vec)
    return [tuple(vec[i].link_names) for i in range(len(vec))]


def acm(env):
    """当前允许碰撞矩阵（只读视图；修复仅在构建期执行一次，见 N3）。"""
    return env.getAllowedCollisionMatrix()


# ---------------------------------------------------------------- Descartes
def descartes_plan(env, info, poses, ik_fn, seed, spacing):
    """笛卡尔规划全流程：program 构造 → 航点加密 → WarmStart 求解 → 状态提取。

    返回 (Q, response, (ok, total))；求解失败返回 None。response.results
    是不透明对象，仅供 totg() 消费——上层不得解包。

    三个 tesseract 绑定的坑（都已规避在本函数内）：
      1. Robot.ik 默认 tip_link 是 getActiveLinkNames() 的末元素（实测
         right_index_touch_link），必须显式传 TCP，否则解出来的位姿完全不对。
      2. 必须直接继承 DescartesMoveProfileD —— nanobind 的 trampoline 注册在这个
         类上；继承 DescartesDefaultMoveProfileD 的 Python 子类不会把虚函数派发回
         Python（createWaypointSampler 一次都不会被调用）。
      3. Robot.ik/fk 的坐标系是 pelvis（组 base 被上溯到根链接），不是 SRDF 里写的
         waist_pitch_link。

    默认采样器用全零关节向量作为 IK 种子，而 KDLInvKinChainLMA 对限位高度
    不对称的臂从零种子不收敛（"LadderGraphSolver failed to build graph"），
    所以这里用带热启动的采样器：逐航点用上一个解做种子。
    """
    from tesseract_robotics.planning.profiles import DESCARTES_DEFAULT_NAMESPACE
    from tesseract_robotics.tesseract_command_language import (
        CartesianWaypoint, CartesianWaypointPoly_wrap_CartesianWaypoint,
        CompositeInstruction, MoveInstruction,
        MoveInstructionPoly_wrap_MoveInstruction, MoveInstructionType_LINEAR,
        ProfileDictionary, WaypointPoly_as_CartesianWaypointPoly)
    from tesseract_robotics.tesseract_motion_planners import PlannerRequest
    from tesseract_robotics.tesseract_motion_planners_descartes import (
        DescartesDefaultMoveProfileD, DescartesMotionPlannerD, DescartesMoveProfileD,
        DescartesStateD, DescartesStateSampleD, DescartesWaypointSamplerD,
        cast_DescartesMoveProfileD)
    from tesseract_robotics.tesseract_motion_planners_simple import (
        generateInterpolatedProgram)

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

        def createWaypointSampler(self, mi, minfo, menv):
            wp = mi.getWaypoint()
            if not wp.isCartesianWaypoint():
                return self._inner.createWaypointSampler(mi, minfo, menv)
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

        def createEdgeEvaluator(self, mi, minfo, menv):
            return self._inner.createEdgeEvaluator(mi, minfo, menv)

        def createStateEvaluator(self, mi, minfo, menv):
            return self._inner.createStateEvaluator(mi, minfo, menv)

    program = CompositeInstruction()
    program.setManipulatorInfo(info)
    for pose in poses:
        mi = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(pose)),
            MoveInstructionType_LINEAR, "WARMSTART")
        mi.setManipulatorInfo(info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

    # 航点已足够密，阈值设成略大于间距即可避免插值产生新的、可能无解的位姿
    interp = generateInterpolatedProgram(program, env, 0.1, spacing * 1.05, 0.1, 1)

    profile = WarmStartProfile()
    profiles = ProfileDictionary()
    profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, "WARMSTART",
                        cast_DescartesMoveProfileD(profile))

    request = PlannerRequest()
    request.instructions = interp
    request.env = env
    request.profiles = profiles
    response = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE).solve(request)
    if not response.successful:
        return None
    Q = _extract_state_positions(response.results)
    return (Q, response, (profile.ok, profile.ok + profile.fail))


def _extract_state_positions(results):
    """从规划结果指令流提取各状态航点的关节位置 (N, dof)。"""
    from tesseract_robotics.tesseract_command_language import (
        InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly)
    Q = []
    for instr in results:
        if instr.isMoveInstruction():
            wp = InstructionPoly_as_MoveInstructionPoly(instr).getWaypoint()
            if wp.isStateWaypoint():
                Q.append(np.asarray(
                    WaypointPoly_as_StateWaypointPoly(wp).getPosition(), float))
    return np.asarray(Q)


# ---------------------------------------------------------------- 时间参数化
def totg(results, env):
    """TOTG 时间参数化（需要 URDF velocity 限位），返回 (Q, ts) 或 None。

    回退判定与现实现一致：compute 返回假值、抛异常、点数 <2 或末时间戳
    非正，均返回 None（由上层回退匀速）。ts 为各航点 time_from_start (s)，
    轨迹首尾速度为零。
    """
    from tesseract_robotics.tesseract_command_language import (
        InstructionPoly_as_MoveInstructionPoly, ProfileDictionary,
        WaypointPoly_as_StateWaypointPoly)
    from tesseract_robotics.tesseract_time_parameterization import (
        TimeOptimalTrajectoryGeneration)

    try:
        if not TimeOptimalTrajectoryGeneration().compute(
                results, env, ProfileDictionary()):
            return None
    except Exception:  # noqa: BLE001 —— 绑定层异常统一归 None，上层回退
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


def totg_traj(Q, joint_names, env, info, dt=0.1):
    """对裸关节轨迹 (N, dof) 做 TOTG（time_param CLI 用）。

    把 Q 包成匀速 StateWaypoint 指令流（间隔 dt）后走与 totg() 相同的
    compute/提取路径；info 为 robot.get_manipulator_info(...) 产物
    （TOTG 需要据此找关节组）。返回 (Q', ts) 或 None（回退判定同 totg）。
    """
    from tesseract_robotics.tesseract_command_language import (
        CompositeInstruction, MoveInstruction,
        MoveInstructionPoly_wrap_MoveInstruction, MoveInstructionType_LINEAR,
        StateWaypoint, StateWaypointPoly_wrap_StateWaypoint)

    Q = np.asarray(Q, float)
    program = CompositeInstruction()
    program.setManipulatorInfo(info)
    for i, q in enumerate(Q):
        swp = StateWaypoint(list(joint_names), np.ascontiguousarray(q))
        swp.setTime(i * dt)
        mi = MoveInstruction(StateWaypointPoly_wrap_StateWaypoint(swp),
                             MoveInstructionType_LINEAR, "TIMED")
        mi.setManipulatorInfo(info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))
    return totg(program, env)
