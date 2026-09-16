"""契约层异常（封装方案 §4.3）：L0，纯标准库，可裸 venv import（N9）。"""


class TienKungPlanningError(Exception):
    """库异常层级基类（§4.3）。"""


class UnavailableError(TienKungPlanningError):
    """能力前置条件缺失（如 URDF 无速度限位 → TOTG 不可用，§4.3/§7）。"""


class PlannerError(TienKungPlanningError):
    """规划失败（可达性预检不过、IK 失败、双臂碰撞等）。"""
