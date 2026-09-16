"""L3 门面层再导出（封装方案 §3/§7）。

import 本包即要求 tesseract（经 core.tess）——这正是 N9 红线只约束顶层
`__init__` 的原因：真机执行侧 `import tienkung_planning.contracts` 不经此包。
"""
from tienkung_planning.facade.collision import CollisionApi
from tienkung_planning.facade.planner import (CheckReport, PlannerOptions,
                                              TienKungPlanner)
from tienkung_planning.facade.time_param import TimeParamApi, TimedTrajectory

__all__ = ["TienKungPlanner", "PlannerOptions", "CheckReport",
           "CollisionApi", "TimeParamApi", "TimedTrajectory"]
