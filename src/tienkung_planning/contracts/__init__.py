"""L0 契约层再导出（封装方案 §4.1/§3）。

真机执行侧（P-B）只 import 本包即可读写产物——import 零第三方依赖
（numpy 延迟到调用期，N9 裸 venv 红线）。
"""
from tienkung_planning.contracts.artifact import (
    ContractVersionError,
    GroupPlan,
    PathErrors,
    PlanArtifact,
    PlanMeta,
    SCHEMA_MAJOR,
    SCHEMA_MINOR,
    SecondaryTrack,
    SUPPORTED_MAJOR,
    TransitionInfo,
    artifact_hash,
    load,
    profile_hash,
    save,
)
from tienkung_planning.contracts.exceptions import (
    PlannerError,
    TienKungPlanningError,
    UnavailableError,
)

__all__ = [
    "PlanArtifact", "GroupPlan", "PlanMeta", "PathErrors", "SecondaryTrack",
    "TransitionInfo", "ContractVersionError", "artifact_hash", "profile_hash",
    "save", "load", "SCHEMA_MAJOR", "SCHEMA_MINOR", "SUPPORTED_MAJOR",
    "TienKungPlanningError", "UnavailableError", "PlannerError",
]
