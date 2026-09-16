"""L0 轨迹产物契约（封装方案 §4.1，实施计划步 5）。

序列化格式（B5 决策）：`.traj` = ZIP 包，内含
    meta.json    —— 全部非数组字段（含 groups 的结构与 secondary 记录）
    arrays.npz   —— 全部 ndarray（positions/timestamps/ideal_path/delta/腰值…）
hash 算法与排除字段见 artifact_hash()。

N9 红线：本包 import 零第三方依赖——numpy 延迟到调用期导入（裸 venv 可
`import tienkung_planning.contracts`，真机执行侧只装本包即可读产物）。
"""
from __future__ import annotations

import base64
import hashlib
import io
import json
import os
import zipfile
from dataclasses import asdict, dataclass, field
from typing import Literal, Optional

SCHEMA_MAJOR = 1          # major：load 拒绝判定的唯一依据（§4.5）
SCHEMA_MINOR = 0

#: 本库 load 支持的最高 major（= SCHEMA_MAJOR）；产物更高即抛错
SUPPORTED_MAJOR = SCHEMA_MAJOR


class ContractVersionError(Exception):
    """产物 schema_version 高于库支持（拒绝过新，而非静默错读，§4.5）。"""


# ---------------------------------------------------------------- 数据结构
@dataclass
class PathErrors:
    max_mm: float
    avg_mm: float
    max_offplane_mm: float


@dataclass
class SecondaryTrack:
    tip_frame: str
    delta: object            # np.ndarray (3,)——注解为字符串，import 期不依赖 numpy
    max_err_mm: float


@dataclass
class TransitionInfo:
    source_artifact_hash: str
    target_artifact_hash: str
    joint_limits: tuple            # (np.ndarray lo, np.ndarray hi)


@dataclass
class PlanMeta:
    library_version: str
    created_at: str                # ISO8601 UTC；不参与 artifact_hash
    mode: str
    options: dict = field(default_factory=dict)


@dataclass
class GroupPlan:
    group_name: str
    joint_names: tuple
    positions: object              # (N, dof) rad
    timestamps: object             # (N,) s
    time_source: Literal["totg", "uniform"]
    tip_frame: str
    ideal_path: object             # (N, 3) m
    err_stats: PathErrors
    secondary: dict = field(default_factory=dict)   # finger -> SecondaryTrack
    frozen_waist: Optional[object] = None           # (3,) 规划时冻结的腰值

    def q(self, i: int) -> dict:
        """第 i 帧的 {joint_name: rad} 派生视图（不参与序列化，§4.1）。"""
        return dict(zip(self.joint_names, self.positions[i]))


@dataclass
class PlanArtifact:
    schema_version: int
    kind: Literal["shape", "transition"]
    profile_id: str
    profile_hash: str
    meta: PlanMeta
    groups: dict
    schema_minor: int = SCHEMA_MINOR
    waypoints: Optional[dict] = None      # group -> (M, 3) 目标路径
    transition_info: Optional[TransitionInfo] = None

    def save(self, path):
        save(self, path)

    @classmethod
    def load(cls, path, max_schema=None):
        return load(path, max_schema)


# ---------------------------------------------------------------- 序列化
_ARRAY_FIELDS = {
    "positions", "timestamps", "ideal_path", "delta", "frozen_waist",
    "waypoints", "lim_lo", "lim_hi",
}


def _plane(obj):
    """dataclass/dict -> 纯 JSON 结构；数组字段以 b64 占位（键集记录在 _arr）。"""
    arrs = {}

    def walk(o, path):
        if isinstance(o, dict):
            return {k: walk(v, f"{path}.{k}" if path else k) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [walk(v, f"{path}[{i}]") for i, v in enumerate(o)]
        if hasattr(o, "__dataclass_fields__"):
            return walk(asdict(o), path)
        if o is None or isinstance(o, (str, int, float, bool)):
            return o
        # ndarray（鸭子判定，import 期不引 numpy）
        if hasattr(o, "shape") and hasattr(o, "tobytes"):
            key = path
            arrs[key] = o
            return {"__arr__": key}
        raise TypeError(f"不可序列化字段 {path}: {type(o)}")

    return walk(obj, ""), arrs


def _unplane(obj, loader):
    def walk(o):
        if isinstance(o, dict):
            if "__arr__" in o:
                return loader[o["__arr__"]]
            return {k: walk(v) for k, v in o.items()}
        if isinstance(o, list):
            return [walk(v) for v in o]
        return o

    return walk(obj)


def _np():
    import numpy as np
    return np


def save(art, path):
    """PlanArtifact -> ZIP(meta.json + arrays.npz)。数组 load 后置只读。"""
    np = _np()
    plane, arrs = _plane(art)
    buf = io.BytesIO()
    np.savez(buf, **{k: np.asarray(v) for k, v in arrs.items()})
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("meta.json", json.dumps(plane, ensure_ascii=False, indent=1))
        z.writestr("arrays.npz", buf.getvalue())


def load(path, max_schema=None):
    """ZIP 产物 -> PlanArtifact；major 高于库支持即抛 ContractVersionError。

    max_schema 可临时压低库支持的 major（测试用）。minor 更高携带的未知
    字段安全忽略（前向兼容，§4.5），且不参与 artifact_hash。
    """
    np = _np()
    limit = SUPPORTED_MAJOR if max_schema is None else max_schema
    with zipfile.ZipFile(path) as z:
        plane = json.loads(z.read("meta.json"))
        with np.load(io.BytesIO(z.read("arrays.npz"))) as npz:
            arrays = {k: npz[k] for k in npz.files}
    if plane["schema_version"] > limit:
        raise ContractVersionError(
            f"产物 schema_version={plane['schema_version']} 高于库支持的 {limit}"
            f"（{path}）——拒绝加载过新产物")
    for a in arrays.values():
        a.setflags(write=False)
    d = _unplane(plane, arrays)
    ti = d.get("transition_info")
    art = PlanArtifact(
        schema_version=d["schema_version"], kind=d["kind"],
        profile_id=d["profile_id"], profile_hash=d["profile_hash"],
        meta=PlanMeta(**d["meta"]),
        groups={k: _group_from(v) for k, v in d["groups"].items()},
        schema_minor=d.get("schema_minor", SCHEMA_MINOR),
        waypoints=d.get("waypoints"),
        transition_info=None if ti is None else TransitionInfo(
            ti["source_artifact_hash"], ti["target_artifact_hash"],
            tuple(ti["joint_limits"])))
    return art


def _group_from(g):
    return GroupPlan(
        group_name=g["group_name"], joint_names=tuple(g["joint_names"]),
        positions=g["positions"], timestamps=g["timestamps"],
        time_source=g["time_source"], tip_frame=g["tip_frame"],
        ideal_path=g["ideal_path"],
        err_stats=PathErrors(**g["err_stats"]),
        secondary={k: SecondaryTrack(v["tip_frame"], v["delta"], v["max_err_mm"])
                   for k, v in g["secondary"].items()},
        frozen_waist=g.get("frozen_waist"))


def artifact_hash(art):
    """内容 hash（§4.6）：全部语义字段，排除 meta.created_at 与 schema 版本号。

    版本号是表示元数据而非内容（"同内容即同值"，§4.6）；未知字段（低 minor
    库读高 minor 产物时的降级省略）天然不参与——hash 只覆盖本库认识的字段。
    算法：语义结构 canonical JSON（sort_keys）+ 各数组 tobytes（键序拼接），
    SHA-256。transition_info.joint_limits 参与。
    """
    np = _np()
    import copy
    clone = copy.deepcopy(art)
    clone.meta.created_at = ""
    clone.schema_version = 0
    clone.schema_minor = 0
    plane, arrs = _plane(clone)
    h = hashlib.sha256()
    h.update(json.dumps(plane, ensure_ascii=False, sort_keys=True).encode())
    for key in sorted(arrs):
        h.update(key.encode())
        h.update(np.ascontiguousarray(arrs[key]).tobytes())
    return h.hexdigest()


def profile_hash(profile):
    """RobotProfile 内容 hash（§4.6）：canonical JSON SHA-256。"""
    return hashlib.sha256(
        json.dumps(profile, ensure_ascii=False, sort_keys=True,
                   default=str).encode()).hexdigest()
