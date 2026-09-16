#!/usr/bin/env python3
"""契约层测试（CI 清单 §8.4 第 3 项）：序列化往返 + 版本拒绝 + hash 稳定性。

用法: python tests/test_contracts.py   （打印 CONTRACT TESTS: ALL PASS）
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
import numpy as np

from tienkung_planning.contracts import (
    ContractVersionError, GroupPlan, PathErrors, PlanArtifact, PlanMeta,
    SecondaryTrack, TransitionInfo, artifact_hash, load, profile_hash, save)
from tienkung_planning.contracts.artifact import SCHEMA_MAJOR


def _sample_artifact():
    return PlanArtifact(
        schema_version=SCHEMA_MAJOR, kind="shape",
        profile_id="tienkung_dex", profile_hash="p" * 64,
        meta=PlanMeta(library_version="0.1.0",
                      created_at="2026-09-16T00:00:00+00:00",
                      mode="full", options={"arm": "both", "waist": "free"}),
        groups={
            "right": GroupPlan(
                group_name="right_arm_index",
                joint_names=("a", "b", "c"),
                positions=np.arange(6, dtype=float).reshape(2, 3),
                timestamps=np.array([0.0, 0.5]),
                time_source="totg", tip_frame="right_index_tip_link",
                ideal_path=np.ones((2, 3)),
                err_stats=PathErrors(0.006, 0.003, 0.004),
                secondary={"thumb": SecondaryTrack(
                    "right_thumb_tip_link", np.array([0.0, 0.1, 0.0]), 0.2)},
                frozen_waist=np.array([0.1, 0.2, 0.3])),
        },
        waypoints={"right": np.zeros((2, 3))})


def test_roundtrip(tmp):
    art = _sample_artifact()
    p = os.path.join(tmp, "a.traj")
    art.save(p)
    back = PlanArtifact.load(p)
    assert back.schema_version == art.schema_version
    assert back.groups["right"].group_name == "right_arm_index"
    assert back.groups["right"].joint_names == ("a", "b", "c")
    assert np.array_equal(back.groups["right"].positions, art.groups["right"].positions)
    assert back.groups["right"].q(1) == {"a": 3.0, "b": 4.0, "c": 5.0}
    assert back.groups["right"].secondary["thumb"].max_err_mm == 0.2
    assert np.array_equal(back.groups["right"].frozen_waist, [0.1, 0.2, 0.3])
    assert back.meta.options == {"arm": "both", "waist": "free"}
    # 只读不变量（§4.1）
    try:
        back.groups["right"].positions[0, 0] = 9
        raise AssertionError("load 后数组应只读")
    except ValueError:
        pass
    # 往返稳定：save(load(save)) 内容 hash 不变（created_at 除外本就同值）
    p2 = os.path.join(tmp, "b.traj")
    back.save(p2)
    assert artifact_hash(PlanArtifact.load(p2)) == artifact_hash(back)
    # created_at 改变不影响 hash（§4.6 排除字段）
    back.meta.created_at = "1999-01-01T00:00:00+00:00"
    assert artifact_hash(back) == artifact_hash(art)


def test_version_reject(tmp):
    art = _sample_artifact()
    art.schema_version = SCHEMA_MAJOR + 999
    p = os.path.join(tmp, "future.traj")
    art.save(p)
    try:
        PlanArtifact.load(p)
        raise AssertionError("过新产物应被拒绝")
    except ContractVersionError:
        pass
    # max_schema 压低：同产物在更老库下同样拒绝
    art2 = _sample_artifact()
    art2.schema_version = 2
    p2 = os.path.join(tmp, "v2.traj")
    art2.save(p2)
    try:
        PlanArtifact.load(p2, max_schema=1)
        raise AssertionError("major=2 应在支持 1 的库上被拒")
    except ContractVersionError:
        pass


def test_minor_forward_compat(tmp):
    """低 minor 库读高 minor 产物：未知字段安全忽略且不参与 hash（§4.5/§4.6）。"""
    art = _sample_artifact()
    p = os.path.join(tmp, "minor.traj")
    art.save(p)
    # 手工把产物改成 schema_minor=7 并塞进未知字段（模拟未来版本）
    import zipfile, io
    with zipfile.ZipFile(p) as z:
        meta = json.loads(z.read("meta.json"))
        arrays = z.read("arrays.npz")
    meta["schema_minor"] = 7
    meta["unknown_future_field"] = {"x": 1}
    p2 = os.path.join(tmp, "minor7.traj")
    with zipfile.ZipFile(p2, "w") as z:
        z.writestr("meta.json", json.dumps(meta))
        z.writestr("arrays.npz", arrays)
    back = PlanArtifact.load(p2)     # 不抛——前向兼容
    assert artifact_hash(back) == artifact_hash(art)   # 未知字段不进 hash


def test_transition_artifact(tmp):
    art = _sample_artifact()
    art.kind = "transition"
    art.transition_info = TransitionInfo("h1", "h2",
                                         (np.array([0.0, 0.0]), np.array([1.0, 1.0])))
    p = os.path.join(tmp, "t.traj")
    art.save(p)
    back = PlanArtifact.load(p)
    assert back.transition_info.source_artifact_hash == "h1"
    assert np.array_equal(back.transition_info.joint_limits[1], [1.0, 1.0])


def test_profile_hash(_=None):
    a = {"robot": "tienkung_dex", "fingers": {"index": {"mult": 1.155}}}
    b = {"fingers": {"index": {"mult": 1.155}}, "robot": "tienkung_dex"}
    assert profile_hash(a) == profile_hash(b)      # 键序无关
    b["fingers"]["index"]["mult"] = 1.0
    assert profile_hash(a) != profile_hash(b)


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as tmp:
        for t in (test_roundtrip, test_version_reject, test_minor_forward_compat,
                  test_transition_artifact, test_profile_hash):
            t(tmp)
            print(f"  {t.__name__}: PASS")
    print("CONTRACT TESTS: ALL PASS")
