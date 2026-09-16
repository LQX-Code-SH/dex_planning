#!/usr/bin/env python3
"""facade 门面测试（v0.2）：能力件走无 mesh 冒烟机型，规划器走真机 profile。

用法:
    python tests/test_facade.py              # 全部（真机部分需 dex env + ROS + mesh）
    python tests/test_facade.py --nomesh     # 只跑能力件（裸环境可跑）

覆盖 §13.1 用法①②：
  - TimeParamApi: TOTG 路径 / 显式 group / UnavailableError（无速度限位）/ ValueError
  - CollisionApi: 单状态与状态序列
  - TienKungPlanner: plan_shape → save/load roundtrip → check（默认口径 ok）→
    plan_transition（同实例同源产物 + transition_info hash 对账）
打印 FACADE TESTS: ALL PASS。
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))

import numpy as np

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
SMOKE = os.path.join(FIXTURES, "smoke_bot.yaml")


def test_capabilities():
    from tienkung_planning.contracts import UnavailableError
    from tienkung_planning.facade import CollisionApi, TimeParamApi

    with tempfile.TemporaryDirectory() as cache:
        tp = TimeParamApi.from_profile(SMOKE, description_root=FIXTURES,
                                       cache_dir=cache)
        Q = np.linspace([0.0, 0.0], [0.5, 0.8], 15)
        out = tp.parameterize(Q, ("j1", "j2"))
        assert out.time_source == "totg", out.time_source
        assert len(out.timestamps) == len(out.positions)
        assert (np.diff(out.timestamps) > 0).all(), "ts 非严格递增"
        out2 = tp.parameterize(Q, ("j1", "j2"), group="arm", dt=0.05)
        assert out2.time_source == "totg"

        tp2 = TimeParamApi.from_profile(SMOKE, description_root=FIXTURES,
                                        cache_dir=cache)
        tp2._robot.get_joint_limits = lambda g: {"j1": {"velocity": 0.0}}
        try:
            tp2.parameterize(Q, ("j1", "j2"))
            raise AssertionError("无速度限位应抛 UnavailableError")
        except UnavailableError:
            pass
        try:
            tp.parameterize(Q, ("zz1", "zz2"))
            raise AssertionError("未知关节应抛 ValueError")
        except ValueError:
            pass

        col = CollisionApi.from_profile(SMOKE, description_root=FIXTURES,
                                        cache_dir=cache)
        r1 = col.check(np.array([0.0, 0.0]), ("j1", "j2"))
        r2 = col.check(np.linspace([0.0, 0.0], [0.5, 0.8], 5), ("j1", "j2"))
        assert isinstance(r1, list)
        assert len(r2) == 5 and all(isinstance(x, list) for x in r2)
    print("  capabilities (smoke_bot): PASS")


def test_planner():
    from tienkung_planning.contracts import PlanArtifact, artifact_hash
    from tienkung_planning.core.profile import find_description_root
    try:
        find_description_root()
    except Exception:
        print("  planner (真机): SKIP（找不到 description root）")
        return

    from tienkung_planning.facade import PlannerOptions, TienKungPlanner

    # §13.1 用法①逐字
    planner = TienKungPlanner.from_profile(
        "tienkung_dex", options=PlannerOptions(arm="right", mode="full"))
    plan = planner.plan_shape("circle", size=0.10)

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "circle.traj")
        plan.save(path)
        art = PlanArtifact.load(path)
    assert art.kind == "shape" and sorted(art.groups) == ["right"]
    assert artifact_hash(art) == artifact_hash(plan), "roundtrip hash 漂移"

    report = planner.check(art)          # 默认口径：结构 + 限位 + 误差
    assert report.ok, report.errors

    plan2 = planner.plan_shape("line")
    trans = planner.plan_transition(plan, plan2)
    assert trans.kind == "transition"
    assert trans.transition_info.source_artifact_hash == artifact_hash(plan)
    assert trans.transition_info.target_artifact_hash == artifact_hash(plan2)
    for s, gp in trans.groups.items():
        assert gp.time_source == "uniform"
        assert len(gp.timestamps) == len(gp.positions)
    rt = planner.check(trans)
    assert rt.ok, rt.errors

    # 同源校验：跨 profile 产物必须拒绝（内存构造，不改磁盘产物）
    import copy
    from tienkung_planning.contracts import PlannerError
    bad = copy.deepcopy(plan2)
    bad.profile_hash = "x" * 64
    try:
        planner.plan_transition(plan, bad)
        raise AssertionError("跨源产物应被拒绝")
    except PlannerError:
        pass
    print("  planner (真机 right/full): PASS")


def main():
    nomesh_only = "--nomesh" in sys.argv[1:]
    test_capabilities()
    if not nomesh_only:
        test_planner()
    print("FACADE TESTS: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
