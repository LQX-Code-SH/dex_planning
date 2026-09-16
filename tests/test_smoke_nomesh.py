#!/usr/bin/env python3
"""无 mesh 冒烟（CI §8.4 第 4 项）：tiny URDF 走通 profile→重写→Robot→TOTG 骨架。

用法: python tests/test_smoke_nomesh.py   （打印 SMOKE NOMESH: ALL PASS）
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
import numpy as np

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")


def main():
    from tienkung_planning.core.profile import load_profile, resolve_description
    from tienkung_planning.core.tess import Robot, totg_traj

    profile = load_profile(os.path.join(FIXTURES, "smoke_bot.yaml"))
    with tempfile.TemporaryDirectory() as cache:
        urdf, srdf = resolve_description(profile, description_root=FIXTURES,
                                         cache_dir=cache)
        robot = Robot.from_files(urdf, srdf)   # 缓存目录存活期内装载
    st = robot.env.getState()
    assert "link2" in st.link_transforms, "FK 冒烟：link2 不在链上"

    info = robot.get_manipulator_info("arm", tcp_frame="link2",
                                      working_frame="base_link")
    Q = np.linspace([0.0, 0.0], [0.5, 0.8], 15)
    out = totg_traj(Q, ("j1", "j2"), robot.env, info)
    assert out is not None, "TOTG 骨架失败"
    Q2, ts = out
    assert len(ts) == 15 and ts[-1] > 0
    print(f"SMOKE NOMESH: ALL PASS (links={len(st.link_transforms)}, "
          f"dur={ts[-1]:.2f}s)")


if __name__ == "__main__":
    main()
