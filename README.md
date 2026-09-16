# 天工 Dex 双臂双手轨迹规划 Demo

基于 tesseract（nanobind bindings 0.35）+ ROS 2 Jazzy 的 TienKung Dex 人形机器人
双臂双手笛卡尔轨迹现场规划与 RViz2 演示。单进程运行，不产生任何中间文件。

## 能力

- 双臂：`--arm right|left|both`（左臂种子由 2⁷ 符号模式 FK 搜索自动镜像标定）
- 腰部冗余：`--waist free` 将腰 3 关节并入变量空间（both 时腰归右臂、左臂冻结）
- 手指：五指任选（欠驱动耦合显式参数化，补偿 tesseract 不识别 URDF `<mimic>`）；
  `--fingers index,thumb` 多指刚体联动（手型保持、副指平行随动，仅验证不做约束）
- 末端模式：`full`（臂+指耦合全链）/ `fixed`（手指定值+臂 7-DOF）/ `tcp`（回归基线）
- 轨迹质量：TOTG 时间参数化、形状间过渡轨迹（碰撞扫描+随机路标绕行）、
  启动可达性预检、失败原因可视化
- 精度：主指尖路径误差 ≤0.01mm，多指随动误差 ≤0.009mm；单形状规划 0.2–1.7s

## 运行

```bash
./run_rviz_sim.sh                            # 右臂食指，full 模式，四种形状轮换
./run_rviz_sim.sh --arm both --mode full     # 双臂镜像协同
./run_rviz_sim.sh --waist free               # 腰 3 关节并入变量空间
./run_rviz_sim.sh --fingers index,thumb      # 多指刚体联动
./run_rviz_sim.sh --fingers index,thumb --waist free   # 组合使用
```

环境依赖：conda env `dex`（Python 3.12 + tesseract-robotics-nanobind），
ROS 2 Jazzy（robot_state_publisher / rviz2）。详见 `run_rviz_sim.sh`。

## 回归

```bash
tests/run_regression.sh    # 15 单指组合 + 4 多指组合 × 4 形状，headless ALL PASS
```

## 作为库使用（tienkung_planning）

包结构（封装方案 v2.1.1，`docs/封装方案.md`）：

    src/tienkung_planning/
    ├── contracts/        L0 产物契约（PlanArtifact 读写，import 零第三方依赖）
    ├── core/             L1 防腐层 tess.py（tesseract 唯一居所）+ profile/mode/pipeline/checks
    ├── facade/           L3 门面：TienKungPlanner / CollisionApi / TimeParamApi
    ├── robots/           RobotProfile YAML（机型 = 配置）
    └── apps/             CLI：traj_plan / time_param / traj_check / traj_map

```python
# 门面（v0.2）：组合便捷入口或按能力取用
from tienkung_planning.facade import TienKungPlanner, PlannerOptions
planner = TienKungPlanner.from_profile("tienkung_dex",
                                       options=PlannerOptions(arm="both", waist="free"))
plan = planner.plan_shape("circle", size=0.10)
plan.save("circle.traj")
```

```bash
# CLI 等价入口
python -m tienkung_planning.apps.traj_plan --arm both --mode full --waist free \
    --shape circle -o out/
# 执行侧（无需装 tesseract）读产物
python -m tienkung_planning.apps.traj_check -i out/circle.traj
python -m tienkung_planning.apps.traj_map  -i out/circle.traj -o mapped.traj --every 4
# 时间参数化（URDF 须含 velocity 限位）
python -m tienkung_planning.apps.time_param -i mapped.traj
```

扩展方式见 `docs/扩展指南.md`；消费方式（组合而非继承、submodule 优先）
见 `docs/封装方案.md` §13。

## 测试

```bash
tests/run_golden.sh                 # 19 组合 golden 基线（19/19 ALL PASS）
python tests/golden_compare.py tests/golden /tmp/golden_step0
python tests/test_contracts.py      # 契约往返/版本拒绝
python tests/test_smoke_nomesh.py   # 无 mesh 管线骨架
python tests/test_facade.py         # 门面三件（§13.1 用法①②）
PYTHONPATH=src tests/bare_venv_import.sh   # N9 裸 venv import 红线
```
