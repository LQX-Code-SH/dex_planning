# 天工 Dex 双臂双手轨迹规划

给 TienKung Dex 人形机器人规划**双臂 + 灵巧手**的末端轨迹：让指尖沿圆、三角、
直线等形状精确描边。可以开 RViz 动画看效果，也可以当作 Python 库离线规划，
把轨迹文件交给真机执行程序。

## 一图看懂

```text
                     ┌──────────────────────────────────┐
                     │ 1. 说清需求                       │
                     │    哪只手、哪根手指、什么形状        │
                     └────────────────┬─────────────────┘
                                      │
                                      v
                     ┌──────────────────────────────────┐
                     │ 2. 库自动规划                      │
                     │    IK 求解、碰撞检查、时间参数化     │
                     └────────────────┬─────────────────┘
                                      │
                                      v
                              3. 打算怎么用？
                                      │
                ┌─────────────────────┴─────────────────────┐
                │                                           │
                v                                           v
┌──────────────────────────────┐            ┌──────────────────────────────┐
│ RViz 看动画                   │            │ 上真机                        │
│ 可视化 demo，实时演示描边       │            │ 保存为 .traj 轨迹文件           │
└──────────────────────────────┘            └───────────────┬──────────────┘
                                                            │
                                                            v
                                            ┌──────────────────────────────┐
                                            │ 4. 执行程序读取                │
                                            │    只装本库，不装 tesseract    │
                                            └──────────────────────────────┘
```

## 快速开始

### 方式一：RViz 演示（看效果）

```bash
./run_rviz_sim.sh --arm both --mode full --waist free
```

机器人双臂镜像协同，指尖现场规划并沿形状描边。想改什么换什么参数：

| 参数 | 作用 |
|---|---|
| `--arm right\|left\|both` | 单臂或双臂镜像协同 |
| `--mode full\|fixed\|tcp` | 臂+指耦合全链 / 臂 7-DOF / 仅 tcp 基线 |
| `--waist fixed\|free\|all` | 腰全锁（默认）。**解锁腰时默认选 `free`**：只放 yaw（水平旋转，扩工作空间、风险低）；`all` 三关节全放**默认不建议**（roll/pitch 改变躯干姿态、会顶 ±25° 限位并触发腰协商），仅确需弯腰抓取时用。**腰动需配 `--arm both`**——腰是左右共享的物理关节，动腰必然同时改变两臂基座 |
| `--fingers index,thumb` | 多指刚体联动（其余手指平行随动） |

### 方式二：Python 库（写代码）

```python
from tienkung_planning.facade import TienKungPlanner, PlannerOptions

planner = TienKungPlanner.from_profile("tienkung_dex",
                                       options=PlannerOptions(arm="both", waist="free"))
plan = planner.plan_shape("circle", size=0.10)
plan.save("circle.traj")          # 轨迹文件，交给执行侧
```

规划失败会抛 `PlannerError` 并带原因；还有两个独立能力件，不必进整套管线：

```python
from tienkung_planning.facade import CollisionApi, TimeParamApi

hits = CollisionApi.from_profile("tienkung_dex").check(q, joint_names)   # 碰撞检查
tp = TimeParamApi.from_profile("tienkung_dex").parameterize(Q, names)    # 时间参数化
```

### 方式三：命令行（脚本 / 跨机器）

```bash
python -m tienkung_planning.apps.traj_plan  --arm both --shape circle -o out/   # 规划
python -m tienkung_planning.apps.traj_check -i out/circle.traj --contacts       # 校验
python -m tienkung_planning.apps.traj_map   -i out/circle.traj --every 4        # 抽稀
python -m tienkung_planning.apps.time_param -i out/circle.traj                  # 时间参数化
```

## 轨迹文件去哪了

`.traj` 是一个 ZIP 包（`meta.json` + `arrays.npz`），自带内容 hash 和版本号。
**执行侧只装本库、不装 tesseract** 就能读：

```python
from tienkung_planning.contracts import PlanArtifact

plan = PlanArtifact.load("circle.traj")
q = plan.groups["right"].positions[0]     # 逐帧关节角 → 喂给 SDK
```

## 想改 / 想扩展

| 想做什么 | 怎么做 |
|---|---|
| 换一种机器人（同族变体） | 写一份 RobotProfile YAML（`src/tienkung_planning/robots/`），不改代码 |
| 加一种模式 | 实现 `ModeStrategy` 协议，注册进 `MODES` |
| 加一种形状 | 写一个普通函数 |
| 理解某层职责 | 见下方架构图 + `docs/封装方案.md` |

## 项目架构

五层（L0–L4），依赖只准向下 import；tesseract 细节只出现在 `core/tess.py` 一个文件里。

```text
谁在用：你的 Python 代码、demo（RViz） -> facade/（L3）
        CLI（python -m tienkung_planning.apps.*） -> apps/（L4）

┌──────────────────────────────────────────────────────────────────────────┐
│ L4 应用层  apps/  （可选安装，pip install .[app]）                       │
│  traj_plan、time_param、traj_check、traj_map                             │
│  demo（RViz 动画）留在原仓库：circle_planning_7axis.py                   │
├──────────────────────────────────────────────────────────────────────────┤
│ L3 门面层  facade/  （唯一推荐入口，import 本包即需 tesseract）          │
│  planner.py      TienKungPlanner、PlannerOptions、CheckReport            │
│  collision.py    CollisionApi                                            │
│  time_param.py   TimeParamApi、TimedTrajectory                           │
├──────────────────────────────────────────────────────────────────────────┤
│ L2 核心层  core/  （不含 tess.py）                                       │
│  pipeline.py   装配与产物：make_context、build_robot、make_artifact      │
│  mode.py       模式契约：ModeConfig、ModeStrategy                        │
│  profile.py    机型装载：load_profile、resolve_description（不碰 tess）  │
│  checks.py     校验三件：check_structure / check_errors / check_contacts │
├──────────────────────────────────────────────────────────────────────────┤
│ L1 防腐层  core/tess.py  （全库唯一 tesseract 入口）                     │
│  contact_test、acm、descartes_plan、totg、totg_traj                      │
├──────────────────────────────────────────────────────────────────────────┤
│ L0 契约层  contracts/  （零重依赖：仅 numpy + 标准库，裸 venv 可 import）│
│  artifact.py     PlanArtifact、GroupPlan、save/load、artifact_hash       │
│  exceptions.py   TienKungPlanningError、UnavailableError、PlannerError   │
│  robots/*.yaml   机型配置（数据，非代码）                                │
└──────────────────────────────────────────────────────────────────────────┘
依赖方向：只准向下 import（L4 -> L3 -> L2 -> L1 -> L0）
执行侧 P-B 只依赖 L0：import tienkung_planning.contracts 即可读写 .traj
```

## 改了代码？跑这些

```bash
tests/run_regression.sh                    # 16 种组合回归（改逻辑后必跑）
tests/run_golden.sh                        # 重捕 golden 基线
python tests/golden_compare.py tests/golden /tmp/golden_step0   # 与基线比对
python tests/test_facade.py                # 门面接口
python tests/test_contracts.py             # 产物契约
python tests/test_smoke_nomesh.py          # 无 mesh 冒烟
PYTHONPATH=src tests/bare_venv_import.sh   # 执行侧零依赖红线
```

## 更多文档

| 文档 | 内容 |
|---|---|
| `docs/封装方案.md` | 完整设计：契约定义、架构决策、评审记录 |
| `docs/扩展指南.md` | 接新机型 / 新模式的步骤 |
| `docs/实施计划.md` | 迁移过程与执行记录 |
