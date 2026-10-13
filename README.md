# Swarm Lab

面向地面车群的分层编队控制实验项目，使用 ROS1/VRPN 动捕状态、编队规划、
可选人工势场避障、底盘控制和 UDP 轮速下发。支持 bearing 向量 PI、bearing
投影矩阵和矩阵权子空间控制，以及终端输出、实时轨迹和实验数据记录。

当前默认入口配置为 `bearing_three`，公共底盘控制器为 `projected`，
控制频率为 **50 Hz**，目标速度模长上限为 **0.30 m/s**。
各实验可覆盖公共默认值，实际参数以配置模块导出的 `CONFIG` 为准。

- [实验构型与车辆映射](configs/README.md)
- [控制公式、Leader 参考、底盘与任务编排](docs/control_reference.md)
- [记录格式、误差指标与离线分析](docs/data_analysis.md)

## 环境与快速开始

实车运行需要 Python 3、ROS1（`rospy`、`geometry_msgs`）、可用的 VRPN 动捕
话题，以及到车辆底盘的 UDP 网络。Python 依赖见 [requirements.txt](requirements.txt)。
ROS 包由 ROS 环境提供，不包含在该依赖文件中。

以下命令在仓库根目录 `swarm_lab/` 执行，并使用已经加载 ROS 环境的 Python：

```bash
python3 -m pip install -r requirements.txt
python3 run_experiment.py --help
python3 run_experiment.py
```

默认配置使用 `car3`、`car6`、`car7`，地址分别为 `10.1.1.83`、
`10.1.1.86`、`10.1.1.87`。动捕订阅前缀默认为 `/vrpn_client_node`；
运行前核对配置中的车辆 ID、地址、拓扑顺序和动捕世界坐标系。

```bash
# 切换实验模块；也支持完整模块名 configs.flyingwing
python3 run_experiment.py --config flyingwing

# 关闭实时绘图，保留数据记录
python3 run_experiment.py --config bearing_three --no-plot

# 同时关闭实时绘图和数据记录
python3 run_experiment.py --config bearing_three --no-plot --no-record
```

`--no-plot` 与 `--no-record` 只控制遥测输出，仍会运行实车控制。
实时轨迹窗口需要图形显示和交互式 Matplotlib backend。Linux/WSL 可安装
Tk 支持并指定 backend：

```bash
sudo apt install python3-tk
MPLBACKEND=TkAgg python3 run_experiment.py
```

默认持续运行；`Ctrl+C` 退出。配置中的 `RuntimeConfig.duration_s` 可设置
自动停止时长。动捕丢失默认停车；控制循环结束时发送停车指令并关闭资源。
启动时先校验配置并构造 planner，再连接 ROS 和底盘。

## 架构与目录

```text
run_experiment.py       实验命令行入口
configs/                实验配置，每个模块导出 CONFIG
task_programs/         时间函数与闭环任务程序
swarm/
  domain/               状态模型、类型化配置、子空间与任务定义
  algorithms/           编队、Leader 参考、PID、底盘控制与避障
  application/          控制循环、接口、planner 工厂与运行装配
  infrastructure/       ROS 动捕、UDP、地面车执行器与遥测适配器
tools/                  离线分析、跟踪曲线与动画
tests/                 不依赖 ROS 或实车的测试
docs/                   控制参考与数据分析说明
runs/                   实验记录输出，已加入 .gitignore
```

一次控制周期的执行顺序：

```text
ROS/VRPN 状态快照
  -> ControlLoop：控制空间投影
  -> SwarmPlanner：编队反馈与运动任务
  -> VelocityCommandFilter：可选人工势场避障
  -> VehicleActuator：底盘控制与轮速分配
  -> UDP 轮速发送
  -> Console / Live Plot / CSV Recorder
```

`ControlLoop` 通过接口注入状态源、planner、速度过滤器、执行器和遥测。
实时绘图在独立进程运行。`planar_xy` 将位置与线速度的 z 分量清零，
`spatial_xyz` 保留三维状态；当前生产装配只提供 `ground_vehicle` 执行器。

## 算法与配置

| `CONFIG.algorithm` | 控制方式 | 拓扑与任务 |
| --- | --- | --- |
| `bearing` | 向量 PI，积分采用滑动窗口 | 有向 bearing 拓扑，可配置 Leader 参考 |
| `bearing_projection` | `I - ggᵀ` 投影矩阵反馈，无积分状态 | 使用相同 bearing 拓扑与参考接口 |
| `subspace` | 固定矩阵权反馈与零空间模态输入 | 无向约束边，可用时间函数或闭环任务 |

两个 bearing planner 共用 `bearing_mode`：默认 `bearing_only` 归一化实际与
期望相对向量；`bearing_based` 使用包含距离尺度的原始向量。
世界系 bearing 约束会限定构型方向；绝对平移和尺度不由 bearing-only 单独指定。
投影矩阵在两种模式下均由实际单位方向构造，不单独提供沿当前边的径向距离控制。

以下为当前配置清单，表中车辆顺序与 `CONFIG.vehicle_ids` 一致。
所有配置从 `configs/` 导入；`--config` 使用模块名，记录目录前缀使用 `CONFIG.name`。

| 模块名 | 车辆 | 主要内容 |
| --- | --- | --- |
| `bearing_leader_motion` | 5 | 单车速度、位置、世界系曲线参考 |
| `bearing_two` | 3、6 | 两车 +y 方向跟随，car3 为 Leader |
| `bearing_three` | 3、6、7 | 等腰直角三角形，car3 为 Leader；默认实验 |
| `bearing_four` | 2、3、6、7 | 正方形；当前 `leader_mask` 全为 False |
| `bearing_six` | 1–6 | 六车柔性构型，有向矩阵拓扑 |
| `bearing_eight` | 1–8 | 中间正方形、两侧柔性，car2 为 Leader |
| `subspace_two_x` | 1、4 | 沿 x 的相对运动自由 |
| `subspace_two_y` | 1、4 | 沿 y 的相对运动自由 |
| `subspace_two_xy` | 1、4 | 沿 x=y 的相对运动自由 |
| `subspace_two_aff` | 1、4 | y 向偏置 0.5 m，沿 x 相对运动自由 |
| `subspace_triangle` | 3、6、7 | 三车仿射约束三角形 |
| `subspace_six_triangles` | 1–6 | 两个独立等边三角形 |
| `subspace_four_rectangle` | 1、4、6、7 | 矩形宽高伸缩与分段平移 |
| `subspace_four_square` | 1、4、6、7 | 正方形收缩与分段平移 |
| `subspace_seven` | 1–7 | 正方形、下部伸缩与侧边偏置节点 |
| `subspace_eight_square_sliders` | 1–8 | 正方形及四个轴向滑臂 |
| `subspace_pentagon_triangle` | 1–8 | 五边形与三角形的反向移动和同步缩放 |
| `flyingwing` | 1、2、3、5、6、7、8 | 七车飞翼，时间演示或闭环目标任务 |

四个 `subspace_two_*` 模块的 `CONFIG.name` 均为 `subspace_two`；
当前不存在 `configs.subspace_two` 模块。Bearing 的有效 Leader 由
`topology.leader_mask` 选择，`VehicleConfig.role` 标签不能替代此掩码。

Subspace 使用 `A(r-b)=0` 约束相对位移，沿 `ker(A)` 的相对运动自由。
任务模态必须满足 `L_A Z=0`；固定系数/时间函数与 `task_feedback` 不能混用。
闭环支持位置、相对位置、边长和投影间距目标，以及 `Sequence`、`Parallel`、
`Delay`、`Hold`、`Release`。`flyingwing` 默认 `USE_TASK_PROGRAM=False`，
运行时间演示；闭环示例位于 `task_programs/flyingwing_goals.py`。

## 关键参数

公共配置定义在 [swarm/domain/config.py](swarm/domain/config.py)，
闭环任务定义在 [swarm/domain/tasks.py](swarm/domain/tasks.py)。
下面列出公共默认值；实验覆盖值及具体边约束见对应配置文件。

| 参数 | 公共默认值 | 含义 |
| --- | --- | --- |
| `RuntimeConfig.control_hz` | `50.0` Hz | 控制频率 |
| `command_speed_limit_mps` | `0.30` m/s | 目标速度向量模长上限，不得超过底盘物理上限 |
| `mocap_prefix` | `/vrpn_client_node` | 动捕话题前缀 |
| `stop_on_loss_of_mocap` | `True` | 动捕丢失停车 |
| `udp_port` / `udp_timeout_s` | `12345` / `0.2` s | UDP 端口与 socket 超时 |
| `duration_s` | `None` | 自动停止时长 |
| `FormationControlConfig.k_p` / `k_i` | `1.2` / `0.5` | Bearing PI 增益 |
| `integral_window_s` / `integral_limit` | `2.0` s / `0.4` | 编队积分窗口与逐轴限幅 |
| `output_limit` / `projection_k_p` | `0.4` / `1.2` | 编队逐轴限幅与投影增益 |
| `bearing_mode` | `bearing_only` | Bearing 向量处理模式 |
| `leader_bearing_enabled` | `False` | 在 Leader 运动参考上叠加编队修正 |
| `bearing_deadband_enabled` | `False` | 角误差滞环死区，进入/退出阈值为 `1.5°` / `3°` |
| `SubspaceControlConfig.k` | `1.0` | 原始矩阵权反馈增益 |
| `modes` / `coefficients` / `task_feedback` | `None` / `()` / `None` | 无默认任务输入 |
| `GroundVehicleControlConfig.controller_type` | `projected` | 可选 `differential`、`projected`、`mecanum` |
| `track_width_m` / `max_speed_mps` | `0.053` m / `0.3` m/s | 轨距与物理速度上限 |
| `wheel_command_min` / `wheel_command_max` | `-100` / `100` | 归一化轮速指令范围 |
| `projected_lookahead_m` / `projected_max_yaw_rate_rps` | `0.06` m / `11.32` rad/s | 投影控制转向参数 |
| `ObstacleAvoidanceConfig.enabled` / `avoid_other_vehicles` | `False` / `False` | 避障总开关与车间排斥开关 |
| `vehicle_radius_m` / `influence_distance_m` | `0.03` m / `0.1` m | 车辆半径与排斥影响距离 |
| `repulsive_gain` / `min_distance_epsilon_m` | `0.0005` / `0.01` m | 排斥增益与最小计算距离 |
| `TelemetryConfig.console_hz` / `live_plot_hz` / `record_hz` | `10` / `10` / `5` Hz | 终端、绘图与记录频率，默认均启用 |
| `record_root` / `record_flush_interval_s` | `runs` / `1.0` s | 记录目录与刷新间隔 |

底盘、编队与 Leader 位置环 PID 分开配置；通用 `PIDConfig.integral_window_s`
默认 1 秒，编队 PI 的公共窗口为 2 秒。位置外环 `output_limit` 为 `None`，
由合成后的全局向量限速约束。控制器细节、PID 字段和任务参数见
[控制参考](docs/control_reference.md)。

人工势场在编队输出后叠加静态圆形障碍物/其他车辆排斥，再限速。
总开关开启不等同于开启车间避障；各实验的两个开关可能不同。
当前实现不包含场地路径规划，子空间方向约束也不单独提供碰撞保证。

## 数据记录与分析

默认输出到 `runs/<CONFIG.name>_<时间戳>/`。目录相对于项目根目录解析：

| 文件 | 内容 |
| --- | --- |
| `metadata.json` | 配置快照，函数保存为 `module.qualname` |
| `trajectory.csv` | 实测状态、目标速度、底盘控制量、轮速与发送状态 |
| `bearing_edges.csv` | 有向 bearing 边的方向与角误差；subspace 运行保留空表头 |
| `subspace_edges.csv` / `subspace_summary.csv` | Subspace 逐边归一化误差与全局 RMS，按需生成 |
| `tasks.jsonl` | 闭环任务状态、目标误差与分配残差，按需生成 |

把以下 `RUN_DIR` 替换为实际运行目录：

```bash
RUN_DIR="runs/实际运行目录"
python3 tools/postprocess.py "$RUN_DIR"
python3 tools/plot_speed_tracking.py "$RUN_DIR"
python3 tools/plot_constraint_animation.py "$RUN_DIR" --fps 20
python3 tools/plot_error_animation.py "$RUN_DIR" --fps 20
```

后处理生成 `analysis.json` 与 `analysis.png`，subspace 另有 `edge_error.png`。
跟踪工具分别生成逐车速度和航向图片；约束工具生成 `trajectory_constraints.gif`。
误差动画工具用于 subspace，输出 `analysis.gif` 和 `edge_error.gif`。
普通轨迹动画及固定场地标注见 [离线分析说明](docs/data_analysis.md)。

Bearing 角误差单位为度；后处理全局误差为有向边误差的二范数，不除以边数。
Subspace 使用谱范数归一化边误差 RMS；非零偏置下误差可大于 1。
稳态定义、无效样本处理和动画选项见离线分析说明。

## 离线验证与扩展

测试无需 ROS、动捕和实车：

```bash
python3 -m unittest discover -s tests -v
```

新增 planner 在 `swarm/algorithms/` 实现 `step(snapshot, dt)` 与 `reset()`，
通过 `swarm/application/planner_factory.py` 注册，并添加配置与对应测试。
平台适配通过 `swarm/application/interfaces.py` 的接口接入。
控制公式、配置约束和扩展步骤见 [控制参考](docs/control_reference.md)。
