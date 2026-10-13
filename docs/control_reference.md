# 控制与配置参考

[返回项目首页](../README.md) · [实验构型](../configs/README.md)

本文记录控制公式、配置片段和任务编排语义。参数实际值以实验模块的 `CONFIG` 及 [公共配置定义](../swarm/domain/config.py) 为准；配置片段用于说明字段，需要合并到完整的 `ExperimentConfig` 中。

## Bearing only/based 模式

两个 bearing planner 共用 `FormationControlConfig.bearing_mode`：

```python
FormationControlConfig(bearing_mode="bearing_only")   # 默认
FormationControlConfig(bearing_mode="bearing_based")
```

设原始相对位置为 `r_ij=p_j-p_i`，拓扑中的期望向量为 `r_ij*`，算法输入为：

```text
bearing_only:  q_ij = r_ij / ||r_ij||, q_ij* = r_ij* / ||r_ij*||
bearing_based: q_ij = r_ij,            q_ij* = r_ij*
```

因此 PI planner 在 only 模式下计算方向误差，在 based 模式下计算包含距离的
`mean_j(q_ij-q_ij*)`。based 模式会把拓扑向量的模长解释为期望相对距离；
动捕位置使用米时，期望向量也应按米设置。现有配置未指定该开关，继续使用
默认的 `bearing_only`，原控制结果不变。

投影 planner 中，only/based 也只改变误差相关向量 `q_ij` 和 `q_ij*`；投影
矩阵始终由实际相对向量的单位方向计算，不使用原始距离构造投影矩阵。

## Bearing 角误差滞环死区

两种 bearing planner 均提供逐车角误差施密特死区，当前公共默认值为关闭。启用后，控制器
使用该车所有有效出边的
最大 bearing 角误差：`planar_xy` 使用投影后的二维向量，
`spatial_xyz` 使用完整三维向量。误差降至
`FormationControlConfig.bearing_deadband_enter_deg=1.5°` 时进入稳定状态，
误差达到 `bearing_deadband_exit_deg=3°` 时恢复控制；两个阈值之间保持原
状态。稳定状态下 bearing 修正精确为零；PI planner 同时复位对应积分状态，
避免接近目标构型时微小速度向量变号导致目标朝向跳变。Leader 的预设运动
参考不受影响，只抑制其可选的 bearing 修正分量。

`bearing_deadband_enabled=True` 可启用该机制。参数会随实验配置写入
`metadata.json`，并要求满足
`0 <= bearing_deadband_enter_deg < bearing_deadband_exit_deg <= 180`。
两种向量模式的 deadband 均先把向量归一化后计算夹角，因此 based 模式的
距离误差不会改变 deadband 的角度判定。

## Bearing 投影矩阵控制

`algorithm="bearing_projection"` 选择无积分状态的投影矩阵 planner。对
每条有效有向边，控制器使用上述模式选出的 `q_ij` 和 `q_ij*` 计算：

```text
g_ij = r_ij / ||r_ij||
P_gij = I - g_ij g_ij^T

bearing_only:  epsilon_ij = g_ij - r_ij* / ||r_ij*||
bearing_based: epsilon_ij = r_ij - r_ij*

u_i = projection_k_p sum_j w_ij P_gij epsilon_ij
```

实现按每条边依次计算 `epsilon_ij`、左乘 `P_gij`、乘
`projection_k_p`，最后对邻居贡献累加。误差定义为“实际减期望”，因此增益
前使用正号；若将误差定义改成“期望减实际”，这里才需要负号。控制链路不再
定义或保存独立的 `projection_error`。

两种模式的 `P_gij` 都是严格的正交投影矩阵，并且实际相对向量长度变化但
方向不变时，投影矩阵保持不变。based 模式只保留期望向量的距离尺度以及
公开误差向量中的原始距离信息，不把实际距离代入投影矩阵。该投影项不单独
生成沿当前边方向的径向距离控制；PI planner 的 based 模式直接使用原始
向量差，因此包含径向距离误差。

当前拓扑没有单独的边权字段，因此对每辆车的有效邻居使用均匀归一化权重
`w_ij=1/|N_i|`。`projection_k_p`、逐轴 `output_limit`、最终速度模长上限、
leader 参考合成和 bearing deadband 继续生效。PI planner 独立使用 `k_p`、
`k_i`、`integral_limit` 和 `integral_window_s`；这些 PI 参数不参与投影控制
计算。任一模式下，零长度实际相对位置都没有单位方向，投影 planner 会记录
该边及当前模式的误差向量，但不使用该边计算投影修正。

原 `algorithm="bearing"` 的向量 PI 控制律保持不变。两个 planner 输出相同
的 `PlannerResult` 和 `VelocityCommand`，其中 `error_vector` 记录当前模式的
`mean_j(q_ij-q_ij*)`，因此现有遥测、避障、底盘控制和 UDP 链路无需调整。

## Leader 运动参考

单车调试配置位于 `configs/bearing_leader_motion.py`。修改文件顶部的
`REFERENCE_MODE` 可在 `time_velocity`、`time_position` 和 `xy_curve`
之间切换；`POSITION_PID_KP/KI/KD/INTEGRAL_LIMIT` 用于调节时间位置和
静态曲线共用的外环位置 PID。位置模式使用绝对世界坐标，运行前必须把
`POSITION_ORIGIN_X_M`、`POSITION_ORIGIN_Y_M` 设置为车辆的实测启动位置。
静态曲线也使用绝对世界坐标，应调整 `CURVE_Y_OFFSET_M`，使车辆启动点
`(x_start, y_start)` 满足 `leader_xy_curve(x_start) ≈ y_start`，避免启动时
产生突变的法向纠偏。
该文件只暴露输入模式、三类轨迹参数和位置环 PID 参数。运行时、底盘、
formation、避障和遥测参数统一使用 `swarm/domain/config.py` 中的公共默认值；
当前公共目标速度上限为 `0.30 m/s`。默认运行配置仍为 `bearing_three`。

`leader_target_velocities` 保留原固定三维速度格式，也可为每个 leader
独立配置运动参考：

```python
from math import sin

from swarm.domain.config import (
    ExperimentConfig,
    FormationControlConfig,
    PIDConfig,
    TimePositionReference,
    TimeVelocityReference,
    XYCurveReference,
)


def car1_velocity(time_s):
    """Return a world-frame time-varying velocity."""

    return 0.08, 0.03 * sin(time_s), 0.0


def car2_position(time_s):
    """Return an absolute world-frame target position."""

    return 0.4 + 0.05 * time_s, 0.6


def car6_curve_y(x_m):
    """Return the world-frame y coordinate of a static curve."""

    return 0.3 * sin(x_m)


CONFIG = ExperimentConfig(
    # 其余实验参数
    leader_target_velocities={
        "car1": TimeVelocityReference(car1_velocity),
        "car2": TimePositionReference(car2_position),
        "car6": XYCurveReference(car6_curve_y, speed_mps=0.06),
    },
    leader_position_pid=PIDConfig(0.8, 0.0, 0.0, 1.0, None),
    formation_control=FormationControlConfig(leader_bearing_enabled=True),
)
```

各引用的语义如下：

- 固定三元组：世界坐标系固定速度，兼容原配置。
- `TimeVelocityReference`：函数参数为有效控制累计时间，返回二维或三维
  世界系速度。
- `TimePositionReference`：函数参数为有效控制累计时间，返回二维或三维
  世界系绝对位置；二维函数令 `vz=0`。
- `XYCurveReference`：函数以当前世界系 `x` 为输入并返回目标 `y`，使用
  `speed_mps` 沿曲线切向运动，位置 PID 修正法向偏差。该模式适用于单值
  `y=f(x)` 曲线，不表示竖直线或回环曲线。

配置函数应定义为配置模块顶层的具名函数。`metadata.json` 记录函数的
`module.qualname` 标识，不保存函数源码、lambda 捕获值或闭包状态。

`leader_bearing_enabled=False` 是兼容默认值，leader 保持原固定参考逻辑。
启用后，leader 的预设参考速度和现有 bearing PI 分量先相加，再共同使用
`RuntimeConfig.command_speed_limit_mps` 做一次向量限幅：

```text
leader_command = vector_limit(reference_velocity + bearing_velocity)
```

两个分量不会分别使用全局速度上限。Bearing PI 原有的逐轴输出保护仍保留，
位置外环 PID 的 `output_limit` 必须为 `None`，避免在合成前形成第二个速度
饱和环节。Follower 和可选 Leader bearing 分量共同使用角误差滞环死区；
APF 顺序、车辆 PID 和 UDP 格式保持不变。
动捕失效期间有效控制时间冻结，恢复后的第一个周期使用标称 `dt`。

## 底盘控制器切换

全局开关位于 `swarm/domain/config.py`：

```python
GROUND_VEHICLE_CONTROLLER_TYPE = "projected"
```

允许值为 `differential`、`projected` 和 `mecanum`，分别构造原
`VehicleController`、连续车体系投影 `ProjectedVehicleController` 和
`MecanumVehicleController`。当前全局默认值为 `projected`；修改全局常量会
切换所有未单独覆盖 `GroundVehicleControlConfig.controller_type` 的实验。
最终选择值会随实验配置写入 metadata，非法值会在连接 ROS 和底盘前被拒绝。

三个控制器具有相同的 `step(...)`、`reset()` 和 `stop_command()` 接口。
原差速控制器使用目标朝向 PID 和速度 PID。车体系投影控制器把世界系
目标与实测速度投影为车体纵向和横向分量，并用目标向量模长缩放相对航向
误差：

```text
raw_forward    =  cos(yaw) * target_vx + sin(yaw) * target_vy
target_lateral = -sin(yaw) * target_vx + cos(yaw) * target_vy
target_norm    = sqrt(target_vx^2 + target_vy^2)
heading_error  = atan2(target_lateral, raw_forward)
target_forward = max(0, raw_forward)

speed_command = max(
    0,
    projected_speed_pid(target_forward, measured_forward),
)
yaw_rate = target_norm * heading_error / projected_lookahead_m

left_speed  = speed_command - yaw_rate * track_width_m / 2
right_speed = speed_command + yaw_rate * track_width_m / 2
```

`target_forward` 和纵向 PID 输出均限制为非负，因此车体不会以负纵向速度
倒车纠偏。目标位于车尾方向时纵向 PID 复位且直行速度为零，车辆按相对航向
误差原地转向；目标进入车体前半平面后才恢复前进。`yaw_rate` 使用
`projected_max_yaw_rate_rps` 限幅；任一轮超过物理速度上限时，左右轮、纵向
速度和角速度按相同比例缩放。由于角速度同时乘以 `target_norm`，世界系目标
向量趋零时直行速度和角速度均连续趋零。若实验用于验证无阶跃的渐近过程，
还应配置
`FormationControlConfig(bearing_deadband_enabled=False)`，否则 formation
层的角误差滞环仍会优先输出精确零。

终端每车行中的 `formation=(vx,vy,vz)`、`apf_target=(vx,vy,vz)` 和
`measured=(vx,vy,vz)` 均为世界坐标系速度。`speed=measured/target` 的两个
标量在 `projected` 模式下是车体坐标系纵向速度：前者为
`measured_forward`，后者为 `target_forward`，不是世界系平面速度模长。
`measured_forward` 可以为负，`target_forward` 始终非负。
`speed_cmd_mps` 是底盘纵向控制输出，`angle_cmd_rps` 是差速/投影控制器
用于左右轮差速的角速度命令；麦轮控制器当前令后者为零。终端不输出 APF
速度增量。

麦轮控制器接收世界坐标系平动目标 `(target_vx, target_vy, target_vz)`，根据
当前 `yaw` 把目标速度和实测速度转到车体系，并使用两套由
`mecanum_speed_pid` 构造的独立 PID 控制车体纵向与横向速度。当前接口没有
目标角速度，因此麦轮模式令目标 `wz=0`，`target_vz` 和 `measured_vz`
不参与平面底盘控制。

三个控制器的默认速度 PID 相互独立：

```python
speed_pid = PIDConfig(1.2, 0.0, 0.5, 1.0, 0.3)
mecanum_speed_pid = PIDConfig(1.5, 0.0, 0.0, 1.0, 0.3)
projected_speed_pid = PIDConfig(1.2, 0.5, 0.0, 1.0, 0.3)
```

`PIDConfig.integral_window_s` 设置标量 PID 的滑动积分窗口时长，默认 1 秒；
积分只包含最近窗口内的误差面积，并继续受 `integral_limit` 约束。

`speed_pid` 只供原差速控制器使用，麦轮纵向和横向 PID 只读取
`mecanum_speed_pid`，投影控制器的前进纵向环只读取
`projected_speed_pid`。三组参数都会写入实验 metadata，可分别调节而不影响
其他底盘控制器。

采用 X 型滚子和 UDP 协议轮序 `<FL,FR,RL,RR>` 时，零目标角速度下的轮缘
速度分配为：

```text
FL = vx_body - vy_body
FR = vx_body + vy_body
RL = vx_body + vy_body
RR = vx_body - vy_body
```

正轮速表示向车体前方滚动。四轮中任一轮超过
`GroundVehicleControlConfig.max_speed_mps` 时，四轮速度会按同一比例缩放，
随后映射到 `wheel_command_min` 至 `wheel_command_max`。该协议输出是底盘
归一化轮速指令，不是带轮半径换算的 `rad/s`。实车启用麦轮模式前应逐轮
核对下位机轮序、正方向和滚子安装方向。

## 避障

人工势场过滤器位于 `swarm/algorithms/obstacle_avoidance.py`。它接收 planner 输出的目标速度，叠加静态障碍物和其他有效车辆产生的二维排斥速度，再按 `RuntimeConfig.command_speed_limit_mps` 统一限幅。

启用静态障碍物示例：

```python
from swarm.domain.config import ObstacleAvoidanceConfig, StaticObstacleConfig

CONFIG = ExperimentConfig(
    algorithm="bearing",
    # 其余实验参数
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(
        StaticObstacleConfig("obs1", x=1.0, y=0.5, radius_m=0.15),
    ),
)
```

障碍物坐标使用动捕世界坐标系。当前人工势场实现不包含动态预测、安全状态机、切向力或局部极小值处理，不提供形式化无碰撞保证；实车测试应从低速和空旷场地开始。

## 新增编队算法

`swarm/application/planner_factory.py` 是算法选择入口。新增算法的一般步骤：

1. 在 `swarm/algorithms/` 中实现新的 planner，并满足 `SwarmPlanner.step(snapshot, dt)` 与 `reset()` 接口。
2. 如需专属参数，在 `swarm/domain/config.py` 增加类型化配置。
3. 在 `planner_factory.py` 中添加构造函数，并注册到 `_PLANNER_BUILDERS`。
4. 新增配置文件，例如 `configs/boids_six.py`，保持 `name="boids_six"`、`algorithm="boids"` 与文件名一致。
5. 增加测试，覆盖命令输出完整性、无效状态、限幅和内部状态重置。

## Subspace 矩阵权子空间控制

两车配置可运行 `python3 run_experiment.py --config subspace_two_aff`。
该配置使用 `car1 / 10.1.1.81` 和 `car4 / 10.1.1.84`，二者均为 peer；
约束 `y_car4-y_car1=0.5 m`，允许相对 x 运动，`k=0.5`，
包含相反 x 向任务速度，避障关闭。另有 `subspace_two_x`、
`subspace_two_y`、`subspace_two_xy`，分别允许沿 x、y、x=y 方向相对运动。
这四个模块的 `CONFIG.name` 均为 `subspace_two`，运行时必须使用实际模块名。
算法不使用 bearing PI、归一化控制向量或角度死区。

`ExperimentConfig.topology` 在 subspace 下使用 `SubspaceTopologyConfig(edges)`。
每条无向边只配置一次：`SubspaceEdgeConfig(source_id, target_id, ...)`。
对于一维自由子空间，直接给出非零 `free_direction` 向量即可；系统自动构造
固定且对称半正定的矩阵权。也可以显式提供 `matrix`，或使用兼容写法
`expected_points=(p_i^*, p_j^*)`，由两点差向量自动构造：

```text
d = p_j^* - p_i^*
A_ij = I - (d/||d||)(d/||d||)^T
```

例如 `free_direction=(0, 1)` 自动得到 `A=diag(1, 0)`，表示 y 方向相对运动自由、
x 方向受约束。`matrix`、`free_direction` 和 `expected_points` 至少提供一个；
优先级为显式 `matrix`、`free_direction`、`expected_points`。
偏置 `bias` 默认零，从 source 指向 target，反向反馈使用相同矩阵与相反偏置。
`planar_xy` 使用二维点和偏置，`spatial_xyz` 使用三维点和偏置；位置、偏置单位为米。

控制律与性能指标分开计算：

```text
r_ij = p_j - p_i
epsilon_ij = A_ij (r_ij - b_ij)
u_i = k sum_j epsilon_ij + Z_i w(t)

e_ij = || (A_ij / ||A_ij||_2) ((r_ij-b_ij) / ||r_ij||_2) ||_2
edge_rms = sqrt(sum_valid_edges(e_ij**2) / valid_edge_count)
```

矩阵范数为谱范数，向量范数为欧氏范数。控制反馈直接求和，不取邻居平均；
归一化仅用于性能指标。所有有效无向边等权统计一次。非零偏置下误差可大于 1，
不截断。零矩阵、相对距离不超过 `1e-9 m`、端点状态缺失或无效的边不参与 RMS；
无有效边时值为 `None`，终端和 CSV 显示空值。重合位置的边仍参与原始偏置反馈。

`SubspaceControlConfig.k` 默认 1.0，必须有限且为正；`modes` 默认 `None`。
推荐使用按模式分组的写法：每个外层元素是一个模式，内部按 `vehicle_ids`
顺序给出每辆车的二维或三维速度向量，形状为 `(模式数, 车辆数, 维度)`。
删除一个模式时只需删除一个外层元素；旧的 `(车辆数×维度, 模式数)` 扁平矩阵
写法仍兼容。
`coefficients` 可以是长度等于模式数的元组，其中每个元素是固定标量或接收有效
控制时间秒数并返回标量的模块顶层函数；也兼容一个返回完整系数元组的函数。
因此每个 mode 可以使用独立的分段函数，删除或替换一个 mode 时不需要修改其它
mode 的函数返回值。
默认空元组表示无任务输入。例如二维两车共同沿 x 方向以 0.05 m/s 平移：

```python
SubspaceControlConfig(
    k=1.0,
    modes=(
        (
            (1.0, 0.0),  # car3
            (1.0, 0.0),  # car6
        ),
    ),
    coefficients=(0.05,),
)
```

启动前检查偏置共同可行和 `L_A Z=0`，容差为 `1e-9`；不接受任意独立车辆参考，
`leader_target_velocities` 必须为空。任务函数返回值在每周期检查维度与有限性；
时间从零累计，默认动捕丢失停车期间冻结，`reset()` 清零。
实验可通过 `RuntimeConfig.duration_s` 设置自动停止时长；控制循环到达该时长后
发送所有车辆的停车指令并关闭遥测。
`swarm.domain.subspace.projection_from_free_basis(X)` 接受按列排列的独立自由基，
通过 QR 构造 `I-QQ^T`；二维空基写作 `((), ())`，相关基被拒绝。

控制器通过现有 planner 工厂和控制循环接入；反馈与任务速度相加后统一限制速度模长，
随后由现有 APF、底盘控制器和 UDP 链路处理。无效车辆输出无效零指令。
NumPy 用于配置矩阵校验及计算，配置数组使用元组以支持 metadata 序列化；
任务函数记录为 `module.qualname`。

性能输出使用 `PlannerResult.subspace_edge_rms`、`subspace_valid_edges` 与
`subspace_samples`；节点 `error_vector`、`mean_error/max_error` 仅保留原始残差诊断。
终端与实时轨迹标题显示边 RMS。记录期间新增 `subspace_edges.csv`（原始相对位置、
偏置、残差、矩阵谱范数、归一化误差及无效原因）和 `subspace_summary.csv`
（RMS、有效边数、总边数）；沿用遥测记录频率，bearing 实验不生成这些文件。
原有轨迹和 bearing CSV 格式兼容，subspace 不生成 bearing 边样本。

三车和七车配置分别通过 `--config subspace_triangle` 与
`--config subspace_seven` 选择。当前三车配置为仿射三角形约束，
七车配置为正方形及可伸缩下部；车辆映射、边偏置和任务时序见
[实验构型说明](../configs/README.md)。

## Subspace 任务闭环与编排

`swarm/domain/tasks.py` 定义不可变的任务配置；`swarm/algorithms/task_program.py`
保存每次实验独立的执行状态并求解模态速度。配置层通过
`SubspaceControlConfig(task_feedback=TaskFeedbackConfig(PROGRAM), coefficients=())`
启用闭环，`modes` 仍在构型配置中定义。未设置 `task_feedback` 时，原固定系数
及时间函数路径保持不变；两种输入不能混用。无需改动 ROS、底盘或运行入口。

基础 Goal 均使用世界坐标系和米：

| Goal | 必填目标参数 | 测量量 |
| --- | --- | --- |
| `NodePositionGoal` | `node_id, target` | 节点坐标；可用 `axes=(0,)` 只控制 x，target 长度随 axes 变化 |
| `RelativePositionGoal` | `source_id, target_id, target` | 从 source 到 target 的完整相对向量 |
| `EdgeLengthGoal` | `source_id, target_id, target` | 两节点欧氏距离，目标必须为正 |
| `ProjectedDistanceGoal` | `source_id, target_id, axis, target` | 相对向量沿归一化 axis 的有符号投影 |

每个 Goal 必须有非空 `id`。公共默认参数：`gain=0.5`、`weight=1.0`、
`tolerance=0.02 m`、`speed_tolerance=0.02 m/s`、`settle_s=1.0`、
`max_rate=0.2 m/s`、`timeout_s=60.0`（`None` 关闭超时），
`hold_after_completion=False`。完成要求位置/长度误差及动捕实测任务速度连续
满足容差；向量任务按二范数判定。投影间距可为负，欧氏距离不区分节点排列方向。
任务端点不必是拓扑边；能否达到目标取决于模态自由度。

编排节点支持嵌套：

- `Sequence((...))`：当前任务闭环完成后，下一控制周期启动后续任务。
- `Parallel((...))`：同时启动，所有子任务完成且组内保持目标仍满足稳定条件后结束。
- `Delay(seconds)`：按有效控制时间延迟启动；不新增保持目标，不阻塞其他并行任务。
- `Hold(seconds)`：捕获进入时的模态坐标 `Z†p` 并闭环驻留；连续静止且保持误差
  达标才累计驻留时间，扰动后重新计时。使用 `tolerance=0.02`、
  `speed_tolerance=0.02 m/s`（检查每辆车的实测速度）、`timeout_s=60.0`；
  保持误差的单位由配置模态的尺度决定。
  Hold 暂存已有保持目标，结束后恢复；不允许放在 Parallel 内。
- `Release(("goal_id", ...))`：释放此前保持的目标，不存在的 ID 不产生动作。

完成后设置保持的 Goal 继续进入统一求解；后续同 `id` 的 Goal 替换旧目标。
并行分支的 ID（包括 Release 引用）不能重复。释放的目标不再保证后续位置或长度。
整个程序结束后，保留目标继续闭环。配置对象不保存运行状态，planner `reset()`
会重新开始程序，各实验实例互不共享状态。

例如先平移，1 秒后叠加伸缩，全部完成后停留 2 秒：

```python
from swarm.domain.tasks import (
    Delay, Hold, NodePositionGoal, Parallel, ProjectedDistanceGoal, Sequence,
)

PROGRAM = Sequence((
    Parallel((
        NodePositionGoal(
            id="anchor", node_id="car3", target=(1.0, 0.5),
            hold_after_completion=True,
        ),
        Sequence((
            Delay(seconds=1.0),
            ProjectedDistanceGoal(
                id="wing", source_id="car5", target_id="car6",
                axis=(0.0, 1.0), target=0.6, hold_after_completion=True,
            ),
        )),
    )),
    Hold(seconds=2.0),
))
```

坐标为示例值。flyingwing 的完整示例位于 `task_programs/flyingwing_goals.py`；
在 `configs/flyingwing.py` 设置 `USE_TASK_PROGRAM=True` 选择它，默认 False
保留 `task_programs/flyingwing_extension.py` 的原时序演示。

对于任务误差 `e=q_target-f(p)`，令 `J=df/dp`、`A=JZ`，任务层求解：

```text
v_task = vector_limit(gain * e, max_rate)
w = argmin sum_goals weight * ||A w - (v_task - J u_formation)||²
           + regularization² * ||w||²
u = u_formation + Z w
```

`TaskFeedbackConfig` 默认 `mode_speed_limit=0.2`、
`mode_acceleration_limit=0.8`、`regularization=1e-6`。采用统一比例限制模态速度，
再统一插值限制相邻周期的系数变化；随后沿用原车辆向量限速和避障。
模态系数的物理尺度取决于 Z 的定义，不能等同于所有车辆的最终速度。
`weight` 表示误差折中，不是硬优先级。未指定的量不会自动保持；固定节点及
边长需求需显式添加 Goal。模态不足或目标冲突时不自动增加自由度，也不将静止
误判为完成，日志保留逐任务误差及分配残差。

目标超时后程序报告 failed 并驻留在当时的可控模态位置，不跳到下一任务；
重新开始需 reset 或重启实验。边长任务端点重合时方向导数未定义，暂停任务
输出并报告 waiting。动捕无效时暂停任务进度、清除连续稳定证据；原停车机制
不变，恢复后重新累计驻留/稳定时间。

终端增加任务状态、活动节点、w、分配残差和逐目标误差；仅闭环任务运行时
新增 `tasks.jsonl`，沿用记录频率，保存有效任务时间、目标稳定计时及失败原因。
`metadata.json` 的任务节点包含 `kind`，可区分 Sequence 和 Parallel。
任务层只做目标反馈与编排，不包含场地路径规划、矩形障碍扩展或自动包围策略。

理想收敛结论要求完整固定无向约束、共同可行偏置及一阶运动学。
自由模式须位于全局零空间；逐车限速、避障和真实底盘响应可能改变模式运动。
子空间共线约束不排除反向或重合，不单独提供碰撞保证。
