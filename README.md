# Swarm Lab

面向地面车群的分层编队控制实验项目。当前实现以 bearing-only 编队控制为核心，支持 ROS/VRPN 动捕状态输入、人工势场速度过滤、可切换差速/麦克纳姆轮地面车控制器、UDP 轮速指令下发，以及控制过程中的终端输出、实时轨迹窗口和实验数据记录。

## 项目结构

```text
run_experiment.py                  # 实验入口
configs/                           # 可切换的实验配置
tools/postprocess.py               # 实验记录后处理
tools/plot_speed_tracking.py       # 每辆车独立速度与航向跟踪图
tests/                             # 不依赖 ROS 和实车的单元测试
swarm/
  domain/                          # 数据模型与类型化配置
  algorithms/                      # 编队、车辆控制、避障算法
  application/                     # 控制循环、接口和 planner 工厂
  infrastructure/                  # ROS、UDP、遥测和地面车适配器
```

一次控制周期的执行顺序为：

```text
ROS/VRPN 动捕状态
  -> ControlLoop
  -> ControlSpace 投影，地面编队截断 z/vz，无人机保留 xyz/vxyz
  -> SwarmPlanner，由 CONFIG.algorithm 选择
  -> VelocityCommandFilter，可选人工势场避障
  -> VehicleActuator，按全局参数选择差速或麦轮速度控制与轮速分配
  -> UDP Command Transport
  -> Console / Live Plot / CSV Recorder
```

`ControlLoop` 保持与平台无关：状态源、编队规划器、速度过滤器、执行器和遥测输出都通过接口注入。新增编队算法或执行平台时，应优先扩展接口实现，而不是改动控制循环。

## 当前能力

- 编队算法：`bearing`。
- 实验配置：单 leader 运动测试，以及两车、三车、四车、六车 bearing 编队；六车配置使用按索引赋值的有向 bearing 矩阵，不自动生成反向边。
- 默认配置：`configs.bearing_three`。
- 控制频率：默认 50 Hz。
- 目标速度上限：默认 `0.30 m/s`，且不得超过地面车物理速度上限。
- 底盘控制：全局选择原差速或麦克纳姆轮控制器，默认保持原差速实现。
- Leader 参考：固定速度、时间速度函数、时间位置函数和静态世界系
  `y=f(x)` 曲线。
- 遥测：终端输出 10 Hz，实时轨迹窗口 10 Hz，CSV 记录 5 Hz。
- 避障：基础二维人工势场过滤器，可叠加静态圆形障碍物和其他车辆的排斥速度。

## 环境要求

运行实车实验需要：

- Python 3
- ROS1
- `rospy`
- `geometry_msgs`
- VRPN 动捕话题，默认前缀为 `/vrpn_client_node`
- Matplotlib，项目 `requirements.txt` 中列出

安装 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
```

如果需要实时轨迹窗口，Linux/WSL 环境还需要可用的图形显示和交互式 Matplotlib backend。若提示 backend 为 `agg`，可安装 Tk 支持后指定 backend：

```bash
sudo apt install python3-tk
MPLBACKEND=TkAgg python3 run_experiment.py
```

## 运行实验

默认运行三车配置：

```bash
python3 run_experiment.py
```

切换配置：

```bash
python3 run_experiment.py --config bearing_two
python3 run_experiment.py --config bearing_three
python3 run_experiment.py --config bearing_four
python3 run_experiment.py --config bearing_six
python3 run_experiment.py --config bearing_leader_motion
```

关闭部分遥测输出：

```bash
python3 run_experiment.py --no-plot
python3 run_experiment.py --no-record
```

也可以传入完整模块名：

```bash
python3 run_experiment.py --config configs.bearing_six
```

程序启动后会先验证配置。若 `CONFIG.algorithm` 不受支持、车辆列表与拓扑维度不一致、leader 缺少目标速度、速度上限不合法或障碍物配置无效，会在连接外部资源前报错退出。

## 实验配置

配置文件位于 `configs/`，统一导出 `CONFIG: ExperimentConfig`。文件名、`CONFIG.name` 和 `CONFIG.algorithm` 前缀应保持一致，例如：

```python
CONFIG = ExperimentConfig(
    name="bearing_six",
    algorithm="bearing",
    control_space="planar_xy",
    # 其余实验参数
)
```

当前配置：

| 配置 | 车辆 | leader | 说明 | 避障 |
| --- | --- | --- | --- | --- |
| `bearing_leader_motion` | `car5` | `car5` | 单车三类 leader 参考与位置 PID 调试 | 关闭 |
| `bearing_two` | `car1`, `car2` | `car1` | 两车 leader/follower 跟随 | 关闭 |
| `bearing_three` | `car3`, `car5`, `car6` | `car3` | 三车等腰直角三角形 | 开启 |
| `bearing_four` | `car1` - `car4` | `car1`, `car2` | 四车正方形子构型 | 关闭 |
| `bearing_six` | `car1` - `car6` | `car1`, `car2`, `car6` | 六车柔性构型 | 开启 |

所有 bearing 都定义在动捕世界坐标系中。Bearing-only 控制约束相对方向和构型比例，不单独指定绝对平移、旋转和尺度。构型示意可参考 `configs/README.md`，实际有效边与方向以各配置文件中的 `topology` 为准。

### Bearing 角误差滞环死区

Bearing PI 提供逐车角误差施密特死区，当前公共默认值为关闭。启用后，控制器
使用该车所有有效出边的
最大 bearing 角误差：`planar_xy` 使用投影后的二维向量，
`spatial_xyz` 使用完整三维向量。误差降至
`FormationControlConfig.bearing_deadband_enter_deg=1.5°` 时进入稳定状态，
误差达到 `bearing_deadband_exit_deg=3°` 时恢复控制；两个阈值之间保持原
状态。稳定状态下 bearing 修正精确为零并复位对应 PI，避免接近目标构型时
微小速度向量变号导致目标朝向跳变。Leader 的预设运动参考不受影响，只抑制
其可选的 bearing 修正分量。

`bearing_deadband_enabled=True` 可启用该机制。参数会随实验配置写入
`metadata.json`，并要求满足
`0 <= bearing_deadband_enter_deg < bearing_deadband_exit_deg <= 180`。

### Leader 运动参考

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

## 关键参数

核心配置定义在 `swarm/domain/config.py`：

- `ExperimentConfig.algorithm`：选择注册过的编队算法，当前支持 `bearing`。
- `ExperimentConfig.control_space`：控制状态空间。`planar_xy` 在动捕快照
  进入 planner 前将 `z/vz` 截断为零；`spatial_xyz` 保留完整三维数据。
- `ExperimentConfig.leader_target_velocities`：每个 leader 的固定速度或运动
  参考。
- `ExperimentConfig.leader_position_pid`：时间位置函数和静态曲线共用的
  外环位置 PID；最终速度由全局向量上限统一约束。
- `FormationControlConfig`：bearing 编队控制增益、滑动积分窗口时长
  `integral_window_s`（默认 1 秒）、积分限幅、输出限幅、
  角误差滞环死区和 leader bearing 分量开关。
- `ObstacleAvoidanceConfig`：避障启用开关、车辆半径、影响距离、排斥增益、最小计算距离和车辆间避障开关。
- `StaticObstacleConfig`：静态圆形障碍物的 ID、世界坐标中心和半径。
- `GroundVehicleControlConfig`：底盘控制器类型、轨距、物理速度上限、轮速
  指令范围，以及原差速、车体系投影和麦轮控制器的独立参数。
- `RuntimeConfig`：控制频率、动捕前缀、UDP 端口、UDP 超时、动捕丢失停车策略和速度上限。
- `TelemetryConfig`：终端、实时轨迹、记录频率、历史点数和记录目录。

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
原差速控制器保持目标朝向加速度 PID 的既有流程。车体系投影控制器把世界系
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
`measured_forward` 可以为负，`target_forward` 始终非负。终端不输出 APF
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
projected_speed_pid = PIDConfig(1.0, 0.0, 0.0, 1.0, 0.3)
```

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

## 实验数据

默认每次记录会在 `runs/<实验名>_<时间戳>/` 下生成：

- `metadata.json`：实验配置和参数快照。
- `trajectory.csv`：车辆位置、速度、目标速度、执行器控制量、轮速指令和发送状态。
- `bearing_edges.csv`：每条有向 bearing 边的期望方向、实际方向、有效性和角误差。

Bearing 角误差定义为有向边的三维夹角误差：

```text
theta_ij = atan2(||g_ij_desired x g_ij_actual||,
                 g_ij_desired . g_ij_actual)
```

零长度实际 bearing 没有定义，会记录为无效样本，并从最大值和稳态值计算
中排除。

后处理命令：

```bash
python3 tools/postprocess.py runs/<运行目录>
```

默认使用每个误差序列最后 `20%` 的观测帧估计稳态误差。估计值为尾段内
有效误差的算术平均，稳态容差带默认为估计值的 `±5%`。从实验末帧向前
搜索连续处于容差带内的记录样本，该终端连续区域定义为稳态区间，其起点
即 `convergence_time_s`。缺失或无效样本会中断连续区域；若最后一帧无效
或超出容差带，则该序列不报告稳态区间。

`convergence_time_s` 是相对于本次记录首帧的收敛时刻，不是判稳计算耗时。
`duration_s` 等于实验末帧时间减去 `convergence_time_s`，表示最终连续稳态
区间覆盖的时间长度；若只有末帧落在容差带内，则样本数为 `1` 且
`duration_s=0`。

尾段比例和相对容差可通过命令行调整：

```bash
python3 tools/postprocess.py runs/<运行目录> \
  --steady-tail-percent 20 \
  --steady-tolerance-percent 5
```

逐边误差独立使用上述方法。全局误差只在全部观测有向边均存在且有效的
完整帧上计算：

```text
global_error = sqrt(edge_1_error^2 + ... + edge_n_error^2)
```

该二范数不除以边数，然后作为全局标量误差使用同一尾段估计和稳态区间
搜索方法。

生成产物：

- `analysis.json`（bearing 数值统一使用 `deg`）：
  - 每辆车的样本数、时长、二维路径长度、最大速度误差和发送成功率；
  - 全局二范数和每条有向边的全程最大误差；
  - 尾段稳态估计、收敛时间、稳态区间时长和样本数；
  - 稳态区间内的平均误差和 RMS；
  - 期望边数、观测帧数、完整帧数、完整帧率和最大观测间隔。
  JSON 中 `bearing_global` 位于 `bearing_edges` 之前。
- `analysis.png`：二维轨迹、全局二范数误差，以及按有向边源车辆拆分的
  bearing 误差子图。每辆具有出边的车辆对应一张子图，图内包含该车所有
  `source->target` 边；全局图显示稳态估计、容差带和最终稳态区间，不再
  绘制速度跟踪曲线。

全流程 bearing 统计只保留最大值；mean 和 RMS 仅用于最终稳态区间。
后处理不再生成 P95、固定末尾窗口或旧的固定阈值保持时间指标。

逐车生成独立速度和航向跟踪图：

```bash
python3 tools/plot_speed_tracking.py runs/<运行目录>
```

默认分别在 `runs/<运行目录>/speed_tracking/` 和
`runs/<运行目录>/angle_tracking/` 下生成 `car1.png`、`car2.png` 等文件。
速度图绘制对应车辆的 `target_speed_mps` 和 `measured_speed_mps`；航向图
将 `target_heading_rad` 和动捕 `yaw_rad` 直接换算为度，不展开 `-π` 到
`π` 边界处的跳变。不同车辆不会画在同一个 Figure 中，速度与航向也使用
不同 Figure。可通过
`--output-directory <目录>` 指定速度图目录，
`--angle-output-directory <目录>` 指定航向图目录，通过 `--dpi <整数>`
调整图片分辨率。

## 测试

测试不依赖 ROS 或真实车辆：

```bash
python3 -m unittest discover -s tests -v
```

建议在改动编队算法、避障策略、配置校验、动捕丢失行为、通信重试或控制公式后运行完整测试。
