# Swarm Lab

面向地面车群的分层编队控制实验项目。当前实现以 bearing-only 编队控制为核心，支持 ROS/VRPN 动捕状态输入、人工势场速度过滤、地面车 PID 执行器、UDP 轮速指令下发，以及控制过程中的终端输出、实时轨迹窗口和实验数据记录。

## 项目结构

```text
run_experiment.py                  # 实验入口
configs/                           # 可切换的实验配置
tools/postprocess.py               # 实验记录后处理
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
  -> SwarmPlanner，由 CONFIG.algorithm 选择
  -> VelocityCommandFilter，可选人工势场避障
  -> VehicleActuator，当前为地面车 PID + 差速轮速分配
  -> UDP Command Transport
  -> Console / Live Plot / CSV Recorder
```

`ControlLoop` 保持与平台无关：状态源、编队规划器、速度过滤器、执行器和遥测输出都通过接口注入。新增编队算法或执行平台时，应优先扩展接口实现，而不是改动控制循环。

## 当前能力

- 编队算法：`bearing`。
- 实验配置：两车、三车、四车、六车 bearing 编队。
- 默认配置：`configs.bearing_three`。
- 控制频率：默认 50 Hz。
- 目标速度上限：默认 `0.30 m/s`，且不得超过地面车物理速度上限。
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
    # 其余实验参数
)
```

当前配置：

| 配置 | 车辆 | leader | 说明 | 避障 |
| --- | --- | --- | --- | --- |
| `bearing_two` | `car1`, `car2` | `car1` | 两车 leader/follower 跟随 | 关闭 |
| `bearing_three` | `car3`, `car5`, `car6` | `car3` | 三车等腰直角三角形 | 开启 |
| `bearing_four` | `car1` - `car4` | `car1`, `car2` | 四车正方形子构型 | 关闭 |
| `bearing_six` | `car1` - `car6` | `car1`, `car2`, `car6` | 六车柔性构型 | 开启 |

所有 bearing 都定义在动捕世界坐标系中。Bearing-only 控制约束相对方向和构型比例，不单独指定绝对平移、旋转和尺度。构型示意可参考 `configs/README.md`，实际有效边与方向以各配置文件中的 `topology` 为准。

## 关键参数

核心配置定义在 `swarm/domain/config.py`：

- `ExperimentConfig.algorithm`：选择注册过的编队算法，当前支持 `bearing`。
- `FormationControlConfig`：bearing 编队控制增益、积分限幅和输出限幅。
- `ObstacleAvoidanceConfig`：避障启用开关、车辆半径、影响距离、排斥增益、最小计算距离和车辆间避障开关。
- `StaticObstacleConfig`：静态圆形障碍物的 ID、世界坐标中心和半径。
- `GroundVehicleControlConfig`：地面车轴距、物理速度上限、轮速指令范围、航向 PID 和速度 PID。
- `RuntimeConfig`：控制频率、动捕前缀、UDP 端口、UDP 超时、动捕丢失停车策略和速度上限。
- `TelemetryConfig`：终端、实时轨迹、记录频率、历史点数和记录目录。

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
- `summary.json`：记录数量、有效 bearing 样本数、平均/RMS/最大 bearing 角误差。

Bearing 角误差定义为有向边的三维夹角误差：

```text
theta_ij = atan2(||g_ij_desired x g_ij_actual||,
                 g_ij_desired . g_ij_actual)
```

零长度实际 bearing 没有定义，会记录为无效样本，并从均值、RMS 和最大值统计中排除。

后处理命令：

```bash
python3 tools/postprocess.py runs/<运行目录>
```

生成产物：

- `analysis.json`：每辆车路径长度、速度 RMSE、发送成功率，以及逐边和全局 bearing 角误差统计。
- `analysis.png`：二维轨迹、逐边 bearing 角误差和速度跟踪曲线。

## 测试

测试不依赖 ROS 或真实车辆：

```bash
python3 -m unittest discover -s tests -v
```

建议在改动编队算法、避障策略、配置校验、动捕丢失行为、通信重试或控制公式后运行完整测试。
