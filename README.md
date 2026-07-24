# bearing_swarm_refactored

## 项目概览

这是原 `bearing_swarm` 的分层重构版本，现位于 `semi-rigid/bearing_swarm_refactored`，与旧项目并列且独立运行。通用运行时代码使用 `swarm` 包名，可承载 bearing、boids 等多种算法。当前 bearing-only 编队误差计算、地面车速度/航向 PID、差速轮速分配和 UDP 报文格式保持原有逻辑；formation 与 APF 使用统一的全局目标速度上限。实验配置通过显式的 `algorithm` 字段选择编队算法。

架构包含三类扩展点：

- `SwarmPlanner`：bearing、boids 等算法实现相同的 `step(snapshot, dt)` 和 `reset()` 接口。
- `VelocityCommandFilter`：在编队规划后、平台执行前修正目标速度；当前实现为可选的基础人工势场避障。
- `VehicleActuator`：后续无人机执行器将速度命令转换为飞控命令，不影响编队算法和控制循环。

## 架构与运行流程

```text
ROS/VRPN StateSource
        ↓
    ControlLoop
        ↓
 SwarmPlanner（由 CONFIG.algorithm 选择）
        ↓
 VelocityCommandFilter（可选 APF）
        ↓
 VehicleActuator（当前为地面车 PID + 差速控制）
        ↓
 UDP Command Transport
        ↓
 Console / Live Plot / CSV Recorder
```

主要目录：

- `swarm/domain`：跨层数据模型和类型化配置。
- `swarm/algorithms`：PID、地面车控制和各类编队算法。
- `swarm/application`：planner factory、接口与确定性的控制循环编排。
- `swarm/infrastructure`：ROS、UDP、地面车执行器和遥测适配器。
- `configs`：互相独立的实验配置。
- `tools/postprocess.py`：实验后处理。
- `tests`：算法特征测试和运行流程测试。

每个有效控制周期严格按以下顺序执行：获取稳定动捕快照、检查状态、计算真实 `dt`、编队规划、速度过滤、平台控制、UDP 下发、遥测、补足控制周期。退出时始终发送停车命令并关闭遥测与 ROS 订阅。

## 当前实验配置

实验配置统一采用 `<算法>_<名字或备注>.py` 命名，模块文件名、`CONFIG.name` 和 `CONFIG.algorithm` 前缀保持一致。当前配置为：

- `configs/bearing_two.py`：两车 leader/follower 跟随。
- `configs/bearing_three.py`：三车等腰直角三角形，当前车辆为 `car3`、`car5`、`car6`。
- `configs/bearing_four.py`：双 leader 的四车正方形。
- `configs/bearing_six.py`：六车柔性构型。

命令行默认使用 `configs/bearing_three.py`。各构型及世界坐标轴示意图见 `configs/README.md`。切换同一算法的实验条件只需修改 `--config`，无需修改算法代码。

## 关键参数

参数集中在 `swarm/domain/config.py`，实验条件位于 `configs/`：

- `ExperimentConfig.algorithm`：显式选择注册的编队算法，当前可用值为 `bearing`。
- `FormationControlConfig`：`k_p`、`k_i`、积分限幅和单分量输出限幅。
- `ObstacleAvoidanceConfig`：启用开关、车辆半径、障碍影响距离、排斥增益、最小计算距离和车间避障开关。
- `StaticObstacleConfig`：静态圆形障碍物的编号、世界坐标中心和半径。
- `GroundVehicleControlConfig`：航向/速度 PID、轨距、物理速度和轮速指令范围。
- `RuntimeConfig`：统一目标速度上限、控制频率、动捕前缀、UDP 端口和动捕未准备时停车行为。
- `TelemetryConfig`：终端、实时轨迹、记录频率和历史点数。

默认终端输出和轨迹刷新均为 10 Hz，控制仍运行在 50 Hz。

## 编队算法选择与扩展

`swarm/application/planner_factory.py` 是算法选择入口。它根据 `CONFIG.algorithm` 查找构造函数；未注册名称会在 ROS、UDP 等外部资源启动前给出可用算法列表。当前配置显式写为：

```python
CONFIG = ExperimentConfig(
    name="bearing_six",
    algorithm="bearing",
    # 其余实验参数
)
```

新增 boids 或其他算法时：

1. 在 `swarm/algorithms/` 新增实现，并实现 `SwarmPlanner` 的 `step(snapshot, dt)` 与 `reset()`。
2. 在 `swarm/domain/config.py` 新增该算法的类型化参数；如果它不使用 bearing 拓扑，同时将算法专属字段和校验从通用配置中拆出。
3. 在 `swarm/application/planner_factory.py` 增加构造函数，并把名称注册到 `_PLANNER_BUILDERS`。
4. 新建如 `configs/boids_six.py` 的实验配置，保持 `name="boids_six"`、`algorithm="boids"` 与文件名一致。
5. 为 planner 输出补充契约测试，确保每个车辆都有 `VelocityCommand`，并覆盖无效状态、速度限幅和内部状态重置。

`ControlLoop`、APF 速度过滤器和地面车执行器均通过接口接收 planner 输出，新增编队算法时不需要修改这些组件。

## 基础人工势场避障

人工势场过滤器位于 `swarm/algorithms/obstacle_avoidance.py`。现有 bearing-only 输出作为目标速度，过滤器叠加静态圆形障碍物和其他有效车辆产生的二维排斥速度，再对平面速度统一限幅。实现不包含动态预测、安全状态机、切向力或局部极小值处理。

终端车辆状态同时显示 APF 前的 `formation` 速度、APF 后的 `apf_target` 速度以及两者之差 `apf_delta`，用于观察势场是否生效并调整排斥参数。

避障默认关闭。formation 和 APF 均使用 `RuntimeConfig.command_speed_limit_mps`，默认值为 `0.30 m/s`，且该值不能超过地面车物理速度上限。实验配置可按以下方式启用并定义障碍物：

```python
from swarm.domain.config import ObstacleAvoidanceConfig, StaticObstacleConfig

CONFIG = ExperimentConfig(
    algorithm="bearing",
    # 其余实验参数保持不变
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(
        StaticObstacleConfig("obs1", x=1.0, y=0.5, radius_m=0.15),
    ),
)
```

所有障碍物坐标均使用动捕世界坐标系。基础人工势场可能出现局部极小值，且不提供形式化的无碰撞保证；实车测试应从低速开始。

## 运行

需要 ROS1、`rospy`、`geometry_msgs`、VRPN 动捕话题和 Matplotlib：

```bash
cd bearing_swarm_refactored
python3 run_experiment.py
```

常用选项：

```bash
python3 run_experiment.py --config bearing_six
python3 run_experiment.py --config bearing_four
python3 run_experiment.py --config bearing_three
python3 run_experiment.py --no-plot
python3 run_experiment.py --no-record
```

## 实时轨迹

轨迹 GUI 使用非 daemon 独立进程，控制进程通过有界非阻塞队列发送位置。绘图进程启动后会明确输出所用 Matplotlib backend；GUI 初始化失败时会打印具体错误，控制循环仍可继续运行，不再静默无图。

Linux/WSL 必须存在图形显示环境和交互式 Matplotlib backend。若提示 backend 为 `agg`，应安装 Tk 支持并在有 DISPLAY/WSLg 的终端运行，例如：

```bash
sudo apt install python3-tk
MPLBACKEND=TkAgg python3 run_experiment.py
```

## 实验数据与后处理

默认仅以 5 Hz 记录分析所需字段，而不是保存全部 50 Hz 控制数据。每次运行在 `runs/<实验名_时间>/` 生成：

- `metadata.json`：实验配置和参数。
- `trajectory.csv`：位置、速度、bearing 误差、控制量、四轮原始指令和发送状态。
- `summary.json`：记录数量和总体 bearing 误差。

后处理命令：

```bash
python3 tools/postprocess.py runs/<运行目录>
```

生成：

- `analysis.json`：每辆车的路径长度、速度 RMSE、bearing 误差和发送成功率。
- `analysis.png`：二维轨迹、bearing 误差和速度跟踪曲线。

## 测试

测试不依赖 ROS 或真实车辆：

```bash
python3 -m unittest discover -s tests -v
```

任何安全策略、动捕超时、通信重试或控制公式调整都应作为独立行为变更处理，不能混入架构重构。
