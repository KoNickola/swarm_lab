# 实验构型

[返回项目首页](../README.md) · [控制与任务参考](../docs/control_reference.md)

车辆映射、有效 Leader 和参数以各模块的 `CONFIG` 为准。图示采用动捕世界坐标系，
不代表车辆车体坐标系；配置中的实验参数覆盖公共默认值。

## 八车中间刚性、两侧柔性构型 `bearing_eight.py`

```text
              +y ↑
car1 ●────● car2 ────● car3 ────● car4
     │    │    ╲    │         │
car5 ●────● car6 ────● car7 ────● car8   → +x
```

中间 `car2、car3、car6、car7` 通过四条边和 `car2-car7` 对角线约束为
正方形。左侧 `car1、car5`、右侧 `car4、car8` 分别连接中间相邻顶点，
形成上下平行边；两侧没有对角线，宽度可以独立变化。所有示意边均双向
启用，共 11 条无向边、22 条有向边。`_BEARINGS[source][target]` 显式
定义方向，索引 0 至 7 对应 `car1` 至 `car8`。

采用 `bearing` 算法和默认 `bearing_only` 模式，边长不固定。
`car2` 为零速度 leader，其余为 follower；车辆地址沿用编号规则
`10.1.1.81` 至 `10.1.1.88`。控制增益与运行时使用公共默认值；避障总开关开启，
车间避障关闭。从 `swarm_lab/` 运行：

```bash
python3 run_experiment.py --config bearing_eight
```

所有 bearing 均定义在动捕世界坐标系中。图中的 `+x`、`+y` 是世界坐标轴，
不是车辆自身坐标轴。默认 `bearing_only` 模式只使用图中的相对方向和比例；
`bearing_based` 模式还把拓扑向量的模长作为期望相对距离。

配置文件通常采用 `<算法>_<名字或备注>.py` 命名。`--config` 选择模块名，
`CONFIG.name` 用于记录目录前缀。`flyingwing` 使用 `subspace` 算法；
四个 `subspace_two_*` 模块共用 `CONFIG.name="subspace_two"`。
运行时通过 `python3 run_experiment.py --config bearing_six` 选择配置。

## 七车飞翼 `flyingwing.py`

从 `swarm_lab/` 运行 `python3 run_experiment.py --config flyingwing`。
使用 `car1`、`car2`、`car3`、`car5`、`car6`、`car7`、`car8`，
地址按车辆编号映射为 `10.1.1.81`、`10.1.1.82`、`10.1.1.83`、
`10.1.1.85`、`10.1.1.86`、`10.1.1.87`、`10.1.1.88`，均为 peer。
采用 `subspace` 算法和 `planar_xy` 控制空间。

```text
    5                 7       +x →
    │                 │       +y ↓
    6     1────2      8
           ╲  ╱
            3
```

此图下方对应世界坐标 +y：`1→3=(0.5, √3/2)`，
`2→3=(-0.5, √3/2)`，分别与 +x 成 60°、120°；1–2 沿 x，
三条边构成等边三角形。1–6 和 2–8 沿 x，5–6 和 7–8 沿 y。
共 7 条无向边，每条只配置一次，双向反馈由现有算法生成。飞翼本体不含机头节点。

方向通过 `free_direction` 定义，`SIN_60=√3/2`，所有偏置默认为零。
沿边方向的相对位移自由，垂直于边的偏差受反馈约束；不固定三角形边长、
翼展或两侧翼长，也不强制左右对称。子空间约束不限定边的正负
方向，初始摆位应对应图示顺序。`k=1.0`；运行时、底盘、
遥测沿用公共默认参数。当前配置已开启避障总开关和车间避障，避障数值参数
沿用公共默认值。

`subspace_control.modes` 包含六个独立模态，顺序与 `coefficients` 一致：

| 序号 | 模态 | 正系数对应的运动 |
| --- | --- | --- |
| 1 | 整体平移 | 全部车辆沿 +x |
| 2 | 1–6 伸缩 | car5、car6 共同沿 −x |
| 3 | 2–8 伸缩 | car7、car8 共同沿 +x |
| 4 | 7–8 伸缩 | car7 沿 −y，car8 不受该模态驱动 |
| 5 | 5–6 伸缩 | car5 沿 −y，car6 不受该模态驱动 |
| 6 | 整体 y 平移 | 全部车辆沿 +y |

按图示摆位，伸缩系数为正表示伸长，为负表示收缩；平移系数为负表示沿对应负轴。
系数单位为 m/s，可分别填写常数或接收时间秒数的函数。平移系数为零，
16、28、56、78 使用独立时间函数，协调展示一次整体展开与收拢。各模态满足 `L_A Z=0`，
1–6 模态保持 5–6 相对位置，2–8 模态保持 7–8 相对位置，两个翼尖的竖直
伸缩不改变水平连接边。合成速度仍受公共速度上限约束。

时间为有效控制累计秒数，0–5 秒只执行构型反馈。16/28 左右翼同步运动，
56/78 两个翼尖同步滞后 0.5 秒，形成从翼根到翼尖的展开与收拢。
每次单向运动 2 秒，中间暂停 1 秒；每段首尾各 0.25 秒使用余弦速度渐变，
中间保持 ±0.2 m/s，使启动、停止和反向连续：

| 模态 | 伸长时段（秒） | 收缩时段（秒） |
| --- | --- | --- |
| 16 | [5, 7) | [8, 10) |
| 28 | [5, 7) | [8, 10) |
| 56 | [5.5, 7.5) | [8.5, 10.5) |
| 78 | [5.5, 7.5) | [8.5, 10.5) |

其余时段对应系数为零，10.5 秒后所有任务系数持续为零，构型反馈继续运行。
计入速度渐变后，每个关节的单程任务位移为 0.35 m，往返任务净位移为零；
实际位移受反馈、限速、避障和底盘跟踪影响。四个模态可同时运动，car5/car7
的水平与竖直任务速度叠加，纯任务速度模长最大约 0.283 m/s；反馈与避障
叠加后的总速度仍由公共速度上限约束。
`task_programs/flyingwing_extension.py` 中的 `EXTENSION_SPEED_MPS`、`EXTENSION_LEG_S`、`EXTENSION_PAUSE_S`
和 `DEMO_START_S` 分别控制速度幅值、单程时长、暂停时长和演示开始时刻；
`WINGTIP_DELAY_S` 控制翼尖滞后，`EXTENSION_RAMP_S` 控制每段首尾速度渐变时长。

### 闭环任务选择

`flyingwing.py` 的 `USE_TASK_PROGRAM=False` 保留上述时间演示。
先在 `task_programs/flyingwing_goals.py` 设置实测世界坐标系中的
`ANCHOR_TARGET`、目标翼展 `WING_SPAN_M` 和翼长 `WING_HEIGHT_M`，再将
`USE_TASK_PROGRAM=True` 切换至闭环程序；运行命令不变。文件内坐标与长度
是示例值，不表示已测量的场地目标。

示例首先使 car3 到达目标位置，延迟 1 秒后同时调整 16/28 的水平间距，
全部收敛后闭环驻留 2 秒，再同时调整 56/78 的竖直间距，最后驻留 2 秒。
各目标设置 `hold_after_completion=True`，因此 car3 位置与已完成的关节长度
在后续阶段仍参与闭环。目标时间由收敛决定，不使用原演示的 2 秒单向限制。

六个模态包括新增的整体 y 平移；其余五个模态与 7 车拓扑对应。任务层只能组合
已有模态：当前 56 模态只驱动 car5，不给 car6 额外竖直任务速度。
时间函数 `coefficients` 与 `task_feedback` 不能同时启用；闭环模式下
`coefficients=()`。通用 Goal 字段、编排语义和参数见
[控制与任务参考](../docs/control_reference.md)。任务层不提供路径规划或矩形障碍避让。

## Leader 运动配置

`leader_target_velocities` 的每个值可以是原有固定三维速度，也可以是
`TimeVelocityReference`、`TimePositionReference` 或 `XYCurveReference`。
各 leader 的函数与内部状态彼此独立。时间函数使用有效控制周期累计时间；
位置函数返回世界系绝对位置；静态曲线函数表示世界系 `y=f(x)`。

若需要把 leader 的运动参考与 bearing PI 分量叠加，在配置中设置：

```python
from swarm.domain.config import FormationControlConfig

formation_control=FormationControlConfig(leader_bearing_enabled=True)
```

两个分量相加后共同使用全局向量速度上限。未设置该开关的现有固定速度配置
保持原 leader 行为。完整函数配置示例和位置外环 PID 参数见
[控制参考](../docs/control_reference.md)。

PI bearing planner 的比例、积分增益分别为 `FormationControlConfig.k_p` 和
`k_i`；投影矩阵 bearing planner 的比例增益为独立的 `projection_k_p`。
修改其中一组增益不会改变另一种 planner 的控制增益。

两个 planner 均通过 `bearing_mode="bearing_only"` 或
`bearing_mode="bearing_based"` 选择向量处理方式。only 模式会归一化实际和
期望向量；based 模式不归一化，动捕相对位置和拓扑向量的距离尺度会进入控制
误差计算。投影矩阵 planner 在两种模式下都使用实际相对向量的单位方向构造
`I-g_ij g_ij^T`，only/based 不改变投影矩阵本身。现有配置使用默认的 only
模式。

## 单车 Leader 运动测试 `bearing_leader_motion.py`

该配置只包含 `car5` leader，并使用无边的 `1×1` topology。文件中的可调项
只包括输入模式、三类轨迹参数和位置环 PID 参数；运行时、底盘、formation、
避障和遥测参数统一使用 `swarm/domain/config.py` 中的公共默认值。当前公共
目标速度上限为 `0.30 m/s`，避障默认关闭。运行命令：

```bash
python3 run_experiment.py --config bearing_leader_motion
```

在配置文件顶部修改 `REFERENCE_MODE` 可测试三类输入：

- `time_velocity`：时间函数直接返回世界系目标速度，不使用位置环 PID。
- `time_position`：时间函数返回世界系绝对目标位置，经外环位置 PID 转换为
  速度。
- `xy_curve`：静态函数定义世界系 `y=f(x)`，切向速度与法向位置 PID 纠偏
  共同生成速度。

`POSITION_PID_KP`、`POSITION_PID_KI`、`POSITION_PID_KD` 和
`POSITION_PID_INTEGRAL_LIMIT` 是位置模式与静态曲线模式共用的外环参数。
使用 `time_position` 前，应把 `POSITION_ORIGIN_X_M/Y_M` 设为实测启动位置。
使用 `xy_curve` 前，应调整 `CURVE_Y_OFFSET_M`，使
`leader_xy_curve(x_start) ≈ y_start`。三类输入最终都经过同一个全局向量
速度上限，该上限不在此实验文件内重复配置。

## 避障配置

`ObstacleAvoidanceConfig.enabled` 控制人工势场过滤器，
`avoid_other_vehicles` 控制车间排斥；两个公共默认值均为 False。
当前 `bearing_three`、`bearing_six`、`bearing_eight` 开启总开关，
但未开启车间排斥。`flyingwing`、`subspace_six_triangles`、
`subspace_four_rectangle`、`subspace_four_square`、
`subspace_eight_square_sliders`、`subspace_pentagon_triangle` 同时开启两者；
`subspace_seven` 仅开启总开关。静态圆形障碍物由各配置的
`static_obstacles` 定义，坐标使用世界坐标系。

## 两车跟随 `bearing_two.py`

```text
car6 ●
     ↑ +y
     │
car3 ●────────→ +x
```

`car3 -> car6 = (0, 1, 0)`。car3 为静止 Leader，car6 为 Follower。

## 三车等腰直角三角形 `bearing_three.py`

```text
car7 ● (0, +d)
     │╲
  +y ↑ ╲
     │  ╲
car3 ●───● car6  → +x
    (0, 0) (+d, 0)
```

三条无向边均启用：

- `car3 -> car6 = (1, 0, 0)`
- `car3 -> car7 = (0, 1, 0)`
- `car6 -> car7 = (-1, 1, 0)`，默认 only 模式进入算法后归一化

反向边由矩阵中的相反向量表示。car3 是静止 Leader，car6 和 car7 是 Follower。

## 四车正方形 `bearing_four.py`

```text
      +y
       ↑
car2 ●────● car3  → +x
     │  ╲ │
car6 ●────● car7
```

有效边为 `car2-car3`、`car2-car6`、`car2-car7`、`car3-car7` 和 `car6-car7`。
车辆角色标签将 car2、car3 标为 leader，但当前 `leader_mask` 全为 False，
`leader_target_velocities` 为空，因此四辆车均执行编队反馈，不使用 Leader 运动参考。

## 六车柔性构型 `bearing_six.py`

```text
      +y
       ↑
car1 ●────● car2  → +x
     │  ╲ │
car3 ●────● car4
     │    │
car5 ●────● car6
```

实际有效边及方向以 `bearing_six.py` 中的拓扑配置为准。六车配置先创建
全零 `6×6` bearing 矩阵，再使用 `_BEARINGS[source][target] = (x, y, z)`
逐条定义有向边；索引 `0` 至 `5` 对应 `car1` 至 `car6`。未赋值的方向保持
为零，程序不会自动生成或修改反向边。邻接矩阵仍根据每个 bearing 是否为零
自动生成。

## 六车并排等边三角形 `subspace_six_triangles.py`

```text
       car3 ●                 ● car6
            ╱ ╲               ╱ ╲
           ╱   ╲             ╱   ╲
   car1 ●───────● car2  car4 ●───────● car5  → +x
```

`car1、car2、car3` 和 `car4、car5、car6` 分别组成两个朝向相同、并排放置的
等边三角形。每个三角形的三条边均为子空间直线约束，共 6 条无向边；两组三角形
之间没有连接边。每条边允许沿自身方向滑动，因此保持三角形形状但不固定边长；
两个三角形各自保留平移自由度。六辆车均为 peer，配置含有一个默认系数为零的
整体 +x 平移模态。车辆使用 `car1` 至 `car6` 对应地址，避障和车间避障开启。运行命令：

```bash
python3 run_experiment.py --config subspace_six_triangles
```

## 四车矩形 `subspace_four_rectangle.py`

```text
car1 ●────────● car4
     │        │
car7 ●────────● car6  → +x
     +y ↓
```

`car1`、`car4`、`car6`、`car7` 依次位于矩形左上、右上、右下、左下。四条边使用水平或
竖直自由方向约束，保持边方向关系并允许矩形宽、高变化；整体平移不受约束。
所有车辆均为 peer，已启用避障和车间避障，避障数值参数使用公共默认值。
四个任务模态依次控制世界系 x 平移、y 平移、沿 x 轴拉伸和沿 y 轴拉伸。拉伸以
矩形中心为中心对称进行，正值扩大对应边长，负值缩小。有效控制时间从 0 秒累计，
前 5 秒只执行构型反馈以供收敛；以下原分段整体后移 5 秒：

| 时间（秒） | 任务速度 |
| --- | --- |
| [0, 8) | 任务速度为零；前 5 秒用于构型收敛，随后保留原有 3 秒静止段 |
| [8, 11) | 沿 −x 平移 0.2 m/s；x、y 同时拉伸，0.2 m/s |
| [11, 13) | 沿 −x 平移 0.2 m/s；x、y 同时缩小，0.1 m/s |
| [13, 14) | 沿 +x、−y 平移各 0.2 m/s；x、y 同时缩小，0.1 m/s |
| [14, 17) | 沿 +x、−y 平移各 0.2 m/s；x、y 同时放大，0.3 m/s |
| [17, 18) | 沿 +x、−y 平移各 0.2 m/s |

各分段之外任务速度为零，构型反馈继续运行。运行命令：

```bash
python3 run_experiment.py --config subspace_four_rectangle
```

## 四车正方形 `subspace_four_square.py`

```text
car1 ●────────● car4
     │╲       │
     │  ╲     │
car7 ●────────● car6  → +x, +y ↓
```

使用与矩形配置相同的 `car1`、`car4`、`car6`、`car7` 及地址。四条边约束边的
方向，`car1-car6` 对角线约束宽高相等，因此保持正方形形状；边长和整体位置可变。
三个任务模态依次为中心收缩、世界系 x 平移和 y 平移。前 5 秒仅执行构型反馈，
随后先以 0.2 m/s 向中心收缩 3 秒，再沿 −x 以 0.2 m/s 平移 3 秒，最后沿 +x、+y
的 45°方向以 0.2 m/s 平移 3 秒。避障及车间避障开启，参数使用公共默认值。

```bash
python3 run_experiment.py --config subspace_four_square
```

## 八车正方形四滑臂 `subspace_eight_square_sliders.py`

```text
                 car5
                  ↑
        car1 ●────● car2 ────● car6 → +x
             │╲   │
        car8 ● car4●────● car3
                         │
                        car7
             +y ↑
```

车辆按 `car1` 至 `car8` 编号，使用 `10.1.1.81` 至 `10.1.1.88`，全部为 peer。
car1、car2、car3、car4 按左上、右上、右下、左下顺序构成正方形；四条边和
car1-car3 对角线约束正方形形状，边长与整体位置可变。car5、car6、car7、car8
分别位于 car1 的 +y、car2 的 +x、car3 的 −y、car4 的 −x 延长方向。每个滑臂
仅约束横向偏差，沿臂方向可独立运动。

六个任务模态按顺序为整体 +x 平移、整体 +y 平移、car5 沿 +y、car6 沿 +x、
car7 沿 −y、car8 沿 −x。初始系数均为零；在配置的
`subspace_control.coefficients` 中为六个模态分别指定速度（m/s）或时间函数。
避障与车间避障开启，其他参数使用公共默认值。运行命令：

```bash
python3 run_experiment.py --config subspace_eight_square_sliders
```

## 五边形与三角形 `subspace_pentagon_triangle.py`

```text
                    car4
                  ╱     ╲
             car5         car3
                ╲         ╱
                 car1────car2
                  │      │
             car7─┴──────car6
                   ╲    ╱
                    car8
```

`car1` 至 `car5` 构成正五边形，`car6` 至 `car8` 构成等边三角形；
car1-car7 和 car2-car6 是两图形之间的竖向约束边，约束其 x 向相对位置。
car6-car7 为水平边，car8 位于该边的 −y 侧；car7-car8 和 car8-car6 保留各自
配置的子空间方向。五边形通过两条对角线三角剖分，三角形使用三条边约束。
所有车使用 `10.1.1.81` 至 `10.1.1.88`，均为 peer。
任务模态依次为全体车辆的世界系 x/y 平移、五边形沿 −y 且三角形沿 +y 的
反向移动，以及五边形和三角形分别绕各自底边中点同步缩放。缩放模态使用边长
1 m 的名义构型，正系数扩张、负系数收缩，并满足现有边约束；该模态中
car1、car2、car6、car7 仅有 x 向速度。各模态系数由 `subspace_control.coefficients`
设置。避障和车间避障开启，其余参数使用公共默认值。运行命令：

```bash
python3 run_experiment.py --config subspace_pentagon_triangle
```


## 两车子空间配置

以下配置均使用 car1、car4，地址为 `10.1.1.81`、`10.1.1.84`；
二者均为 peer，`k=0.5`，避障关闭。

| `--config` 模块名 | 自由方向 | 偏置 | 受约束量 |
| --- | --- | --- | --- |
| `subspace_two_x` | `(1, 0)` | `(0, 0)` | 相对 y 为 0 |
| `subspace_two_y` | `(0, 1)` | `(0, 0)` | 相对 x 为 0 |
| `subspace_two_xy` | `(1, 1)` | `(0, 0)` | 相对位移沿 x=y |
| `subspace_two_aff` | `(1, 0)` | `(0, 0.5)` | `y_car4-y_car1=0.5 m` |

每个配置包含一个两车相向/反向运动模态。模态系数在 [0,3) 秒为 0、
[3,6) 秒为 0.2、[6,9) 秒为 −0.2，之后为 0；运动方向由各模态向量决定。
四个模块的 `CONFIG.name` 均为 `subspace_two`，记录目录使用这个前缀；
不存在 `configs.subspace_two` 模块。示例：

```bash
python3 run_experiment.py --config subspace_two_aff
```

矩阵构造、模态校验和归一化误差见 [控制参考](../docs/control_reference.md)。

## subspace_seven

运行 `python3 run_experiment.py --config subspace_seven`。I、J、K、L、M、N、O
依次对应 car1 至 car7。正方形边及 I-L 对角线使用 `free_direction + bias`
约束，保持方向和构型关系，不固定边长。K-M、L-N 为竖直自由边，M-N 为
水平自由边；J-O 保持 x 向偏置 0.3 m，允许相对 y 运动。

当前 `k=2`，共四个模态：整体 x 平移、M/N 共同 y 向伸缩、正方形尺度变化、
O 单独 y 向运动；后两个系数为 0。整体平移前 5 秒为 0，之后为 +0.2 m/s。
M/N 伸缩系数为：前 8 秒 0、[8,11) 秒 −0.2、[11,16) 秒 0、
[16,20) 秒 +0.2 m/s，之后为 0。`duration_s=25`，到时自动停车。
避障总开关开启，车间避障关闭。

## subspace_triangle

运行 `python3 run_experiment.py --config subspace_triangle`。使用 car3、car6、
car7，当前为三车仿射约束配置，实际边参数如下：

| 边（source → target） | `free_direction` | `bias`（m） |
| --- | --- | --- |
| car3 → car6 | `(0, 1)` | `(0.3, 0)` |
| car3 → car7 | `(1, 0)` | `(0, 0.4)` |
| car6 → car7 | `(-0.3, 0.4)` | `(0, 0)` |

边约束为 `A(r-b)=0`；偏置中位于自由方向的分量不产生约束。
当前 `k=5`，配置整体 x 平移模态，但系数为 0；避障关闭。
文件中的 `SIDE_M`、`HEIGHT_M` 不用于当前有效边参数。
