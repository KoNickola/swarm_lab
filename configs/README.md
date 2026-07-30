# 实验构型

所有 bearing 均定义在动捕世界坐标系中。图中的 `+x`、`+y` 是世界坐标轴，不是车辆自身坐标轴。构型图表示相对方向和比例，不规定绝对平移、旋转以外的自由尺度。

配置文件统一采用 `<算法>_<名字或备注>.py` 命名，文件名需与 `CONFIG.name` 一致，前缀需与 `CONFIG.algorithm` 一致。例如，`bearing_six.py` 中使用 `name="bearing_six"` 和 `algorithm="bearing"`。运行时通过 `python3 run_experiment.py --config bearing_six` 选择配置。

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
保持原 leader 行为。完整函数配置示例和位置外环 PID 参数见项目
`README.md`。

## 单车 Leader 运动测试 `bearing_leader_motion.py`

该配置只包含 `car3` leader，并使用无边的 `1×1` topology。文件中的可调项
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

原有四个多车实验配置只保留避障启用开关和静态障碍物列表。车辆半径、影响距离、排斥增益、最小计算距离和车间避障开关使用 `swarm/domain/config.py` 中的全局默认值。将实验中的 `enabled` 改为 `True` 即可启用基础人工势场，并可通过 `static_obstacles` 定义世界坐标系中的静态圆形障碍物。当前 `bearing_three.py` 和 `bearing_six.py` 已启用车间避障，但四个多车配置均未指定实际场地障碍物；障碍物中心和半径应按实验场地测量结果填写。

## 两车跟随 `bearing_two.py`

```text
car2 ●
     ↑ +y
     │
car1 ●────────→ +x
```

`car1 -> car2 = (0, 1, 0)`。

## 三车等腰直角三角形 `bearing_three.py`

```text
car6 ● (0, +d)
     │╲
  +y ↑ ╲
     │  ╲
car3 ●───● car5  → +x
    (0, 0) (+d, 0)
```

三条无向边均启用：

- `car3 -> car5 = (1, 0, 0)`
- `car3 -> car6 = (0, 1, 0)`
- `car5 -> car6 = (-1, 1, 0)`，进入算法后归一化

反向边由矩阵中的相反向量表示。`car3` 是静止 leader，`car5` 和 `car6` 是 follower。

## 四车正方形 `bearing_four.py`

```text
      +y
       ↑
car1 ●────● car2  → +x
     │  ╲ │
car3 ●────● car4
```

该配置取自六车构型的上部 `2×2` 子构型。`car1` 和 `car2` 是静止 leader，`car3` 和 `car4` 是 follower；有效边为 `car1-car2`、`car1-car3`、`car1-car4`、`car2-car4` 和 `car3-car4`。

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
