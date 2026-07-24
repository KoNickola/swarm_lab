# 实验构型

所有 bearing 均定义在动捕世界坐标系中。图中的 `+x`、`+y` 是世界坐标轴，不是车辆自身坐标轴。构型图表示相对方向和比例，不规定绝对平移、旋转以外的自由尺度。

配置文件统一采用 `<算法>_<名字或备注>.py` 命名，文件名需与 `CONFIG.name` 一致，前缀需与 `CONFIG.algorithm` 一致。例如，`bearing_six.py` 中使用 `name="bearing_six"` 和 `algorithm="bearing"`。运行时通过 `python3 run_experiment.py --config bearing_six` 选择配置。

## 避障配置

四个实验配置只保留避障启用开关和静态障碍物列表。车辆半径、影响距离、排斥增益、最小计算距离和车间避障开关使用 `swarm/domain/config.py` 中的全局默认值。将实验中的 `enabled` 改为 `True` 即可启用基础人工势场，并可通过 `static_obstacles` 定义世界坐标系中的静态圆形障碍物。当前 `bearing_three.py` 和 `bearing_six.py` 已启用车间避障，但四个配置均未指定实际场地障碍物；障碍物中心和半径应按实验场地测量结果填写。

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

实际有效边及方向以 `bearing_six.py` 中的拓扑矩阵为准。
