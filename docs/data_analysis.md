# 实验记录与离线分析

[返回项目首页](../README.md) · [控制与配置参考](control_reference.md)

所有命令从仓库根目录执行，`runs/<运行目录>` 需替换为实际记录目录。

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
- subspace 实验的 `analysis.png` 只包含归一化边误差 RMS 随时间变化曲线，
  纵轴为 `Normalized constraint error (RMS)`，范围固定为 0–1，无标题和图例，
  坐标轴名称和刻度字号为默认值的 3 倍，画布为 10×9 英寸，
  不绘制车辆轨迹或 bearing 面板。
- subspace 实验同时生成 `edge_error.png`：将 `subspace_edges.csv` 中所有
  无向边的 `normalized_error` 绘制在同一坐标轴，图例标识源车和目标车，
  无效样本显示为断点。纵轴范围为 0–1，时间范围沿用 `--max-time-s`。

subspace 两张误差图可分别生成随时间逐步展开的 GIF：

```bash
python3 tools/plot_error_animation.py runs/<运行目录> --fps 20
```

输出 `analysis.gif`（整体 RMS）和 `edge_error.gif`（逐边误差），读取与
后处理相同的 `subspace_summary.csv`、`subspace_edges.csv`，沿用静态图的
曲线样式、有效性规则及 0–1 纵轴。超出纵轴范围的值沿用静态图的裁剪显示。
两张动画从实验 `time_s=0` 开始，`--fps` 默认为 `20`，支持 `(0, 50]`。
曲线沿相邻有效样本的线段连续展开，逐边无效数据断点保留；提高帧率不改变
实验播放速度，也不增加测量数据。GIF 时间精度为 10 ms，按绝对时间取整
避免累计漂移；帧间隔至少 20 ms，末帧停留 20 ms 后循环播放。
播放一轮时长与实验末时间相差不超过 25 ms（播放器自身计时误差除外）。
`--output-directory` 指定输出目录，`--dpi` 默认 `160`；
`--max-time-s` 仅保留指定时刻及以前的数据，与后处理截取方式一致。

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

约束边 GIF（从 `trajectory.csv` 和 `metadata.json` 读取实际位置及实验拓扑）：

```bash
python3 tools/plot_constraint_animation.py runs/<运行目录> --fps 20
```

默认生成横向紧凑布局的 `trajectory_constraints.gif`，标题按构型或约束内容
生成，副标题为 `Measured robot positions`；节点以 `Robot N` 标注，时间保留
一位小数，坐标范围在整段动画中固定。
线性子空间约束使用实线连接节点；仿射约束 `A(r-b)=0` 在 `A b` 非零时，
绘制两条经过当前两节点、沿 `ker(A)` 方向的平行实线，不直接连接两节点。实线随节点移动，
表示自由运动方向，约束尚未满足时两线间距是当前实测间距。自由方向另用
双向箭头和 `Relative motion along x/y` 直接标注。两条平行线之间使用带方向
箭头的紫色虚线表示实测有符号偏移；画面右上角分别显示固定的
`Target x/y-offset` 与逐帧更新的 `Measured x/y-offset`。目标值取自实验配置，
实测值按 source 到 target 的目标法向计算，因此会保留正负号。
按拓扑中的无弦闭合环淡色填充内部区域；不补充缺失边，也不填充开放链。
带对角约束边的四边形按闭合三角形填色，重叠填色保持统一透明度。
支持二维 `subspace` 与 bearing 拓扑；仿射平行线表示要求自由空间为一维，
不支持三维记录或自由空间为零维的仿射约束。
`--output <文件.gif>` 指定输出，`--dpi` 默认 `180`，
画面不使用底部图例，约束含义由线旁标注和右上角数值说明。`--fps` 默认为
`20`，支持 `(0, 50]`，对相邻实测位置做线性插值；改变帧率
只影响画面更新频率，实验仍按原速播放。原 `plot_trajectory_animation.py`
也采用此规则。两个轨迹脚本的 GIF 与误差 GIF 共用时间轴和逐帧编码器，
相同数据起止时刻和帧率下保持同步；GIF 时间精度和末帧停留规则同上。
`--axis-max <数值>` 可将 x、y 坐标轴上限同时固定到指定值；例如 seven
实验使用 `--axis-max 2.5`。仿射自由方向的双向箭头及文字位于目标节点外侧
的方向延长线上，并按方向旋转，避免遮挡机器人节点。
若构型较密，可用 `--hide-free-direction-label` 隐藏自由方向文字，同时保留
双向箭头；seven 实验采用该选项避免文字遮挡节点。
`--obstacle-rectangle XMIN YMIN XMAX YMAX` 添加一个固定浅色障碍物方块，
该参数可重复传入；障碍物参与固定坐标范围计算，但不改变实验数据。
seven 实车画面中的单个障碍物来自原四角测量点的外接矩形，使用
`--obstacle-rectangle 0.534 -0.185 0.874 0.4407`。

## 普通轨迹动画与障碍物标注

```bash
python3 tools/plot_trajectory_animation.py runs/<运行目录> --fps 20
python3 tools/plot_constraint_animation_obstacle.py runs/<运行目录> --fps 20
```

普通轨迹动画默认尝试用 FFmpeg 输出 `trajectory_animation.mp4`；
编码失败时自动回退为同名 GIF。通过 `--output runs/<运行目录>/trajectory_animation.gif` 可生成 GIF。
`plot_constraint_animation_obstacle.py` 是带固定场地图形的展示脚本，
其图形用于离线标注，不参与实车控制或配置中的避障计算。
各工具的完整选项通过 `python3 tools/<脚本名>.py --help` 查看。
