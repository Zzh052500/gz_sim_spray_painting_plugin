# UR5e 喷涂仿真 —— 算法说明

> 更新时间：2026-08-19
> 本文档解释项目各层算法的原理、关键公式与代码位置。
> 配套阅读：`USE_GUIDE.md`（操作手册）、`DEBUG_SUMMARY.md`（调试历史）。

---

## 目录

- [一、总体架构（四层）](#一总体架构四层)
- [二、轨迹执行算法](#二轨迹执行算法cartesian_path_executorpy)
- [三、正/逆运动学算法](#三正逆运动学算法generate_spray_posesp)
- [四、运动控制（第三方）](#四运动控制第三方)
- [五、喷涂模拟算法](#五喷涂模拟算法spraypaintpluginc)
- [六、"识别/感知"算法说明](#六识别感知算法说明)
- [七、性能数据](#七性能数据)

---

## 一、总体架构（四层）

```
 你自己的轨迹(笛卡尔点) ──[IK]──> 关节角YAML ──[executor]──> 连续轨迹
      ──[JTC样条插值]──> Gazebo机械臂动 ──[插件:锥形射线]──> 命中过滤
      ──[找link+去重]──> 薄圆盘补丁贴到link上 ──> 车身出现油漆
```

| 层 | 文件 | 职责 |
|----|------|------|
| 1. 轨迹执行 | `scripts/cartesian_path_executor.py` | 把关节角路点合成一条**连续不减速**的轨迹 |
| 2. 运动学 | `scripts/generate_spray_poses.py` | 笛卡尔目标 → 6关节角（数值IK） |
| 3. 运动控制 | `joint_trajectory_controller` + gz_ros2_control | 样条插值执行轨迹（ROS 2标准库） |
| 4. 喷涂模拟 | `src/gz_sim_spray_painting_plugin/src/SprayPaintPlugin.cc` | 射线采样 → 命中判定 → 油漆补丁沉积 |

> **设计关键**：轨迹层（关节角）与效果层（射线+补丁）**完全解耦**。
> 改轨迹形状只动第1、2层；改油漆效果只动第4层。

---

## 二、轨迹执行算法（`cartesian_path_executor.py`）

### 2.1 数据

- YAML 的 `joint_configs`：每个路点 = 6个关节角（机械臂实际执行的数据）。
- `poses` 段（笛卡尔TCP位置+姿态）仅供生成器参考，executor 不读。

### 2.2 分段时长（`_move_duration`，第67-73行）

```
speed = MAX_JOINT_SPEED × clamp(velocity_scaling, 0.05, 1.0)   # 1.0 × 0.1（launch默认）
Δt = max(1.5s, 相邻路点最大关节差 / speed)
```

- `MAX_JOINT_SPEED = 1.0 rad/s`（第64行）。
- `velocity_scaling`：launch 默认 `0.1`，executor 内部默认 `0.35`。

### 2.3 中间点速度 —— 中心差分（第144-148行）

```
v[i] = (q[i+1] - q[i-1]) / (t[i+1] - t[i-1])     # 首尾点为0
```

**目的**：JTC 默认会按时间插值、把速度端点设为零。若每个路点都归零，
机械臂会"走走停停"。中心差分让中间路点速度连续 → 8个路点合一条
**连续扫描**轨迹，边动边喷。

### 2.4 执行流程（`run`，第243-314行）

```
回home ──> 走到wp0(关喷) ──> 开喷 ──> 连续扫过所有路点 ──> 关喷(无条件) ──> 回home
```

- 开喷/关喷默认走 **ROS → ros_gz_bridge → 插件**（进程内 rclpy 发布，可靠），
  桥断了才回退 `gz topic` CLI（第168-242行）。
- `try/finally` 保证关喷**一定执行**，回home后再补一刀（第298-311行）。

---

## 三、正/逆运动学算法（`generate_spray_poses.py`）

### 3.1 正运动学（FK）

UR5e 改进DH参数（第47-49行）：

```
D  = [0.1625,  0.0,    0.0,     0.1333, 0.0997,  0.0996]
A  = [0.0,    -0.425, -0.3922,  0.0,    0.0,     0.0   ]
AL = [π/2,     0.0,    0.0,     π/2,   -π/2,     0.0   ]
```

每关节一个 4×4 齐次矩阵（`_dh`，第61-69行），6个连乘得末端位姿（`fk`，第72-77行）。

### 3.2 逆运动学（IK）—— 数值优化

代价函数（第80-85行）：

```
cost = ‖Δ位置‖² × 100  +  Σ(Δ旋转矩阵)²
```

- 优化器：`scipy.optimize.minimize` L-BFGS-B（第102行）。
- 关节限位：前两关节 ±2π，肘关节 ±π，腕部 ±2π。
- **收敛判据**：末端位置误差 < 2mm（第112行）。

### 3.3 连续性技巧

每个路点的求解**种子 = 上一个路点的解**（第225行 `q_seed = q_sol.copy()`），
保证相邻路点角度连续、不跳变。

### 3.4 轨迹形状

生成器当前只做**直线扫描**：x、z固定，y线性插值，姿态全程不变（第187-196行）。
要做任意形状（圆、多排Z字、斜线）只需改这里的 `waypoints` 列表，IK自动转换。

---

## 四、运动控制（第三方）

- `joint_trajectory_controller`（JTC）：对带速度的轨迹点做样条插值，输出关节位置/速度指令。
- `gz_ros2_control` 插件在 Gazebo 进程内托管 `controller_manager`，1000Hz 物理步进。
- executor 发布轨迹后按计算时长 `sleep(total+0.5)` 等控制器执行完。
- 配置：`config/ur_sim_controllers.yaml`（joint_state_broadcaster + JTC）。

---

## 五、喷涂模拟算法（`SprayPaintPlugin.cc`）—— 核心

每 `paint_interval_steps`（默认10步 ≈ 0.05s@1000Hz）执行一次扫描（STEP 6-9）。

### 5.1 锥形射线生成 —— Fibonacci 盘采样（`GenerateConeRays`，第192-208行）

喷头沿 +X 喷，`num_rays=16` 条射线分布在半锥角 `10°`、射程 `0.8m` 的锥内。

采样点在**单位圆盘**上用 **Fibonacci/sunflower 采样**（`UnitDiskSamples`，第155-172行）：
- 索引0 = 圆心（主轴射线）。
- 其余点：`r = √(i/(N-1))`，`θ = i × 黄金角`（≈2.3999 rad）。

> **为什么用sunflower**：面积密度均匀、无网格偏置，比均匀网格采样覆盖更好。

射线终点在 `x = coneMaxRange_` 的圆盘上，圆盘半径
`diskRadius = coneMaxRange × tan(halfAngle) × sampleDiskFraction_`（故意比锥截面小，
见 5.2）。

### 5.2 补丁尺寸求解 —— 圆盘覆盖问题（`ComputePatchSizing`，第242-277行）

要让16个补丁圆并起来**恰好盖满锥形投影**，需同时满足：
1. **内部无空洞**：补丁半径 ≥ 采样模式的**覆盖半径**（pattern上离最近采样点最远的距离）。
2. **边缘可控**：最外补丁到达 `rim_reach_` 倍锥半径处。

求解方式：在单位圆盘上做 **256×256 网格扫描**，逐点找最近采样点，测出覆盖半径（第247-266行）。
```
measured = 数值测得的覆盖半径
h  = max(measured × 1.02,  patch_overlap_factor_ / √N)        # N=16
sampleDiskFraction_  = rim_reach_ / (1 + h)
patchRadiusFraction_ = h × sampleDiskFraction_
```

默认参数（xacro）：`patch_overlap_factor=1.30`，`rim_reach=1.12`。

> **已知局限**（注释第232-240行）：有限个圆盘永远无法完全覆盖锥边缘，
> 边缘是**扇贝形**。rim_reach=1.12 时总失配（漏喷+溢出）约14%（N=16）。
> 想更好得改补丁表示（单个脚印形印章 / alpha蒙版贴花），不是加射线能解决的。

### 5.3 命中过滤（STEP 9，第788-822行）

```
丢弃  fraction ≤ 0（贴脸命中）或 ≥ 1（没打到东西，物理引擎哨兵值）
丢弃  法向量长度 < 0.5 的命中
轴向深度 axialDepth = fraction × coneMaxRange_
丢弃  axialDepth < 1mm
```

### 5.4 命中面识别 —— `FindHitLink`（第295行起）

"识别"命中的是哪个可喷涂部件，**两遍搜索**：
1. 主遍：用**碰撞体**的世界位姿中心（多link/偏移模型准确）。
2. 回退：用 link 原点（处理 PLANE 等没有 Collision 组件的几何）。
- 机械臂自身模型的 link 通过 `ownLinks_` 排除（不能往自己身上喷）。

### 5.5 补丁生成 —— `MakePatch`（第105-144行）

```
coneRadiusAtDepth = axialDepth × tan(coneHalfAngle)     # 深度处锥半径
radius = coneRadiusAtDepth × patchRadiusFraction_
radius = max(radius, min(kMinRadius=0.02, coneRadiusAtDepth))   # 近距离可见下限
补丁 = 薄圆柱：半径=radius，厚度=0.003m
```

朝向：绕 `z × normal` 的轴旋转 `acos(z·normal)`，把圆盘**平铺**到表面上（第127-139行）。
中心点沿法向抬 0.0015m（半厚度），避免与表面重叠（第125行）。

### 5.6 去重（第843-850行）

同一 link 上已有补丁中心距 < `patch_spacing`（默认0.02m）就跳过。
- 防止反复扫同一位置堆积实体（**实体创建是最大开销**）。
- 去重在 **link局部系** 做：工件动了也能正确去重（第826-832行）。

### 5.7 贴漆（第852-882行）

- 创建薄圆柱 **Visual** 实体，材质=油漆色，高光调暗30%（第780-786行做光泽感）。
- **parent 到命中的 link**（第876-878行）→ 工件移动时油漆跟着走。
- 不投影阴影（表面标记不是物体，第872行）。

### 5.8 粒子可视化（可选，STEP 3）

发射器在喷头处喷粒子做喷淋效果，纯视觉，不影响补丁沉积。

---

## 六、"识别/感知"算法说明

**重要澄清：本项目没有视觉/识别算法。**

- 没有相机、没有目标检测、没有图像处理、没有深度学习。
- 轨迹是**预先算好**的（`generate_spray_poses.py` 按已知工件位置生成关节角），
  不是"看到工件再决定往哪喷"。
- 插件不"认识"车，只做**几何判定**：射线打到哪、最近的可喷涂面是哪个 link。

项目中**最接近"识别"**的两处，都是几何而非视觉：

| 名称 | 位置 | 是什么 |
|------|------|--------|
| `FindHitLink` | `SprayPaintPlugin.cc:295` | 命中点的**最近可喷涂link查找**（两遍最近邻搜索） |
| 命中过滤 | `SprayPaintPlugin.cc:798` | 射线 hit 的**有效性判定**（fraction/法向量） |

**如果真要加"识别"**（比如相机识别工件、自主规划喷涂路径），方向是：
1. 在 Gazebo 里给 UR5e 加相机传感器（cam），
2. 图像经 ros_gz_bridge 转 ROS（`sensor_msgs/Image`），
3. 用传统CV（OpenCV 轮廓/ArUco）或深度学习（YOLO 等）识别工件位姿，
4. 把工件位姿喂给 `generate_spray_poses.py` 动态生成轨迹（替换目前写死的坐标）。
   这是另一个工程量级的扩展，当前项目刻意没做。

---

## 七、性能数据

喷涂扫描单次开销（`spray_perf_n10.csv` 的 `scan_us` 列）：
- 16射线 + 命中处理 + 找link + 去重，约 **100-200 微秒**/扫描。
- 实体创建（新增补丁）时会跳到 **13000+ 微秒**（首帧一次性），此后因去重几乎不再创建。
- 说明：补丁沉积的主要成本是**实体创建**，去重正是为了把它的频率压到最低。
