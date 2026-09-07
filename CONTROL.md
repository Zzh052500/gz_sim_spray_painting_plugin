# UR5e 喷涂仿真 —— 控制逻辑分析

> 更新时间：2026-08-19
> 本文档解释整套系统的**控制逻辑**：启动时序、三大控制回路、
> 控制器管理、双时钟问题。
> 配套阅读：`ALGORITHM.md`（算法原理）、`USE_GUIDE.md`（操作手册）。

---

## 目录

- [一、总控链路全景图](#一总控链路全景图)
- [二、启动时序（launch 调度）](#二启动时序launch-调度)
- [三、控制回路1：轨迹下发（executor → 机械臂）](#三控制回路1轨迹下发executor--机械臂)
- [四、控制回路2：喷头触发（ROS→桥→插件）](#四控制回路2喷头触发ros桥插件)
- [五、控制回路3：状态反馈](#五控制回路3状态反馈)
- [六、两个 controller_manager 的坑](#六两个-controller_manager-的坑)
- [七、双时钟问题](#七双时钟问题)
- [八、常见"控制"类问题速查](#八常见控制类问题速查)

---

## 一、总控链路全景图

```
                    ┌────────────────────────────┐
                    │  ur_spray_demo.launch.py   │  ← 调度大脑（TimerAction错峰）
                    └───────┬────────┬───────────┘
            ┌───────────────┘        └───────────┐
            ▼                                    ▼
   ┌─────────────────┐                   ┌─────────────────┐
   │ gz sim server   │                   │ ros_gz_bridge   │
   │ (物理引擎1000Hz)│                   │ (ROS↔GZ翻译官)  │
   │  内置:          │                   └──┬──────┬───────┘
   │  • 喷涂插件      │                      │      │
   │  • gz_ros2_     │                      │      │
   │    control的    │              /clock  │      │ /spray_paint/trigger
   │    controller_  │              GZ→ROS  │      │ ROS→GZ
   │    manager      │                      │      │
   └────────┬────────┘                      ▼      ▼
            │                         ┌─────────────────┐
            │  /joint_states          │ robot_state_pub │→ TF
            ▼                         │ joint_state_pub │→ 初始姿态
   ┌─────────────────┐                └─────────────────┘
   │ JSB(广播器)     │
   └────────┬────────┘
            │
            ▼
   ┌─────────────────┐     /joint_trajectory    ┌─────────────────────┐
   │ cartesian_path_ │ ───────────────────────▶ │ JTC(轨迹控制器)     │
   │ executor.py     │      controller/joint_   │  样条插值→位置指令   │
   │ (喷涂执行器)    │        trajectory        └──────────┬──────────┘
   └───────┬─────────┘                                     │
           │ /spray_paint/trigger(ROS Bool)                ▼
           ▼                                     ┌─────────────────────┐
   ┌─────────────────┐                            │ gz_ros2_control/    │
   │ ros_gz_bridge   │                            │ GazeboSimSystem     │
   └────────┬────────┘                            │ (接口桥接)          │
            │ gz Boolean                           └──────────┬──────────┘
            ▼                                                ▼
   ┌─────────────────┐                            ┌─────────────────────┐
   │ 喷涂插件         │                            │ Gazebo物理引擎       │
   │ OnSprayMsg→      │                            │ →机械臂运动          │
   │ sprayActive_     │                            └─────────────────────┘
   └─────────────────┘
```

**一句话**：launch调度 → executor（双臂）→ 左边走JTC动机械臂、右边走桥开喷 → 插件留漆。

---

## 二、启动时序（launch 调度）

所有动作在 `ur_spray_demo.launch.py` 里用 `TimerAction` 错峰启动。

| 时间 | 动作 | 位置 | 为什么错峰 |
|------|------|------|-----------|
| T+0 | gz sim server（+可选GUI） | `gazebo` / `gazebo_headless` | 物理引擎最先就位 |
| T+0 | ros_gz_bridge（/clock + /spray_paint/trigger） | 固定启动 | 翻译官常开 |
| T+0 | robot_state_publisher（TF） | 固定启动 | 坐标变换 |
| T+0 | joint_state_publisher（初始姿态占位） | 固定启动 | JSB没起来前先撑住姿态 |
| T+8 | `gz service /world/{stem}/create` 注入UR5e | TimerAction(8.0) | 等gz server就绪；pose z=0.8m、绕z转90° |
| T+20 | spawn `joint_state_broadcaster` | TimerAction(20.0) | 机器人已注入，开始发真实关节状态 |
| T+25 | spawn `joint_trajectory_controller` | TimerAction(25.0) | **机械臂能动的开关** |
| 末尾 | move_group（MoveIt） | 固定启动 | 规划/可视化；加载失败仅告警不阻断 |

**下游依赖**：所有"要动机械臂"的程序必须等 **T+25**（JTC激活）之后才能发轨迹。
`run_stack.py` 的 tmux 里 executor 用 `sleep 20` 延迟，正是为此。

---

## 三、控制回路1：轨迹下发（executor → 机械臂）

```
cartesian_path_executor.py
   publish JointTrajectory(6关节: position+velocity+time_from_start)
   ▼
/joint_trajectory_controller/joint_trajectory  （话题）
   ▼
JTC（gz进程内controller_manager）100Hz
   │  样条插值，command_interfaces:[position]
   ▼
gz_ros2_control/GazeboSimSystem（xacro:89-92）
   │  ROS接口读写 ↔ gz sim关节
   ▼
Gazebo物理引擎 → 机械臂动
   ▼
关节状态 → JSB → /joint_states → 回喂executor
```

### 3.1 executor 的执行顺序（`run()`，`cartesian_path_executor.py`）

```
读YAML的joint_configs（≥2个路点）
  → 等JTC就绪（订阅数>0，60s超时）
  → 回HOME
  → 走到wp0（关喷）
  → try: 开喷 + 连续轨迹扫过所有路点
    finally: 无条件关喷          ← 修复：无论是否异常必关喷
  → 回HOME
  → 再补一刀关喷（最终保险）
```

### 3.2 两个关键控制技巧

1. **中间路点速度用中心差分**（第144-148行）：
   `v[i]=(q[i+1]-q[i-1])/(t[i+1]-t[i-1])`，首尾归零 → JTC不会在路点间刹停，
   8路点合成一条**连续扫描**轨迹。
2. **时长按关节最大差算**（第67-73行）：
   `Δt = max(1.5s, 最大关节差 / (1.0 × velocity_scaling))`，launch默认0.1。

### 3.3 为什么绕过 MoveIt

move_group在跑，但executor**直接发JTC**（不调用规划服务）。原因（脚本注释）：
MoveIt把地面放在机械臂底座平面（实际在底座下0.8m），导致**误判自碰撞**、
拒绝规划。所以实际运动控制 = executor→JTC；MoveIt只做可视化/规划场景。

---

## 四、控制回路2：喷头触发（ROS→桥→插件）

```
executor.set_spray()  →  publish std_msgs/Bool
   ▼
ROS  /spray_paint/trigger
   ▼
ros_gz_bridge（parameter_bridge）  direction: ROS_TO_GZ
   ▼
GZ   /spray_paint/trigger（gz.msgs.Boolean）
   ▼
插件 OnSprayMsg（SprayPaintPlugin.cc:484）
   → sprayActive_.store(msg.data())
   ▼
true : 创建粒子发射器 + 每10步射线扫描 + 沉积补丁
false: 移除发射器、停止扫描
```

### 4.1 触发方式演进（为什么改成ROS）

| 版本 | 路径 | 问题 |
|------|------|------|
| 旧 | executor `subprocess` 调 `gz topic` CLI | 一次性gz发现传输不可靠，**OFF消息被丢**，喷头一直开 |
| 新 | executor **rclpy进程内**发ROS → 桥 → 插件 | 可靠；桥断了才回退CLI（`trigger_method`参数） |

### 4.2 触发与轨迹的耦合

executor**单线程顺序执行**：`开喷 → 发扫描轨迹(等它走完) → 关喷`。
喷的时机 = 轨迹的时机，无并行。

---

## 五、控制回路3：状态反馈

```
Gazebo物理引擎 → JSB(100Hz) → /joint_states
                                ├→ executor：算当前姿态、确认JTC在线
                                ├→ move_group：规划场景同步
                                └→ RViz：可视化
```

- executor用`/joint_states`当"当前在哪"的反馈（`_js_cb`），做回home/走位。
- 控制器加载用`ros2 control list_controllers`验证（USE_GUIDE第4步）。

---

## 六、controller_manager 架构（已删除 standalone 节点）

整套系统里 controller_manager 实际上只有**一个**（之前有"两个"的设计已移除）:

| manager | 在哪 | 状态 |
|---------|------|------|
| gz内置 | `gz_ros2_control/GazeboSimROS2ControlPlugin`（URDF xacro:73-81）在**gz进程内** | ✅ **唯一真正干活的**。spawner连的`/controller_manager`就是它 |

**为什么删除 standalone ros2_control_node:**
- 原设计在 launch:T+2 启动一个独立 `ros2_control_node` 来加载控制器
- 但它会 SIGABRT（`pluginlib::LibraryLoadException`），因为 URDF 里的 `<plugin>gz_ros2_control/GazeboSimSystem</plugin>` 只能在 gz 进程内加载
- 实际的控制器由 **gz 插件内置的 manager** 负责（T+8 注入机器人时自动起动），spawner 连的就是它
- 删掉 standalone 节点后，控制器加载流程保持不变，只是去掉了每次必然崩溃的启动噪音

**控制器配置**（`ur_sim_controllers.yaml`）:
- manager `update_rate: 100`
- JTC：`command_interfaces:[position]`，`state_interfaces:[position,velocity]`
- 约束：`trajectory: 0.2 rad`，`goal: 0.1 rad`，`stopped_velocity_tolerance: 0.2`
- 硬件块（URDF xacro:89+）：每关节 `position+velocity` 指令接口，`position+velocity+effort` 状态接口

---

## 七、双时钟问题

| 节点 | 时钟 | 表现 |
|------|------|------|
| JTC / bridge / move_group / 插件 | **sim time**（`use_sim_time:true`） | 轨迹按sim时间执行 |
| executor | **wall clock**（`time.sleep()`） | 按真实时间估算轨迹时长 |

- 正常：gz实时跑（RTF≈1.0），两者吻合。
- 卡顿/重载：RTF<1 → JTC实际执行比executor估算慢 → 时序偏差。
- 缓解：executor在估算时长上`+0.5s`余量，且JTC的`trajectory_execution`容忍放宽。

---

## 八、常见"控制"类问题速查

| 现象 | 控制层原因 | 排查 |
|------|-----------|------|
| 机械臂不动 | JTC未激活（T+25没到/加载失败） | `ros2 control list_controllers` 看JTC是否active |
| 有轨迹但机械臂乱走 | 外来joint_states污染（DDS域） | `/joint_states` 是否只有UR5e 6关节（见USE_GUIDE Q1） |
| 喷了不停 | 触发OFF没送达（已修复：try/finally保证） | executor日志看`Spray OFF ... delivered` |
| 到点不喷 | 桥没起/触发话题没桥 | `ros2 topic info /spray_paint/trigger` 看桥节点 |
| executor报controller未就绪 | JTC订阅还没建立 | 等30秒，确认T+25已过 |
| move_group疯狂报错 | MoveIt与gz地面坐标不一致 | 属已知问题，executor已绕过MoveIt |

