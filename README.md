# UR5e 喷涂仿真 —— 完整使用指南（新手版）

> 更新时间：2026-08-21
> 这个文档写给**完全没接触过** ROS / Gazebo / Docker 的人。
> 每一步都告诉你：**做什么 → 为什么 → 怎么验证**。
> 看不懂命令没关系，直接照抄即可。

---

## 目录

- [一、先搞懂几个概念（必读）](#一先搞懂几个概念必读)
- [二、项目里有什么](#二项目里有什么)
- [三、手动启动全流程（重点！超详细）](#三手动启动全流程重点超详细)
  - [第0步 准备工作](#第0步-准备工作)
  - [第1步 启动容器](#第1步-启动容器)
  - [第2步 进入容器 / 建tmux](#第2步-进入容器--建tmux)
  - [第3步 启动仿真](#第3步-启动仿真)
  - [第4步 验证控制器就绪](#第4步-验证控制器就绪)
  - [第5步 启动喷涂程序](#第5步-启动喷涂程序)
  - [第6步 打开画面看效果](#第6步-打开画面看效果)
  - [第7步 打开RViz看机器人](#第7步-打开rviz看机器人)
  - [第8步 手动控制喷涂开关](#第8步-手动控制喷涂开关)
- [四、一键启动（startScript.sh）](#四一键启动startscriptsh)
- [五、常用操作速查表](#五常用操作速查表)
- [六、关闭与重新打开](#六关闭与重新打开)
- [七、故障排查（遇到问题先看这里）](#七故障排查遇到问题先看这里)
- [八、调试历史（DDS污染是怎么被解决的）](#八调试历史dds污染是怎么被解决的)
- [九、家具模型替换（汽车 → 桌子/柜子）](#九家具模型替换汽车--桌子/柜子)

> 📖 想知道**算法原理**（轨迹怎么算的、油漆怎么喷上去的）？看 [`ALGORITHM.md`](ALGORITHM.md)。
> 🔀 想知道**控制逻辑**（启动时序、机械臂怎么被指挥动的、喷头怎么开关）？看 [`CONTROL.md`](CONTROL.md)。

---

## 一、先搞懂几个概念（必读）

| 概念                            | 一句话解释                                                   |
| ------------------------------- | ------------------------------------------------------------ |
| **Docker 容器**                 | 一个"小电脑"，里面装好了所有软件。我们的仿真在容器里跑，不会弄脏你的真实电脑。 |
| **Gazebo Server**               | 仿真的"引擎"，负责计算物理、让机械臂真的动。它**没有画面**。 |
| **Gazebo GUI**                  | 仿真的"画面窗口"，连接Server就能看到3D场景。                 |
| **ROS 2 节点**                  | 一个小程序，负责一件事（比如读关节、规划路径）。             |
| **话题（topic）**               | 节点之间传递数据的"频道"。比如 `/joint_states` 发布机械臂关节角度。 |
| **参数（param）**               | 节点的设置项。比如 `robot_description` 就是机器人的URDF描述。 |
| **MoveIt / move_group**         | 机械臂的"大脑"，负责规划路径、避碰。                         |
| **controller_manager**          | 控制器总管，负责加载/启动控制器。                            |
| **joint_trajectory_controller** | 让机械臂按轨迹移动的控制器（简称JTC）。                      |
| **tmux**                        | 终端多窗口工具。在一个窗口里开多个子窗口，分别跑不同程序。   |
| **DDS 域**                      | ROS 节点互相发现的"频道组"。**必须用 ROS_DOMAIN_ID=10**，否则会被电脑上其他机器人项目的数据干扰（详见第八章）。 |

**核心流程一句话**：Gazebo让机械臂动 → controller_manager让控制器接管机械臂 → executor指挥机械臂做喷涂动作 → 插件在车身上留下油漆。

---

## 二、项目里有什么

```
~/gz_sim_spray_painting_plugin/
├── Dockerfile                    # 容器环境定义
├── startScript.sh                # 一键启动菜单（自动方式）
├── USE_GUIDE.md                  # 就是本文件
├── ALGORITHM.md                  # 算法原理（轨迹/运动学/喷涂模拟）
├── CONTROL.md                    # 控制逻辑（启动时序/三大控制回路/双时钟）
├── DEBUG_SUMMARY.md              # 调试过程总结
├── file_logs/                    # 喷涂日志/性能数据（宿主机能看到）
│   ├── spray_perf_n10.csv        #   喷涂性能数据（补丁数）
│   ├── start_rviz.sh             #   修好RViz的启动脚本（见第7步）
│   └── spray_loop.sh             #   循环喷涂脚本
├── run_scripts/
│   ├── build_code.py             # 编译脚本
│   ├── run_stack.py              # 世界选择+建tmux
│   └── docker/run_docker.sh      # 容器启动脚本
└── src/
    ├── gz_spray_painting_plugin/        # 喷涂插件本体（C++）
    └── gz_spray_painting_plugin_demo/   # 示例：URDF/launch/世界/配置
        ├── urdf/ur_spray_gz.urdf.xacro  # UR5e机器人描述（改这里调参数）
        ├── models/                      # 喷涂工件模型（SDF）
        │   ├── prius_hybrid/            #   汽车（原版demo）
        │   └── office_cabinet/          #   手写办公柜（家具demo，2026-08-21新增）
        ├── config/ur_sim_controllers.yaml  # 控制器配置
        ├── config/cartesian_poses.yaml   # 喷涂路径点
        ├── launch/ur_spray_demo.launch.py    # 仿真总启动文件
        ├── launch/cartesian_spray.launch.py  # 喷涂执行器
        └── worlds/
            ├── demo_car.sdf             # 汽车世界（UR模式）
            └── demo_cabinet.sdf         # 柜子家具世界（UR模式，2026-08-21新增）
```

---

## 三、手动启动全流程（重点！超详细）

> **手动方式** = 一步一步自己启动每个组件。适合想清楚知道每一步在干嘛的人。
> 不想折腾就用自动方式（第四章）。

### 第0步 准备工作

打开终端，检查镜像是否已构建：

```bash
docker images | grep spray_paint_plugin
```

如果**没有任何输出**，需要先构建镜像（约10-15分钟）：

```bash
cd ~/gz_sim_spray_painting_plugin
./startScript.sh
# 选 [3] Docker Build → 等它完成 → 选 [2] Code Build → Ctrl+C 退出菜单
```

> 构建只需要做一次。以后只要没改Dockerfile就不用再构建。

---

### 第1步 启动容器

```bash
cd ~/gz_sim_spray_painting_plugin
./run_scripts/docker/run_docker.sh detach
```

**做了什么**：在后台启动一个名为 `spray_paint_stack` 的容器。它已经设置好
`ROS_DOMAIN_ID=10`（防干扰）和 `DISPLAY=:0`（能显示画面）。

**怎么验证**：

```bash
docker ps | grep spray_paint_stack
```

看到一行记录 = 容器在跑。

> 如果已经有一个旧容器在跑，这个脚本会自动先停掉再起新的，不用担心。

---

### 第2步 进入容器 / 建tmux

**进容器**（进入仿真的"小电脑"）：

```bash
docker exec -it spray_paint_stack bash
```

你现在是在容器里的shell。**建议用tmux开几个窗口**，方便分别看不同程序的输出：

```bash
tmux new-session -d -s spray_paint -n sim
tmux new-window -t spray_paint -n cartesian
tmux new-window -t spray_paint -n control
tmux attach -t spray_paint
```

窗口分工：

- `sim` 窗口 → 跑仿真（第3步）
- `cartesian` 窗口 → 跑喷涂程序（第5步）
- `control` 窗口 → 手动开喷（第8步）

> **tmux切窗口**：按 `Ctrl+B` 松开，再按窗口名首字母或序号。
> `Ctrl+B` 然后 `0` 回 sim，`1` 去 cartesian，`2` 去 control。

---

### 第3步 启动仿真

在 **sim** 窗口里执行：

```bash
cd /ws && . install/setup.bash
ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py headless:=true
```

**做了什么**：

1. 启动 Gazebo Server（物理引擎，无画面——所以叫 `headless`）
2. 启动 ros_gz_bridge（Gazebo 和 ROS 的翻译官）
3. 启动 robot_state_publisher（发布机器人坐标变换）
4. 第8秒 把UR5e机械臂放进场景
5. 第20秒 启动关节状态广播器
6. 第25秒 启动轨迹控制器（JTC）——**机械臂能动的关键**
7. 启动 MoveIt（move_group，规划路径的大脑）

**怎么验证**：等30秒后，在 **另一个新终端**（不用退出容器，开新终端即可）里：

```bash
docker exec -it spray_paint_stack bash
```

然后：

```bash
cd /ws && . install/setup.bash
ros2 control list_controllers
```

**应该看到两行**（都是绿色 `active`）：

```
joint_state_broadcaster     ... active
joint_trajectory_controller ... active
```

> 看到 `joint_trajectory_controller ... active` 就说明机械臂可以被控制了。

---

### 第4步 验证控制器就绪

继续刚才的终端，确认关节数据是干净的（只有UR5e的6个关节）：

```bash
ros2 topic echo /joint_states --once
```

输出应包含 `shoulder_pan_joint`、`shoulder_lift_joint`、`elbow_joint`、
`wrist_1_joint`、`wrist_2_joint`、`wrist_3_joint`。

> ⚠️ 如果出现 `joint1`~`joint6` 或 `FR_foot` 之类的名字，说明DDS域没隔离好
> （看第七章Q1）。

---

### 第5步 启动喷涂程序

切到 **cartesian** 窗口：

```bash
cd /ws && . install/setup.bash
ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py
```

**做了什么**：一个程序自动指挥机械臂按8个路径点做喷涂扫描。

**你会看到这些日志**（这就是它在正常工作）：

```
Loaded 8 joint configs.
Moving to home configuration...
Approaching start position (spray OFF)...   ← 机械臂往车前走（约35秒）
Spray ON                                     ← 喷头武装
Starting continuous sweep (8 waypoints)...  ← 开始边移动边喷
Spray OFF
Returning to home configuration...
```

**怎么验证机械臂在动**（另开终端）：

```bash
docker exec -it spray_paint_stack bash
cd /ws && . install/setup.bash
ros2 topic echo /joint_states --once | grep -A 6 "^position:"
```

多次执行，看到数值变化 = 机械臂在动。

---

### 第6步 打开画面看效果

切到 **sim** 窗口（`Ctrl+B` `0`），你正在看仿真日志。要**看3D画面**，
再开一个**新终端**：

```bash
docker exec spray_paint_stack bash -c 'gz sim -g'
```

**做了什么**：`gz sim -g` = 只打开画面窗口（GUI），自动连上正在跑的
Gazebo Server。

**你会看到**（一个3D窗口）：

- 一辆 **Prius轿车**（被喷涂的工件）
- 一个底座上的 **UR5e机械臂**
- 机械臂正在扫描时，**车身上出现油漆**

> 这个窗口要一直开着（别关），它就是你的"摄像头"。
> 如果机械臂已经喷完回到原位，等它下一轮，或重新跑第5步。

---

### 第7步 打开RViz看机器人

RViz 可以显示 MoveIt 规划的机器人模型和坐标轴。再开一个**新终端**：

```bash
docker exec spray_paint_stack bash -c 'bash /ws/file_logs/start_rviz.sh'
```

**为什么不能直接 `rviz2`**：手动启动的 RViz 缺 `robot_description` 参数，
机器人模型是空的。`start_rviz.sh` 会自动从 `/robot_description` 话题
取URDF并传给RViz，机器人就能显示了。

> 等价的手动命令（如果你想知道原理）：
>
> ```bash
> docker exec -it spray_paint_stack bash
> cd /ws && . install/setup.bash
> URDF=$(ros2 topic echo /robot_description --once --field data)
> { echo "rviz:"; echo "  ros__parameters:"; echo "    robot_description: |"; \
> echo "$URDF" | sed "s/^/      /"; } > /tmp/rviz_params.yaml
> rviz2 -d /ws/install/gz_spray_painting_plugin_demo/share/gz_spray_painting_plugin_demo/config/moveit.rviz \
>    --ros-args --params-file /tmp/rviz_params.yaml
> ```

**RViz里你会看到**：UR5e机械臂模型、坐标轴（TF）、MoveIt规划场景。

> RViz和Gazebo画面是两个不同窗口，功能不同：
> Gazebo = 真实的物理仿真画面；RViz = MoveIt的规划/状态可视化。

---

### 第8步 手动控制喷涂开关

切到 **control** 窗口：

```bash
# 打开喷涂（喷头开始喷）
gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: true"

# 关闭喷涂
gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: false"
```

**注意**：光开喷还不够，**喷嘴必须对着车身**才有效果。所以通常让
第5步的executor自动控制（它会边移动边喷）。手动开喷适合机械臂已经
对准工件时用。

**怎么验证喷了**：看 `file_logs/spray_perf_n10.csv`：

```bash
tail -3 ~/gz_sim_spray_painting_plugin/file_logs/spray_perf_n10.csv
```

每行最后一列 `patches_created` 大于0 = 留下了油漆补丁。

---

## 四、一键启动（startScript.sh）

不想一步步来？用自动方式：

```bash
cd ~/gz_sim_spray_painting_plugin
./startScript.sh
```

| 选项                    | 功能                                                         |
| ----------------------- | ------------------------------------------------------------ |
| **[1] Start Stack**     | 自动：起容器 → 选世界 → 建tmux（sim/cartesian/spray_control） |
| **[2] Code Build**      | 改代码后重新编译                                             |
| **[3] Docker Build**    | 构建镜像（首次）                                             |
| **[4] Empty Container** | 打开容器shell                                                |

选了[1]后：

1. 弹出**世界选择菜单** → 选 `[1] demo_car`（汽车喷涂）或 `demo_cabinet`（柜子家具喷涂）
2. 自动建3个tmux窗口：
   - 窗口0 `sim` = 仿真
   - 窗口1 `cartesian_spray` = 20秒后自动开始喷涂
   - 窗口2 `spray_control` = 喷涂开关（上下两格，按Enter触发）
3. 要**看画面**还是得手动执行第6步（`gz sim -g`）和第7步（RViz）。

---

## 五、常用操作速查表

| 想做什么     | 命令                                                         |
| ------------ | ------------------------------------------------------------ |
| 启动容器     | `./run_scripts/docker/run_docker.sh detach`                  |
| 进容器       | `docker exec -it spray_paint_stack bash`                     |
| 启动仿真     | `ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py headless:=true` |
| 跑柜子demo   | `./startScript.sh` → [1] Start Stack → 选 `demo_cabinet`（2026-08-21新增） |
| 看控制器     | `ros2 control list_controllers`                              |
| 启动喷涂     | `ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py` |
| 看Gazebo画面 | `docker exec spray_paint_stack bash -c 'gz sim -g'`          |
| 看RViz       | `docker exec spray_paint_stack bash -c 'bash /ws/file_logs/start_rviz.sh'` |
| 开喷         | `gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: true"` |
| 停喷         | `gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: false"` |
| 查关节角度   | `ros2 topic echo /joint_states --once`                       |
| 看喷涂数据   | `tail ~/gz_sim_spray_painting_plugin/file_logs/spray_perf_n10.csv` |
| 循环喷涂     | `docker exec spray_paint_stack bash -c 'bash /ws/file_logs/spray_loop.sh'` |
| 关闭整套     | `docker rm -f spray_paint_stack`                             |

---

## 六、关闭与重新打开

### 只关画面窗口（仿真继续）

在tmux里 `Ctrl+B` → 窗口序号 → 按 `Ctrl+C`。

- 关闭 Gazebo 画面 / RViz 不影响仿真。

### 整套关闭（推荐重开方式）

```bash
docker rm -f spray_paint_stack
```

一条命令停掉仿真、tmux、所有窗口。

### 重新打开

```bash
cd ~/gz_sim_spray_painting_plugin
./run_scripts/docker/run_docker.sh detach      # 手动方式
# 或
./startScript.sh                                # 自动方式
```

然后按第2步到第8步重新走（或选startScript的[1]）。

---

## 七、故障排查（遇到问题先看这里）

### Q1: 出现 `Joint 'joint1' not found in model 'ur'` 或 Go2的帧名

**原因**：ROS_DOMAIN_ID没设好，被电脑上其他机器人项目（Go2/rm_eco65）的
残留数据污染。
**检查**：`docker exec spray_paint_stack bash -c 'echo $ROS_DOMAIN_ID'` 应输出 `10`。
**解决**：杀掉容器重新用 `run_docker.sh detach` 起（它内置域10）。
**完整说明**：见第八章。

### Q2: RViz里面机器人是空的

**原因**：手动启动rviz2没带 `robot_description` 参数。
**解决**：用 `bash /ws/file_logs/start_rviz.sh` 启动（会自动带上URDF）。

### Q3: 机械臂不动，`joint_trajectory_controller` 不是active

**原因**：控制器没起来，或DDS污染让spawner连错。
**解决**：

```bash
docker exec -it spray_paint_stack bash
cd /ws && . install/setup.bash
ros2 control list_controllers
```

应看到两个active。如果JTC不是active，等30秒再看；还不行就重启容器。

### Q4: Gazebo画面空白 / 连不上

**原因**：`gz sim -g` 可能连到了错误的server，或DISPLAY没设。
**解决**：确认用 `gz sim -g`（只开GUI），且容器里 `echo $DISPLAY` 是 `:0`。

### Q5: 编译报错/卡住

**原因**：内存不够（只有5.7GB）。
**解决**：编译前关掉Gazebo画面和RViz，只保留容器。编译脚本已限制并行度。

### Q6: 手动开喷了但车身没油漆

**原因**：喷头没对准工件（光开喷没用，要喷在车上）。
**解决**：让executor自动控制（第5步），它会边移动边喷。

### Q7: 想换油漆颜色 / 改喷射角度

修改 `src/gz_spray_painting_plugin_demo/urdf/ur_spray_gz.urdf.xacro` 里
插件的参数：

```xml
<spray_color>1.0 0.2 0.1 1.0</spray_color>    <!-- RGBA颜色 -->
<cone_half_angle_deg>15</cone_half_angle_deg>  <!-- 锥角 -->
<cone_max_range>1.0</cone_max_range>           <!-- 射程 -->
<particle_rate>100</particle_rate>             <!-- 粒子密度 -->
<num_rays>16</num_rays>                        <!-- 采样射线数 -->
```

改完重新编译：`./startScript.sh` → [2] Code Build，再重启仿真。

---

## 八、调试历史（DDS污染是怎么被解决的）

### 症状

之前 move_group 一直报错：

```
Joint 'joint1'~'joint6' not found in model 'ur'
Unable to transform object from frame 'FR_foot' / 'imu_link' ...
```

而且 `ros2 node list` 出现一堆奇怪的节点：`mujoco_go2_bridge`、
`rm_group_controller`、`slam_toolbox`……

### 排查过程

- 镜像重装、代码重编译、缓存全清——都试了，没用。
- 最后发现：代码、URDF、SRDF**全是干净的**（模型明明只有UR5e）。
- 对比 `ros2 node list`（有鬼）和 `ros2 node list --no-daemon`（干净）
  → 确认是**ROS 2的发现缓存**在搞鬼。

### 根本原因

容器是用 `--network host` 启动的（共享宿主机网络）。以前在这台电脑上
跑过的 **Go2四足 + rm_eco65机械臂 + 导航** 项目，它们的ROS节点数据
还残留在同一个DDS域（默认域0）里。move_group 订阅 `/joint_states`
和碰撞话题时，收到了这些"别人的"数据，套到UR5e模型上就报错。

### 解决方案

给整套栈一个**独立的DDS域号** `ROS_DOMAIN_ID=10`，让它和域0彻底隔开。

```bash
# 在 run_docker.sh 里加一行（已加好）：
# DOCKER_ARGS+=("-e" "ROS_DOMAIN_ID=10")
```

**效果**：move_group 干干净净地加载UR5e，控制器正常激活，机械臂自动喷涂成功。

### 经验教训

1. `--network host` 的容器 = 和宿主机共享DDS域 → 必须隔离。
2. 报错里的外来关节/帧名，是查"数据从哪来"的线索。
3. 镜像/代码干净 ≠ 运行环境干净，还要查网络层的发现数据。

---

## 九、家具模型替换（汽车 → 桌子/柜子）

> 更新时间：2026-08-21
> 把喷涂工件从"汽车"换成"家具"（桌子/柜子）的完整操作记录与指南。
> 前提：整套 stack 已经能跑通 `demo_car`（见第三章）。

### 9.1 先纠正一个关键认知

**汽车其实不是 URDF，是 SDF 模型。** 项目里的 URDF 是 **UR5e 机械臂**，不是车。

| 东西           | 是什么                                                       | 在哪                          |
| -------------- | ------------------------------------------------------------ | ----------------------------- |
| 汽车（Prius）  | SDF 模型（含 OBJ 网格）                                      | `models/prius_hybrid/`        |
| 汽车怎么被加载 | 世界文件里 `<include><uri>model://prius_hybrid</uri></include>` | `worlds/demo_car.sdf`         |
| UR5e 机械臂    | 这才是 URDF                                                  | `urdf/ur_spray_gz.urdf.xacro` |

所以"换家具"实际是两件事：**① 换被喷涂的工件模型，②（位置/形状不同时）重算喷涂轨迹**。

### 9.2 喷涂链路（一句话）

```
家具模型放在场景里 → 机械臂按 cartesian_poses.yaml 的关节角路点移动
  → 喷嘴发锥形射线命中家具表面 → 插件把油漆补丁贴到命中 link
```

机械臂"喷哪里"完全由 `config/cartesian_poses.yaml` 决定。**只要家具放在原车被喷的位置，轨迹就不用重算**；换了位置/形状才需要重算（见 9.7）。

### 9.3 这次改了什么（2026-08-21）

| 文件                                         | 改动                                                         |
| -------------------------------------------- | ------------------------------------------------------------ |
| `models/office_cabinet/model.sdf`（新增）    | 手写双开门柜子：1.2×0.5×1.7 m，单 link，collision=前脸（喷涂命中面） |
| `models/office_cabinet/model.config`（新增） | 模型注册文件（`model://office_cabinet` 靠它解析）            |
| `worlds/demo_cabinet.sdf`（新增）            | 复制 demo_car，柜子前脸放在原车近侧面 y=0.945                |
| `launch/ur_spray_demo.launch.py`             | 世界名参数化：新增 `world:=` 参数，机械臂注入服务改为 `/world/{stem}/create` |
| `run_scripts/run_stack.py`                   | `demo_cabinet` 加入 `UR_WORLDS` 集合，菜单可选 UR 模式       |

**为什么柜子放在世界 y=1.195（前脸 y=0.945）**：
机械臂底座在世界 (0, 0, 0.8)、绕 z 转 90°；原有 8 个路点喷的是一条约在 y=0.36、z=1.03、x∈[-0.35, 0.35] 的带。原车近侧面就在 y≈0.945（车在 (0.24, 1.82)，宽 1.75m）。柜子深度 0.5m → 前脸 y = 1.195 − 0.25 = 0.945，正好落在原车喷漆面上，所以轨迹不用动。

### 9.4 家具模型必须满足的 3 个条件

1. **visual + collision 都要有**（插件用碰撞体判定命中，只有 visual 会喷不上）。
2. **位置在机械臂可达范围内**（UR5e 工作半径约 0.85m；喷嘴到工件距离 < 0.8m 射程）。
3. **`static: true`**（静止工件，和车一样）。

> 细节技巧：油漆补丁贴在碰撞面外 1.5mm 处。任何**凸出前脸的 visual** 都会盖住油漆。所以前脸的门板等细节要内嵌在碰撞面之后；把手可以略凸出（只挡住一小块，可接受）。

### 9.5 想再换一个家具（操作步骤）

1. 在 `models/<名字>/` 下建 `model.sdf` + `model.config`（照抄 `office_cabinet` 改尺寸即可）。
2. 新建 `worlds/demo_xxx.sdf`（复制 `demo_cabinet.sdf`，改 `<world name>`、`<include>` 里的模型 URI 和 `<pose>`）。
   - 摆放：让要喷的面在 **世界 y≈0.945 附近、面朝 -Y（朝机械臂）、高度覆盖 z≈1.03**。
   - 位置差一点 → 只调 `<pose>` 的 x/y/z，不用改轨迹。
3. `run_scripts/run_stack.py` 的 `UR_WORLDS` 集合里加上新世界名（否则会掉进"仅喷头"的 demo 模式，看不到机械臂）。
   - ⚠️ 还要确认 UR 模式的 sim 启动命令带了 `world:={world_stem}`（2026-08-21 曾漏掉这行，导致选了家具世界却还是启动 demo_car，见 9.9）。
4. `./startScript.sh` → **[2] Code Build** 重新编译安装（launch / worlds / models 的改动需要这一步）。
5. 启动验证（见 9.6 验证清单）。

### 9.6 验证清单（本次已通过 + 你跑通后自查）

| 项       | 方法                                                         | 结果                              |
| -------- | ------------------------------------------------------------ | --------------------------------- |
| 模型语法 | `gz sdf -k models/office_cabinet/model.sdf`                  | ✅ Valid                           |
| 世界能起 | `gz sim -s worlds/demo_cabinet.sdf -r`                       | ✅ 零 error                        |
| 柜子在场 | `gz model --list`                                            | ✅ office_cabinet / robot_pedestal |
| 柜子位置 | `gz model -m office_cabinet -p`                              | ✅ (0, 1.195, 0)                   |
| 喷上漆   | `tail ~/gz_sim_spray_painting_plugin/file_logs/spray_perf_n10.csv` 看 `patches_created>0` | 你跑后自查                        |
| 画面效果 | `gz sim -g` 看柜门前红漆                                     | 你跑后自查                        |

> ⚠️ 注意：`gz sdf -k` 校验**世界文件**会报 `Unable to find uri[model://...]`——这是单测工具不加载 Gazebo 资源回调的**正常现象**，原版 `demo_car.sdf` 也会报同样错。真正的 `gz sim` 运行不会有问题。

### 9.7 后续计划

1. **桌子（水平桌面）**：要改 `scripts/generate_spray_poses.py`，加"喷嘴朝下、水平面扫描"模式，重新生成 `cartesian_poses.yaml`（现有生成器只会固定 z 的水平扫，只适合垂直面）。
2. **精美模型**：把 `office_cabinet/` 目录整体换成下载的家具模型，保持 front face（-Y）朝机械臂即可。
3. **多工件**：世界文件里放多个 `<include>`，配不同轨迹。
4. **装按 Harmonic 编译的 ros_gz_bridge**（让 `ros` 触发和 `/clock` 都通）：当前 Debian 版是 Fortress 编译（`ignition.msgs`），和 Harmonic server（`gz.msgs`）不兼容（见 9.10）。装好后把 `cartesian_spray.launch.py` 的 `trigger_method` 切回 `ros`，`/clock` 也恢复。

### 9.8 DDS 隔离提醒（换模型不影响）

本次改动没有触碰任何网络/DDS 代码，`ROS_DOMAIN_ID=10` 原样保留。跑起来后照旧自查：

- `docker exec spray_paint_stack bash -c 'echo $ROS_DOMAIN_ID'` → 应输出 `10`。
- `ros2 topic echo /joint_states --once` → 应只有 UR5e 的 6 个关节（`shoulder_pan_joint` 等）。
- 出现 `joint1~6` / `FR_foot` 等外来名字 = 老问题复发信号，重启容器再起。

### 9.9 翻车记录：选了家具世界却还是汽车（2026-08-21 已修复）

**症状**：`./startScript.sh` 选了 `demo_cabinet`，打开 `gz sim -g` 看到的却还是汽车。

**排查**：`docker exec spray_paint_stack bash -c 'gz topic -l | grep "scene/info"'` 输出
`/world/demo_car/scene/info` → 说明**运行的服务器本身加载的就是 demo_car**，不是 GUI 连错。

**根因**：`run_scripts/run_stack.py` 的 UR 模式启动 sim 的命令写死了，没带世界名：

```python
# 错误（修复前）
send("sim.0", f"{ROS} && ros2 launch ... ur_spray_demo.launch.py headless:=true")
# 正确（修复后）：必须把菜单选中的世界名传进去
send("sim.0", f"{ROS} && ros2 launch ... ur_spray_demo.launch.py headless:=true world:={world_stem}")
```

于是 launch 用了默认值 `demo_car`——菜单选了柜子，实际起的是汽车。

**经验教训**：参数化 launch 后，**所有调用它的入口都要同步传参**，漏一个入口就会"菜单生效、实际不生效"。本项目的两个入口是 `ur_spray_demo.launch.py`（已在 9.3 参数化）和 `run_stack.py` 的 UR 分支（本次修复）。改完 `run_stack.py` 是运行时脚本，**不用重新编译**，重启容器即可。

### 9.10 翻车记录：自动扫描不喷漆，手动 `gz topic` 却可以（2026-08-21 已修复）

**症状**：选 `demo_cabinet` 后，executor 日志显示 `Spray ON [ros] delivered → Starting continuous sweep → Spray painting complete`，但柜子没漆；`spray_perf_n10.csv` 只有表头（一行数据都没有）。在 spray_control 窗口手动 `gz topic -p /spray_paint/trigger ... data:true` → 立刻开始喷。

**排查**（逐层缩小）：

1. 手动 `gz topic -p` → 插件收到（sim 窗口出现 `OnSprayMsg ACTIVE`）→ 插件的订阅没问题。

2. 手动 `ros2 topic pub /spray_paint/trigger` → 插件**没收到**；桥日志出现过 `Passing message from ROS ... to Gazebo`。

3. `/clock`（桥的 GZ→ROS 方向）也不流 → **桥和 gz server 之间两个方向全断**。

4. `gz topic -i -t /spray_paint/trigger` 看到发布者/订阅者**类型不一致**：

   ```
   Publishers : tcp://...:46037, ignition.msgs.Boolean   ← 桥注册的
   Subscribers: tcp://...:36105, gz.msgs.Boolean         ← 插件注册的
   ```

5. 查进程加载的库：桥 `parameter_bridge` 加载 **`libignition-msgs8.so`**（Fortress 时代，`ignition.msgs` 命名空间）；gz sim server（Harmonic）用 **`libgz-msgs10.so`**（`gz.msgs`）。gz-transport 把两种消息类型当成**不同类型**，跨进程直接丢弃（桥启动日志的 `Unknown message type [8]` 就是它）。

**根本原因**：容器里的 Debian `ros-humble-ros-gz-bridge`（0.244.25）是**按 Ignition Fortress 编译的**，注册的是 `ignition.msgs.*` 类型；而 gz sim 是 **Harmonic**，插件订阅 `gz.msgs.*`。两者类型哈希不一致，桥永远无法把消息递给插件——**之前汽车 demo 的自动喷洒其实也从没走通桥**，靠的一直是 spray_control 窗口的手动 gz 触发。

**修复**：executor 本来就预留了 `trigger_method` 参数（`ros | gz_cli | both`），但默认 `ros`，且 `_publish_ros` 用 `get_subscription_count() > 0` 判断"已投递"——桥订阅了该 topic，所以永远返回成功，把 gz_cli 回退给跳过了。本次改动：

1. `cartesian_path_executor.py`：默认 `trigger_method` 改为 **`gz_cli`**（直接 `gz topic -p`，绕开废桥、走已验证可靠的路径）；`_publish_gz_cli` 加重试（3 次，防 docker 网桥偶发丢首包）。
2. `cartesian_spray.launch.py`：新增 `trigger_method` 参数透传（默认 `gz_cli`），以后装好 Harmonic 桥可切回 `ros`。
3. 两个文件都是 Python、`--symlink-install` 安装 → **改源码即生效，不用重新编译**；重启栈（Ctrl+C 旧 sim → 重发 launch 命令）即可。

**验证**（2026-08-21 通过）：

- executor：`Spray ON [gz_cli] delivered` → `Spray OFF [gz_cli] delivered`
- 插件：`OnSprayMsg trigger ACTIVE`（15:50:59）→ `INACTIVE`（15:51:20.9）
- perf：430 行，扫描窗口 sim 63.7→85.1（正好 21.4 s = 扫描行程时长），`valid_hits=16`（16 条射线全命中柜子正面），累计 **112 个 patch**
- patch 归属：`patch_count: 1 links have patches` = **cabinet_body**（柜子本体）
- perf 停止增长 → OFF 生效，无残留喷漆、没喷到地板

**经验教训**：

1. **"发布成功" ≠ "送达"**：`get_subscription_count() > 0` 只证明桥订阅了，不证明插件收到。跨进程链路要按**目标端日志/效果**验证（perf 有数据行 + 插件 `ACTIVE`），不能只看发布方日志。
2. **同一个 gz 生态里 `ignition.msgs` 和 `gz.msgs` 是两套不兼容的类型**：Fortress 编译的桥喂不进 Harmonic 的 server。换组件/换版本后，用 `gz topic -i` 看发布者/订阅者的 Message Type 是否一致。
3. **`/clock` 桥同样断**（GZ→ROS 也是同一毛病）。当前 executor 用墙钟、不依赖 `/clock`，功能不受影响；要彻底治好（让 `ros` 触发和 `/clock` 都通），需要装**按 Harmonic 编译的 ros_gz_bridge**（列入 9.7 后续计划）。

### 9.11 结论：为什么"喷满整个柜子前脸"做不到（2026-08-21 记录）

**目标**：让自动喷涂覆盖柜子前脸的**整个面**（不只当前 z≈1.03 那一条横带），路径重新规划成满面栅格。

**结论先说**：**在当前布局下做不到，这是机械臂几何可达性的硬限制，不是路径规划/代码没写对。** 当前演示保留单条横带扫描，已验证能喷。

**排查过程**（用什么方法得出这个结论）：

1. 写了一个自包含的可达性分析脚本 `scripts/feasibility_check.py`（UR5e 标准 DH 正解 + Newton-DLS 逆解，纯 numpy，不依赖容器里没装的 scipy）。
2. 先自检：用逆解复现 `config/cartesian_poses.yaml` 里全部 8 个工作路点的关节角，位置误差 **0.0 mm** —— 证明运动学模型是对的。
3. 跑可达性地图。**一个重要坑**：逐格"是否存在逆解"的地图是**分支相关的**，同一格换个种子就判成不可达（出现"夹在两个可达格中间却是 X"的碎片图）。可靠的做法是**沿真实栅格路径做链式逆解**（相邻格用上一格解作种子），这才是机械臂连续走时真正能达到的范围。
4. 链式实测（站距 0.585m，喷嘴世界 y=0.36）给出真实可达范围：**喷嘴世界 x∈[-0.40,+0.40]、z∈[0.7,1.3] 全程可达**（测试区 0.4×0.4 步长的 81 格里 79 格可达，98%）；能完整刷到的最大安全前脸约 **0.7 m 宽 × 0.6 m 高，中心放在世界 z≈1.0**。

**根本原因（为什么整个面不行）**：

- **几何臂展**：UR5e 肩部在世界 z≈0.96（基座 0.8 + 第一连杆 0.1625），最大臂展半径约 **0.85 m**。柜子前脸 1.2 m 宽 × 1.7 m 高（x∈[-0.6,0.6]，z∈[0,1.7]，在 y=0.945）。
  - 顶角（x=±0.6，z=1.7）到肩部距离约 **1.1 m，超出臂展**——无论怎么重新规划路径都到不了。
  - 底部 z<0.2 的区域同样够不着。
- **姿态约束**：喷嘴 tool 的 z 轴固定朝前脸方向、**俯角约 14°**，加上 0.585 m 站距，把可达窗口进一步压缩到大约 z∈[0.5,1.35] 的整宽范围。所以即便只喷前脸中部，能整面覆盖的矩形也被限制在肩部高度附近一小块。

**为什么"把柜子改小"也没直接干成**：

- 可达窗口中心在肩部高度 z≈1.0。把柜子改小但还放地上（z∈[0,0.6]），前脸照样落在窗口**下方**——必须**垫高**（加台面）让它落在 z∈[0.62,1.18]，再缩小到 0.7×0.56，前脸才能整面被刷到。这需要改世界文件（加台面模型）+ 改柜子模型 + 重生成栅格路径，是一套改动。
- 生成栅格时还碰到一个可解的细节坑：栅格第一个路点放在 x=-0.40（远离已知可达分支的一侧），从所有恢复种子（WP0/零位/Home）都收敛失败；改成从 x=+0.40（已知 WP0 分支一侧）起链即可绕过。**这个是细节，不是硬限制**——真正的硬限制还是臂展够不到。

**最终决定**（本次已实施 / 未实施）：

- **保留**：当前单条横带扫描（世界 z≈1.03，x∈[-0.35,0.35]），已验证能喷、效果稳定，作为演示。
- **未实施**：满面栅格。因为几何上就不可能喷满原尺寸前脸，所以没改世界/模型/路径，全部还原，demo 保持原样。
- **分析脚本留在 `scripts/`**：`feasibility_check.py`、`fullface_test.py`、`generate_fullface.py` —— 纯离线测算，不影响 demo 运行，以后想捡起来随时可用。

**如果以后真想要"喷满整面"，唯一干净的路**（二选一，都涉及改布局）：

1. **抬高机械臂基座**，让肩部对准前脸中心（比如基座抬到 z≈1.4，肩部 z≈1.56），整个前脸就都在臂展内；
2. **抬高并缩小柜子**：加一个台面把柜子垫到肩部高度，同时把前脸缩到约 0.7×0.6 放进可达窗口（本次已验证该尺寸可行），再按链式逆解生成栅格路径。

### 9.12 以后想喷满整面：结构性步骤（思路，不用照抄细节）

推荐走 **方案 2（垫高 + 缩小柜子）**：只改 SDF 世界层和路径，不碰机器人/URDF/MoveIt，改动面最小。整体思路是 **"先量出可达窗口 → 再改布局 → 再生成栅格 → 跑通 → 固化"**。

**步骤 0｜确定目标前脸尺寸（先量再做，不要拍脑袋）**

- 用链式逆解实测当前站位的可达窗口（方法已在 9.11 记录：`fullface_test.py`）。
- 取窗口的**安全内切矩形**做前脸。本次已验证：前脸 **0.7 m 宽 × 0.6 m 高、中心 z≈1.0** 全程可达，留有余量。

**步骤 1｜改布局（世界层，只动 SDF）**

- 加一个静态台面模型，把柜子垫到肩部高度（如台面顶 z≈0.62，柜子前脸落在 z∈[0.62,1.18]）。
- 把柜子模型缩到步骤 0 定的尺寸，**前脸仍对齐 y=0.945**（喷头站距不变）。
- 验证：`demo_cabinet.sdf` 是 symlink 装进 install，改源码即生效，不用重新编译。

**步骤 2｜生成栅格路径（核心思路）**

- 用 `generate_fullface.py` 的框架：boustrophedon 栅格，4~5 条横带，**横带间距 ≤ 单条漆面高度**（约 0.15 m，保证重叠不漏）。
- 每行横扫 x 略超前脸边缘（如前脸 ±0.35 就扫到 ±0.40），让行间的垂直连接段喷到柜子外面，不会在脸上留下竖线。
- 两个已踩过的坑要处理：① 栅格**首点从已知可达分支一侧起链**（x=+0.40，别从 -0.40 起）；② 喷嘴俯角 14° 使落点比喷嘴 z 低约 0.148 m，横带 z 要按落点校正。

**步骤 3｜验证后写 YAML**

- 链式逆解全部路点通过（无 FAIL、相邻路点关节步长平滑），输出覆盖 `config/cartesian_poses.yaml`。

**步骤 4｜跑一遍看效果**

- 重启栈，看横带是否整面覆盖、有没有漏带或喷出边界；按实际落点微调横带 z，迭代一两轮即可。

**步骤 5｜固化**

- 更新 USE_GUIDE（新尺寸/路径参数/坑），`cartesian_poses.yaml` 和脚本存档，把本次 9.11 的结论衔接上。

> 方案 1（抬高基座）的思路是：改 URDF/launch 里的基座高度 → 重算「世界↔基座」z 偏移（`world_to_base` 里 `-0.8` 那项）→ 重生成栅格。更彻底，但改动面大，除非想保留原尺寸柜子才选它。
