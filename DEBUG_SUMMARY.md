# UR5e 喷涂仿真调试总结

> 日期：2026-08-19
> 项目：gz_sim_spray_painting_plugin（UR5e 自动喷涂）

---

## 一、问题的产生

### 症状
启动 `ur_spray_demo.launch.py` 后，MoveIt 报错：
```
Joint 'joint5' not found in model 'ur'
```
同时 RViz / move_group 加载的是 **rm_eco65** 机器人模型，而不是 UR5e。

### 根本原因
系统中存在 **三层残留污染**，导致新项目启动时读取到旧配置：

| 污染层 | 说明 | 是否曾为问题根源 |
|--------|------|-----------------|
| ① colcon 构建产物（build/install） | 之前的 rm_eco65 项目构建缓存 | ✅ 最深层的污染源 |
| ② ROS 参数缓存（~/.ros/） | 参数服务器缓存了旧的 robot_description | ✅ |
| ③ Docker 镜像 | 镜像构建时 baked 了旧配置 | ✅ 表层 |

> 之前仅清理其中一层或两层，问题反复出现。

### 为什么会加载错误模型
ROS 2 参数服务器是全局共享的。MoveIt / controller_manager 启动时：
1. 从参数服务器读取 `robot_description`
2. 读到的是缓存中的 rm_eco65（joint1~joint6 命名）
3. 而项目传给它的 UR5e URDF（shoulder_pan_joint 等）被忽略
4. 导致 joint 名不匹配 → `Joint 'joint5' not found`

---

## 二、排查过程

1. **确认污染来源**
   - `ros2 param get /move_group robot_description` → 显示 `name="rm_eco65_description"`
   - 确认不是 launch 文件问题，而是参数服务器读取到旧值

2. **逐层清理**
   - ❌ 仅清理 `~/.ros/` → 无效（colcon 缓存还在）
   - ❌ 仅删除 Docker 镜像重建 → 无效（build/install 缓存还在）
   - ✅ **三层全清** → 成功

3. **发现 gz_ros2_control 为空目录**
   - `src/gz_ros2_control` 是空的（git 孤儿链接，非 submodule）
   - 导致 ros2_control_node 崩溃、gz_ros2_control-system 插件加载失败
   - **解决**：克隆官方 humble 分支
     ```
     git clone -b humble https://github.com/gazebosim/gz_ros2_control.git src/gz_ros2_control
     ```

4. **launch 文件问题**
   - `/tmp/ur_spray_generated.urdf` 权限不足 → launch 崩溃
   - **解决**：添加 try/except 回退到家目录

---

## 三、解决方案

### 彻底清理命令
```bash
# 1. 清理 Docker
docker rm -f spray_paint_stack 2>/dev/null
docker rmi spray_paint_plugin 2>/dev/null
docker system prune -af

# 2. 清理 colcon 产物（关键！）
cd ~/gz_sim_spray_painting_plugin
sudo rm -rf build install log

# 3. 清理 ROS 缓存
rm -rf ~/.ros/

# 4. 重新构建干净镜像
docker build -t spray_paint_plugin .
```

### 关键修复
1. **补齐 gz_ros2_control**（ros2_control 与 Gazebo 的桥接插件）
   ```bash
   git clone -b humble --depth 1 https://github.com/gazebosim/gz_ros2_control.git src/gz_ros2_control
   ```

2. **launch 文件权限回退**（`ur_spray_demo.launch.py`）
   ```python
   urdf_tmp = "/tmp/ur_spray_generated.urdf"
   try:
       with open(urdf_tmp, "w") as f:
           f.write(urdf_str)
   except PermissionError:
       urdf_tmp = os.path.expanduser("~/ur_spray_generated.urdf")
       with open(urdf_tmp, "w") as f:
           f.write(urdf_str)
   ```

3. **controller_manager 延迟启动**（等 robot_description 传播）
   ```python
   controller_manager = TimerAction(
       period=2.0,  # 等待 robot_description 设置完成
       actions=[controller_manager_node],
   )
   ```

---

## 四、启动与验证

### 启动容器（只挂载 src，install 用镜像内的完整版本）
```bash
docker run -d --name spray_paint_stack \
  -v /home/zzh/gz_sim_spray_painting_plugin/src:/ws/src \
  spray_paint_plugin \
  bash -c "cd /ws && . install/setup.bash && ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py headless:=true"
```

### 验证模型正确
```bash
docker exec spray_paint_stack bash -c \
  '. /opt/ros/humble/setup.bash && . /ws/install/setup.bash && \
   ros2 param get /move_group robot_description | grep -o "name=\"[^\"]*\"" | head -3'
```
✅ 应显示 `name="ur"`（UR5e），而不是 `name="rm_eco65_description"`

### 验证节点
```bash
docker exec spray_paint_stack bash -c \
  '. /opt/ros/humble/setup.bash && . /ws/install/setup.bash && ros2 node list'
```
应包含：`/move_group`、`/controller_manager`、`/robot_state_publisher` 等

---

## 五、结果

| 项目 | 状态 |
|------|------|
| MoveIt 加载 UR5e robot_description | ✅ `name="ur"` |
| 关节名（shoulder_pan_joint 等） | ✅ 正确 |
| gz_ros2_control 插件 | ✅ 已补齐 |
| controller_manager 正常启动 | ✅ |
| Cartesian Spray Executor | 🔄 待验证 |

---

## 六、经验教训

1. **Docker 镜像重建 ≠ 干净** — colcon 缓存（build/install）和 ~/.ros/ 才是深层污染源
2. **参数服务器是全局的** — 旧项目的 robot_description 会干扰新项目
3. **检查空目录** — 克隆项目后要验证 `src/` 下的每个包目录都有实际内容
4. **colcon --symlink-install** — install 里的文件是 src 的符号链接，改源码即时生效（配合挂载）

---

## 七、真正的元凶：DDS 域污染（2026-08-19 追加）

### 症状
- move_group 启动时模型加载**完全干净**（`Loading robot model 'ur'`），但运行中狂刷错误：
  - `Joint 'joint1'~'joint6' not found in model 'ur'`（rm_eco65 关节）
  - `Unable to transform object from frame 'FR_foot'/'RL_calf_rotor'/'imu_link'...`（Go2 四足机器人的帧）
  - 7 分钟内刷了 **52MB 日志** 后崩溃
- `ros2 node list` 出现幽灵节点：重复的 controller_manager / move_group、`mujoco_go2_bridge`、`rm_group_controller`、slam_toolbox 等

### 根本原因
容器以 `--network host` 运行，**与宿主机共享同一个 DDS 域（domain 0）**。
用户之前做过的 **Go2 四足 + rm_eco65 机械臂 + Nav2/slam 栈** 的 DDS 发现元数据/残留进程
（甚至可能是局域网其他机器）在 domain 0 上继续广播。

move_group 的 planning_scene_monitor 订阅 `/joint_states`、`/collision_object` 等话题，
收到了这些外来节点发布的**别家机器人的关节和碰撞对象**，套用到 UR5e 模型上 → 报错刷屏。
独立的 ros2_control_node 也因配置/冲突 SIGABRT。

### 解决方案：DDS 域隔离
给整套栈设置**唯一 ROS_DOMAIN_ID**（如 10），彻底隔离 domain 0 上的外来污染：

```bash
# 在容器内启动 launch 时加上：
export ROS_DOMAIN_ID=10
ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py headless:=true

# cartesian executor 也要同域：
export ROS_DOMAIN_ID=10
ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py
```

隔离后效果：
- ✅ `/joint_states` 只有 UR5e 的 6 个关节
- ✅ `joint_state_broadcaster` / `joint_trajectory_controller` 都 **active**
- ✅ 机械臂按 8 个 waypoint 自动扫描喷涂，perf 日志显示补丁持续创建
- ✅ move_group 不再刷错误

### 补充：独立的 ros2_control_node 崩溃
launch 里的独立 `ros2_control_node` 会 SIGABRT
（`pluginlib::LibraryLoadException`，因为它和 gz_ros2_control 插件内置的
controller_manager 重复加载同一硬件）。**gz 插件内置的 controller_manager 已够用**，
两个控制器由它加载成功。可从 launch 移除独立 ros2_control_node 消除噪音（不影响功能）。

---

## 八、可视化方法

当前 launch 默认 `headless:=true`（gz 只有 server 无 GUI）。要可视化：

```bash
# 1) Gazebo GUI —— 连接到已运行的 server（-g = 仅 GUI）
docker exec spray_paint_stack bash -c 'export DISPLAY=:0 && gz sim -g'

# 2) RViz —— 看 MoveIt 机器人模型 / TF / 规划场景
docker exec spray_paint_stack bash -c \
  ". /opt/ros/humble/setup.bash && . /ws/install/setup.bash && \
   export ROS_DOMAIN_ID=10 DISPLAY=:0 && \
   rviz2 -d /ws/install/gz_spray_painting_plugin_demo/share/gz_spray_painting_plugin_demo/config/moveit.rviz"

# 3) 再跑一次喷涂（让机械臂动给你看）
docker exec spray_paint_stack bash -c \
  ". /opt/ros/humble/setup.bash && . /ws/install/setup.bash && \
   export ROS_DOMAIN_ID=10 && \
   ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py"
```

关键：**所有 ROS 命令都要带 `ROS_DOMAIN_ID=10`**，否则会掉回污染的 domain 0。
