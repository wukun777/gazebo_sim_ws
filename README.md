# Gazebo 仿真工作空间

本工作空间用于无人机挂载机械臂的算法仿真，使用 ROS 2 和 Gazebo Classic
直接验证无人机控制、机械臂控制以及后续的空中机械臂耦合算法。

本仿真不涉及 PX4、MAVROS、PX4 SITL、Fast-LIO 或 Livox 驱动。当前控制器
直接读取 Gazebo 状态，并向 `roarm_quad` 模型发布旋翼力指令。

ubuntu 22.04 humble

## 一、完整目录结构

```text
gazebo_sim_ws/
├── src/                                      # ROS 2 源码和仿真资源
│   ├── aerial_manipulator_description/       # 空中机械臂组合模型包
│   │   ├── models/
│   │   │   └── roarm_quad/
│   │   │       ├── model.sdf                 # 无人机+机械臂完整 SDF 模型
│   │   │       ├── model.config               # Gazebo 模型描述
│   │   │       └── meshes/                    # 模型网格
│   │   │           ├── uav/                   # 无人机机身网格
│   │   │           ├── arm/                   # 机械臂和夹爪网格
│   │   │           └── rotors/                # 四个旋翼网格
│   │   ├── CMakeLists.txt                     # 安装模型资源
│   │   └── package.xml
│   │
│   ├── gazebo_worlds/                         # Gazebo 世界包
│   │   ├── worlds/
│   │   │   └── simple_room.world              # 当前基础测试世界
│   │   ├── CMakeLists.txt                     # 安装世界文件
│   │   └── package.xml
│   │
│   ├── gazebo_sim_launch/                     # 仿真启动包
│   │   ├── launch/
│   │   │   └── hover_sim.launch.py            # 启动 Gazebo、生成模型和悬停控制器
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── uav_control/                           # 无人机控制功能域目录
│   │   ├── README.md
│   │   └── hover_control/                     # 当前已有的 ROS 2 控制包
│   │       ├── hover_control/
│   │       │   ├── hover_node.py              # 四旋翼直接悬停控制器
│   │       │   └── __init__.py
│   │       ├── resource/hover_control          # ROS 2 Python 包资源标记
│   │       ├── test/                           # Python 代码检查测试
│   │       ├── package.xml
│   │       ├── setup.py
│   │       └── setup.cfg
│   │
│   ├── arm_control/                            # 机械臂控制功能域
│   │   └── README.md                            # 当前仅记录规划，没有算法源码
│   │
│   ├── aerial_manipulator_control/             # 空中机械臂耦合控制功能域
│   │   └── README.md                            # 当前仅记录规划，没有算法源码
│   │
│   └── aerial_manipulator_interfaces/          # 跨包接口功能域
│       └── README.md                            # 未来放置 msg/srv/action 接口
│
├── build/                                      # colcon 编译中间文件，由 Ubuntu 生成
├── install/                                    # colcon 安装结果，由 Ubuntu 生成
├── log/                                        # colcon 编译日志，由 Ubuntu 生成
├── README.md                                   # 本工作空间说明
├── .gitignore                                  # 忽略编译产物和 Python 缓存
└── .stfolder/                                  # 同步工具元数据
```

同步工具产生的 `*.sync-conflict-*` 文件是冲突副本，不属于正式源码包，不能
替代同目录下的正式文件。确认同步内容后再处理这些副本。

## 二、各部分作用

### 1. `aerial_manipulator_description`

负责完整的 `roarm_quad` 仿真模型，包括：

- 无人机机身、起落架和四个旋翼
- 机械臂基座、三段连杆和夹爪
- 各 link 的质量、惯量、关节和碰撞参数
- IMU 传感器
- 旋翼力插件的 SDF 配置

模型网格按照无人机、机械臂和旋翼分类，但 Gazebo 模型名称统一保持为
`roarm_quad`。

### 2. `gazebo_worlds`

保存可被多个仿真任务复用的世界文件。当前的 `simple_room.world` 是基础测试
场景，包含地面和简单障碍物。后续可以在这里增加抓取场景、障碍场景和任务场景。

### 3. `gazebo_sim_launch`

负责组合仿真流程：

1. 启动 Gazebo Classic。
2. 加载 `gazebo_worlds` 中的世界。
3. 设置 `aerial_manipulator_description` 的模型搜索路径。
4. 生成 `roarm_quad` 模型。
5. 启动 IMU 的静态 TF。
6. 启动 `hover_control` 悬停控制器。

### 4. `uav_control/hover_control`

这是当前唯一已有的控制算法包。它不经过 PX4，直接完成：

- Gazebo `LinkStates` 状态读取
- 高度 PID 控制
- 水平位置保持
- 姿态控制
- 四旋翼推力分配
- 向四个旋翼发布 `Wrench` 力指令
- 调用 Gazebo 关节服务驱动旋翼视觉转动

### 5. `arm_control`

预留机械臂关节控制、轨迹规划、运动学和末端执行器算法。目前只有 README，
没有虚构的控制器文件。

### 6. `aerial_manipulator_control`

预留无人机与机械臂的耦合控制算法，例如机械臂运动引起的姿态扰动补偿、末端
位姿控制和空中操作任务控制。目前只有 README。

### 7. `aerial_manipulator_interfaces`

预留未来跨功能包通信使用的 ROS 2 `msg`、`srv` 和 `action` 接口。目前没有
具体通信协议，因此暂不创建接口文件。

## 三、统一模型名称

完整仿真机器人统一使用 `roarm_quad`：

- 模型目录：`models/roarm_quad/`
- SDF 模型名：`roarm_quad`
- 网格 URI：`model://roarm_quad/...`
- Gazebo 实体名：`roarm_quad`
- 控制器参数：`model_name=roarm_quad`
- 旋翼话题：`/roarm_quad/rotor_*/cmd_force`

## 四、编译结果和同步关系

`build/`、`install/` 和 `log/` 是 Ubuntu 虚拟机通过 `colcon` 生成的结果，
会同步回当前电脑。它们不是源码，不能手工修改；修改源码后应在虚拟机重新
编译，再通过同步工具更新回来。

当前 Gazebo 旋翼力插件已经可以在仿真环境中找到并正常使用。插件由模型的
SDF 加载，不作为当前工作空间中的算法源码包维护。

## 五、Ubuntu 编译和启动

```bash
source /opt/ros/humble/setup.bash
cd ~/gazebo_sim_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch gazebo_sim_launch hover_sim.launch.py
```
