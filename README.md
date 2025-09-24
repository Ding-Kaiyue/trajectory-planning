# Trajectory Planning

[![ROS Version](https://img.shields.io/badge/ROS-ROS2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()

一个**现代化的机械臂轨迹规划系统**，基于领域驱动设计(DDD)架构，为工业机器人提供高性能、线程安全的运动规划和轨迹执行。

> 📖 **完整文档**: 查看 [docs/README.md](docs/README.md) 获取详细的文档中心，包括架构设计、开发指南、代码规范等。

## 🚀 特性

- **四种核心规划模式**: MoveJ(关节空间)、MoveL(直线)、MoveC(圆弧)、JointConstrained(关节约束规划)
- **领域驱动设计**: 清晰的架构分层，应用服务模式，易于维护和扩展
- **高性能集成**: 基于 MoveIt2 + TracIK，快速轨迹生成，支持开环轨迹规划
- **双架构支持**: 统一节点(生产环境) + 分布式节点(开发测试)
- **可靠传输**: QoS配置为Reliable，确保关键运动指令不丢失
- **硬件集成**: 深度对接硬件轨迹控制器，支持真实机器人控制

## 📦 安装

### 依赖要求

#### MoveIt 2 (源码安装，推荐)
```bash
# 创建MoveIt工作空间
mkdir -p ~/moveit2_ws/src
cd ~/moveit2_ws/src

# 克隆MoveIt 2
git clone -b humble https://github.com/ros-planning/moveit2.git
vcs import < moveit2/moveit2.repos

# 安装依赖并编译
cd ~/moveit2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --mixin release
source install/setup.bash
```

#### TracIK (源码安装，推荐)
```bash
# 创建独立的TracIK工作空间
mkdir -p ~/trac_ik_ws/src
cd ~/trac_ik_ws/src

# 克隆TracIK
git clone https://github.com/aprotyas/trac_ik.git

# 编译
cd ~/trac_ik_ws
colcon build
source install/setup.bash
```

> [!NOTE]
> **说明**: 本项目在**MoveIt2和TracIK源码环境下**测试通过。使用TracIK可获得更高性能的逆运动学求解。

#### 可选：使用KDL求解器（无需安装TracIK）
使用MoveIt默认的KDL求解器。需要修改机器人配置中的 `kinematics.yaml` 文件：

```yaml
# 将 kinematics.yaml 中的 kinematics_solver 改为：
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```
 
### 项目编译（推荐）
```bash
# 创建工作空间
mkdir -p ~/trajectory_planning_ws/src
cd ~/trajectory_planning_ws/src

# 克隆项目
git clone https://github.com/Ding-Kaiyue/trajectory-planning.git

# 编译
cd ~/trajectory_planning_ws
colcon build --symlink-install
source install/setup.bash
```

### 硬件集成

> [!TIP]
> **完整机器人系统**: 推荐配合使用 **[Hardware Driver Library](https://github.com/Ding-Kaiyue/hardware-driver)**，获得从轨迹规划到硬件执行的完整解决方案。

> [!CAUTION]
> **关节模组模式切换**: 当前机器人关节模组在**位置模式**和**速度模式**之间切换时必须先失能再使能。当前系统使用位置模式，后续版本添加其他功能时必须注意这一点。

## 🚀 快速开始

### 统一节点模式（生产推荐）
```bash
# 启动统一轨迹规划节点 - 支持所有四种规划模式
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=trajectory_planning
```

### 分布式节点模式（开发测试）
```bash
# 启动特定功能节点
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=movej
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=movel
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=movec
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=joint_constrained
```

### 基本使用

#### 关节空间运动
```bash
ros2 topic pub --once /movej_goals sensor_msgs/msg/JointState "
{position: [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]}"
```

#### 直线运动
```bash
ros2 topic pub --once /movel_goals geometry_msgs/msg/Pose "
{position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}}"
```

#### 圆弧运动
```bash
ros2 topic pub --once /movec_goals robot_interfaces/msg/MoveCRequest "
{route_type: 0, waypoints: [
  {position: {x: 0.4, y: 0.2, z: 0.4}, orientation: {w: 1.0}},
  {position: {x: 0.3, y: 0.3, z: 0.3}, orientation: {w: 1.0}}
]}"
```

#### 关节约束规划
```bash
ros2 topic pub --once /joint_constrained_goals robot_interfaces/msg/JointConstrainedRequest "
{goal_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}},
 joint_constraints: [{joint_index: 1, type: 0, fixed_value: 0.0, weight: 1.0, is_hard: true}],
 max_attempts: 10}"
```

## 📋 API 参考

### 轨迹规划
```cpp
// 应用服务层API
auto motion_service = std::make_shared<MotionPlanningService>(
    movej_strategy, movel_strategy, movec_strategy, joint_constrained_strategy,
    moveit_adapter, logger);

// 关节空间规划
auto result1 = motion_service->planJointMotion(joint_state);

// 直线运动规划
auto result2 = motion_service->planLinearMotion(pose_goal);

// 圆弧运动规划
auto result3 = motion_service->planArcMotion(movec_request);

// 关节约束规划
auto result4 = motion_service->planConstrainedMotion(constraint_request);
```

### 轨迹执行
```cpp
auto execution_service = std::make_shared<TrajectoryExecutionService>(
    trajectory_executor, logger);

auto result = execution_service->execute(trajectory);
```

### 配置参数
```yaml
move_group: "arm"                        # MoveIt规划组
planner_id: "RRTConnectkConfigDefault"   # 规划算法
planning_time: 5.0                       # 规划超时(秒)
velocity_scaling: 0.2                    # 速度缩放因子
acceleration_scaling: 0.2                # 加速度缩放因子
goal_tolerance: 0.001                    # 目标容差
```

## 🧪 性能

> [!IMPORTANT]
> **测试环境**: 以下数据基于**工业级ARM机械臂**测试环境获得。

- **规划速度**: < 100ms (典型场景)
- **轨迹精度**: ±0.1mm 位置精度，±0.1° 姿态精度
- **轨迹采样**: 10Hz (100ms间隔) 轨迹点生成
- **规划成功率**: > 95% (无障碍环境)
- **内存占用**: < 200MB

## 🛠️ 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- MoveIt 2 (源码安装)
- TracIK (源码安装)
- GCC 10+ (C++20)
- 支持的机器人：ARM380, ARM620

## 🔍 故障排除

### 节点启动失败
```bash
# 检查MoveIt配置
ros2 param get /move_group robot_description

# 验证参数加载
ros2 param list | grep move_group
```

### 奇点问题

> [!WARNING]
> **奇点自动分段**: 当机器人接近奇点时，轨迹会自动分段以保证安全性，这是**正常的保护机制**。

解决方案：
- 调整起始位置避开奇点区域
- 使用约束规划限制关节范围（可能降低成功率）
- 选择更安全的运动路径


## 📄 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件

## 📞 联系方式

- **GitHub**: [Issues](https://github.com/Ding-Kaiyue/trajectory-planning/issues)
- **Email**: kaiyue.ding@raysense.com

---

⭐ **如果这个项目对你有帮助，请给我们一个星标！**