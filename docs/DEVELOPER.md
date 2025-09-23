# 开发指南

本指南介绍如何为轨迹规划系统贡献代码和设置开发环境。

## 开发环境设置

### 系统要求
- Ubuntu 22.04 LTS
- C++20 兼容的编译器 (GCC 10+)
- CMake 3.16+
- Git

### 依赖安装
```bash
# 基础开发工具
sudo apt update
sudo apt install build-essential cmake git

# ROS2 Humble
sudo apt install ros-humble-desktop python3-argcomplete

# MoveIt 2 (源码编译推荐)
mkdir -p ~/moveit2_ws/src && cd ~/moveit2_ws/src
git clone -b humble https://github.com/ros-planning/moveit2.git
vcs import < moveit2/moveit2.repos
cd ~/moveit2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --mixin release

# TracIK (源码编译推荐)
mkdir -p ~/trac_ik_ws/src && cd ~/trac_ik_ws/src
git clone https://github.com/aprotyas/trac_ik.git
cd ~/trac_ik_ws && colcon build
```

### 获取源码
```bash
mkdir -p ~/trajectory_planning_dev_ws/src && cd ~/trajectory_planning_dev_ws/src
git clone https://github.com/Ding-Kaiyue/trajectory-planning.git
cd ~/trajectory_planning_dev_ws
source ~/moveit2_ws/install/setup.bash
source ~/trac_ik_ws/install/setup.bash
```

## 项目结构

```
trajectory_planning/
├── src/
│   ├── trajectory_planning_v3/      # 核心轨迹规划系统
│   │   ├── include/                 # 头文件 (DDD分层架构)
│   │   ├── src/                     # 源文件实现
│   │   ├── launch/                  # 启动文件
│   │   └── nodes/                   # ROS2节点实现
│   ├── robot_interfaces/            # 自定义消息定义
│   ├── robot_config/                # 机器人配置 (arm620, arm380)
│   ├── robot_description/           # URDF/XACRO文件
│   ├── robot_bringup/               # 系统启动配置
│   └── trac_ik_kinematics_plugin/   # TracIK运动学插件
└── docs/                            # 文档
```

## 构建项目

### 快速构建
```bash
cd ~/trajectory_planning_dev_ws
colcon build --symlink-install
source install/setup.bash
```

### 构建选项
```bash
# Debug构建
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Release构建
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 仅构建特定包
colcon build --packages-select trajectory_planning_v3
```

## 运行测试

```bash
# 运行测试
colcon test --packages-select trajectory_planning_v3

# 查看测试结果
colcon test-result --verbose

# 启动系统测试
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=trajectory_planning
```

## DDD架构概览

### 分层结构
- **Domain Layer**: 实体、值对象、领域服务 (不依赖外部框架)
- **Application Layer**: 应用服务、编排器 (协调领域对象)
- **Infrastructure Layer**: MoveIt集成、TracIK适配、规划策略

### 命名空间组织
```cpp
trajectory_planning::domain::entities     // RobotState, Trajectory
trajectory_planning::application::services // MotionPlanningService
trajectory_planning::infrastructure::planning // 策略实现
```

### 节点架构
- **统一节点**: `trajectory_planning` (生产推荐，支持所有策略)
- **分布式节点**: `movej`, `movel`, `movec`, `joint_constrained` (开发测试)

## 添加新功能

### 1. 添加新的规划策略
```bash
# 创建策略文件
touch include/trajectory_planning_v3/infrastructure/planning/strategies/new_strategy.hpp
touch src/infrastructure/planning/strategies/new_strategy.cpp
```

```cpp
// 策略接口模式
class NewPlanningStrategy {
public:
    explicit NewPlanningStrategy(integration::MoveItAdapter& moveit_adapter);
    domain::entities::Trajectory plan(const YourRequestType& request);
};
```

### 2. 扩展应用服务
```cpp
// 在MotionPlanningService中添加
PlanningResult planNewMotion(const YourRequestType& request);
```

### 3. 添加自定义消息
```bash
# 在robot_interfaces/msg/中添加
touch src/robot_interfaces/msg/NewRequest.msg
```

## 代码规范

### 命名约定
- **类名**: PascalCase (`MotionPlanningService`)
- **函数名**: camelCase (`planJointMotion`)
- **变量名**: snake_case (`joint_positions_`)
- **私有成员**: 以下划线结尾 (`moveit_adapter_`)

### 文件组织
- **头文件**: 只包含声明，不包含实现
- **源文件**: 对应头文件的完整实现
- **测试文件**: 对应功能的单元测试

### 注释规范
```cpp
/**
 * @brief 函数功能简要描述
 * @param goal 目标参数描述
 * @return PlanningResult 返回值描述
 */
PlanningResult planJointMotion(const sensor_msgs::msg::JointState& goal);
```

## 提交代码

### 提交前检查
```bash
# 运行测试
colcon test --packages-select trajectory_planning_v3

# 代码格式检查
find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format --dry-run

# 构建检查
colcon build --packages-select trajectory_planning_v3
```

### 提交规范
```bash
git commit -m "feat: add new planning strategy

- Implement XYZ planning algorithm
- Add comprehensive unit tests
- Update documentation"
```

### 提交类型
- `feat`: 新功能
- `fix`: 修复bug
- `docs`: 文档更新
- `refactor`: 重构
- `test`: 测试相关

## 调试指南

### 常见问题
1. **MoveIt初始化失败**: 检查`robot_description`参数和URDF有效性
2. **规划失败**: 检查目标可达性和关节限制
3. **TracIK问题**: 验证TracIK插件配置

### 调试工具
```bash
# 检查节点状态
ros2 node info /trajectory_planning

# 监控话题
ros2 topic echo /movej_goals

# 检查参数
ros2 param get /trajectory_planning move_group

# 性能分析
ros2 run rqt_top rqt_top
```

## 发布流程

### 版本更新
- 更新`package.xml`版本号
- 更新`CHANGELOG.md`
- 创建发布标签

### 发布检查清单
- [ ] 所有测试通过
- [ ] 文档更新
- [ ] 版本号更新
- [ ] 发布说明

## 联系方式

- **GitHub Issues**: [问题报告](https://github.com/Ding-Kaiyue/trajectory-planning/issues)
- **Email**: kaiyue.ding@raysense.com

---

**注意**: 贡献代码前请确保已阅读并同意项目的MIT许可证。