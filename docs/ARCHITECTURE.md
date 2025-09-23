# 架构设计

基于领域驱动设计(DDD)的现代化轨迹规划系统，为工业机器人提供高性能、可扩展的运动规划解决方案。

## 架构概览

### DDD分层架构

```
┌─────────────────────────────────────────┐
│         Presentation Layer              │  ROS2节点、话题订阅
├─────────────────────────────────────────┤
│         Application Layer               │  MotionPlanningService
├─────────────────────────────────────────┤
│           Domain Layer                  │  Trajectory, RobotState
├─────────────────────────────────────────┤
│       Infrastructure Layer              │  MoveIt、TracIK适配器
└─────────────────────────────────────────┘
```

### 设计原则

1. **依赖倒置**: 核心业务逻辑不依赖外部框架
2. **单一职责**: 每个组件职责明确，边界清晰
3. **策略模式**: 可插拔的规划算法实现
4. **事件驱动**: 松耦合的组件通信

## 核心组件

### Domain Layer (领域层)

**职责**: 封装核心业务逻辑，不依赖任何技术实现

#### 实体 (Entities)
- **Trajectory**: 轨迹数据和状态管理
- **RobotState**: 机器人状态表示
- **TrajectoryPoint**: 单个轨迹点定义

#### 值对象 (Value Objects)
- **JointPosition**: 关节位置数据
- **Pose**: 末端执行器位姿
- **Duration**: 时间表示

#### 领域服务 (Domain Services)
- **KinematicsService**: 运动学计算
- **TrajectoryValidationService**: 轨迹有效性验证

### Application Layer (应用层)

**职责**: 协调领域对象，实现用例流程

#### 应用服务 (Application Services)
```cpp
class MotionPlanningService {
    // 四种核心规划接口
    PlanningResult planJointMotion(const sensor_msgs::msg::JointState& goal);
    PlanningResult planLinearMotion(const geometry_msgs::msg::Pose& goal);
    PlanningResult planArcMotion(const robot_interfaces::msg::MoveCRequest& request);
    PlanningResult planConstrainedMotion(const robot_interfaces::msg::JointConstrainedRequest& request);
};
```

### Infrastructure Layer (基础设施层)

**职责**: 提供技术实现，对接外部系统

#### 规划策略 (Planning Strategies)
- **MoveJPlanningStrategy**: 关节空间规划
- **MoveLPlanningStrategy**: 直线运动规划
- **MoveCPlanningStrategy**: 圆弧运动规划
- **JointConstrainedPlanningStrategy**: 约束规划

#### 集成适配器 (Integration Adapters)
- **MoveItAdapter**: MoveIt 2框架集成
- **TracIKAdapter**: TracIK运动学求解器集成
- **HardwareAdapter**: 硬件驱动库集成

## 节点架构

### 微服务设计

系统支持两种部署模式：

#### 统一节点模式 (生产推荐)
```
TrajectoryPlanningNode
├── /movej_goals_internal
├── /movel_goals_internal
├── /movec_goals_internal
└── /joint_constrained_goals_internal
```

#### 分布式节点模式 (开发测试)
```
MovejNode ──────── /movej_goals
MovelNode ──────── /movel_goals
MovecNode ──────── /movec_goals
JointConstrainedNode ── /joint_constrained_goals
```

### 消息流

```
ROS2话题 → 节点订阅 → 应用服务 → 规划策略 → MoveIt → 硬件执行
```

## 规划策略架构

### 策略模式实现

每种规划算法独立实现，支持运行时切换：

```cpp
// 策略接口模式 (无统一基类，保持灵活性)
class MoveJPlanningStrategy {
    domain::entities::Trajectory plan(const sensor_msgs::msg::JointState& goal);
};

class MoveLPlanningStrategy {
    enum class PlanningType { JOINT, CARTESIAN, INTELLIGENT };
    domain::entities::Trajectory plan(const geometry_msgs::msg::Pose& goal, PlanningType type);
};
```

### 规划流程

1. **请求验证**: 参数合法性检查
2. **运动学预处理**: IK/FK计算
3. **路径规划**: MoveIt规划算法
4. **轨迹优化**: 速度、加速度平滑
5. **安全检查**: 碰撞检测、约束验证

## 数据流架构

### 输入数据流
```
ROS2消息 → 消息适配器 → 领域对象 → 应用服务
```

### 处理数据流
```
应用服务 → 规划策略 → MoveIt适配器 → 运动学求解
```

### 输出数据流
```
轨迹结果 → 硬件适配器 → 控制器 → 机器人执行
```

## 集成架构

### MoveIt 2集成

- **Planning Scene**: 场景管理和碰撞检测
- **Move Group**: 运动规划接口
- **Kinematics Plugin**: TracIK或KDL求解器

### Hardware Driver集成

与[Hardware Driver Library](https://github.com/Ding-Kaiyue/hardware-driver)深度集成：

- **事件驱动**: 实时状态监控
- **高性能**: 微秒级控制延迟
- **多模式**: 位置、速度、力矩控制

### Trajectory Interpolator集成

集成[Trajectory Interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator)实现：

- **样条插值**: 高精度轨迹插值
- **实时性能**: 支持在线轨迹生成
- **约束检查**: 速度、加速度限制

## 安全架构

### 多层安全保护

1. **输入验证**: 参数边界检查
2. **运动学约束**: 关节限位验证
3. **碰撞检测**: MoveIt场景碰撞检查
4. **轨迹验证**: 速度、加速度约束
5. **硬件保护**: 驱动层安全机制

### 故障恢复

- **规划失败**: 自动重试和降级策略
- **执行中断**: 安全停止和状态恢复
- **通信异常**: 超时检测和错误处理

## 性能架构

### 优化策略

1. **并发设计**: 多线程规划和执行
2. **内存优化**: 对象池和缓存机制
3. **算法优化**: 高效的路径搜索
4. **硬件加速**: TracIK求解器优化

### 性能指标

- **规划延迟**: < 100ms (典型场景)
- **执行精度**: ±0.1mm 位置精度
- **内存占用**: < 200MB
- **CPU利用率**: < 30% (单核)

## 扩展架构

### 水平扩展

- **多机器人**: 支持多臂协调规划
- **分布式**: 云端规划服务
- **插件化**: 自定义规划算法

### 垂直扩展

- **新硬件**: 支持不同机器人型号
- **新算法**: 添加先进规划策略
- **新功能**: 实时避障、力控等

---

**设计原则**: 保持架构清晰、组件解耦、易于测试和维护。