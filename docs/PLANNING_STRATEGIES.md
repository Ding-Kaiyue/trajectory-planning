# 规划策略详解

轨迹规划系统提供四种核心规划策略，涵盖工业机器人的主要运动需求。

## 策略概览

| 策略 | 用途 | 空间 | 特点 |
|------|------|------|------|
| **MoveJ** | 关节运动 | 关节空间 | 快速、无碰撞保证 |
| **MoveL** | 直线运动 | 笛卡尔空间 | 精确直线路径 |
| **MoveC** | 圆弧运动 | 笛卡尔空间 | 平滑曲线轨迹 |
| **JointConstrained** | 关节约束规划 | 混合空间 | 满足特定约束 |

## MoveJ (关节空间规划)

### 算法原理
在关节空间中规划从当前位置到目标关节角度的轨迹，通过插值生成平滑的关节运动。

### 适用场景
- **自由空间运动**: 快速移动到目标位置
- **避障运动**: 绕过工作空间障碍物
- **初始化**: 机器人复位和准备动作

### 技术实现
```cpp
class MoveJPlanningStrategy {
public:
    domain::entities::Trajectory plan(const domain::value_objects::JointPosition& goal);
private:
    // 关节空间插值算法
    void interpolateJointSpace(const std::vector<double>& start,
                              const std::vector<double>& goal);
};
```

### 使用示例
```bash
# 移动到零位
ros2 topic pub --once /movej_goals sensor_msgs/msg/JointState "
{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 移动到工作位置
ros2 topic pub --once /movej_goals sensor_msgs/msg/JointState "
{position: [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]}"
```

### 优势
- **速度快**: 直接关节空间规划，计算效率高
- **成功率高**: 避免笛卡尔空间奇点问题
- **路径可预测**: 关节运动轨迹确定

### 注意事项
- 末端轨迹非直线，不适合精密装配
- 需要预先检查中间路径的安全性

## MoveL (直线运动规划)

### 算法原理
在笛卡尔空间规划直线轨迹，确保末端执行器沿直线路径运动到目标位置。

### 规划模式
- **CARTESIAN**: 笛卡尔路径规划 (精确直线)
- **JOINT**: 关节空间规划 (近似直线)
- **INTELLIGENT**: 智能选择 (优先笛卡尔，失败则关节)

### 适用场景
- **精密装配**: 插拔、装配操作
- **涂胶/焊接**: 需要精确路径控制
- **检测**: 传感器沿直线扫描

### 技术实现
```cpp
class MoveLPlanningStrategy {
public:
    enum class PlanningType { JOINT, CARTESIAN, INTELLIGENT };
    domain::entities::Trajectory plan(const geometry_msgs::msg::Pose& goal,
                                      PlanningType type = PlanningType::INTELLIGENT);
};
```

### 使用示例
```bash
# 移动到指定位置
ros2 topic pub --once /movel_goals geometry_msgs/msg/Pose "
{position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}}"

# 装配任务
ros2 topic pub --once /movel_goals geometry_msgs/msg/Pose "
{position: {x: 0.4, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}"
```

### 优势
- **路径精确**: 严格按照直线轨迹运动
- **适用面广**: 满足大多数工业应用需求
- **智能切换**: 自动选择最优规划方式

### 局限性
- **奇点敏感**: 可能遇到运动学奇点
- **规划复杂**: 笛卡尔规划计算量大

## MoveC (圆弧运动规划)

### 算法原理
在笛卡尔空间规划圆弧或曲线轨迹，支持多种路径类型的平滑运动。

### 路径类型
1. **ARC**: 两点圆弧路径
2. **BEZIER**: 贝塞尔曲线路径
3. **CIRCLE**: 完整圆形轨迹
4. **CIRCLE3PT**: 三点确定圆弧

### 适用场景
- **焊接**: 圆弧焊接轨迹
- **喷涂**: 曲面喷涂路径
- **搬运**: 平滑搬运轨迹

### 技术实现
```cpp
class MoveCPlanningStrategy {
public:
    domain::entities::Trajectory plan(const robot_interfaces::msg::MoveCRequest& request);
private:
    Trajectory planArcTrajectory(const MoveCRequest& request);
    Trajectory planBezierTrajectory(const MoveCRequest& request);
    Trajectory planCircleTrajectory(const MoveCRequest& request);
};
```

### 使用示例

#### 圆弧路径
```bash
ros2 topic pub --once /movec_goals robot_interfaces/msg/MoveCRequest "
{route_type: 0, route_name: 'welding_arc',
 waypoints: [
   {position: {x: 0.4, y: 0.2, z: 0.4}, orientation: {w: 1.0}},
   {position: {x: 0.3, y: 0.3, z: 0.3}, orientation: {w: 1.0}}
 ]}"
```

#### 完整圆形
```bash
ros2 topic pub --once /movec_goals robot_interfaces/msg/MoveCRequest "
{route_type: 2, route_name: 'circle_path',
 waypoints: [
   {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {w: 1.0}},
   {position: {x: 0.3, y: 0.1, z: 0.3}, orientation: {w: 1.0}}
 ]}"
```

### 算法特点
- **路径平滑**: 连续的速度和加速度
- **类型丰富**: 支持多种曲线类型
- **参数化**: 灵活的路径参数控制

## JointConstrained (约束规划)

### 算法原理
在满足特定关节约束的条件下，规划到达目标位姿的轨迹，支持硬约束和软约束。

### 约束类型
- **FIXED**: 固定关节约束 (关节保持特定角度)
- **RANGE**: 范围约束 (关节在指定范围内)

### 适用场景
- **受限空间**: 狭窄空间中的规划
- **避免奇点**: 通过约束避开奇异位置
- **特殊姿态**: 保持特定关节配置

### 技术实现
```cpp
class JointConstrainedPlanningStrategy {
public:
    domain::entities::Trajectory plan(const robot_interfaces::msg::JointConstrainedRequest& request);
private:
    bool satisfiesConstraints(const std::vector<double>& joint_values);
    void applyOptimization(const JointConstrainedRequest& request);
};
```

### 约束定义
```msg
# JointConstraint.msg
uint8 type              # 0=FIXED, 1=RANGE
int32 joint_index       # 关节索引 (0-5)
float64 weight          # 约束权重 (0.0-1.0)
bool is_hard           # 硬约束/软约束
float64 fixed_value    # 固定值 (FIXED类型)
float64 min_value      # 最小值 (RANGE类型)
float64 max_value      # 最大值 (RANGE类型)
```

### 使用示例

#### 固定关节约束
```bash
ros2 topic pub --once /joint_constrained_goals robot_interfaces/msg/JointConstrainedRequest "
{goal_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}},
 planning_type: 2,
 joint_constraints: [
   {type: 0, joint_index: 1, weight: 1.0, is_hard: true, fixed_value: 0.0}
 ],
 max_attempts: 10}"
```

#### 范围约束
```bash
ros2 topic pub --once /joint_constrained_goals robot_interfaces/msg/JointConstrainedRequest "
{goal_pose: {position: {x: 0.4, y: 0.2, z: 0.4}, orientation: {w: 1.0}},
 planning_type: 1,
 joint_constraints: [
   {type: 1, joint_index: 2, weight: 0.8, is_hard: false, min_value: -1.57, max_value: 1.57}
 ],
 max_attempts: 15}"
```

### 优化算法
- **多次尝试**: 通过多次IK求解寻找满足约束的解
- **权重优化**: 软约束通过权重进行优化
- **梯度下降**: 数值优化满足复杂约束

## 策略选择指南

### 场景驱动选择

| 应用场景 | 推荐策略 | 原因 |
|----------|----------|------|
| 快速移动 | MoveJ | 高效、可靠 |
| 精密装配 | MoveL | 路径精确 |
| 焊接/涂胶 | MoveC | 平滑曲线 |
| 狭窄空间 | JointConstrained | 满足约束 |
| 避障运动 | MoveJ | 自由空间规划 |
| 力控任务 | MoveL | 可预测路径 |

### 性能对比

| 策略 | 规划速度 | 成功率 | 路径精度 | 适用性 |
|------|----------|--------|----------|--------|
| MoveJ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| MoveL | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| MoveC | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| JointConstrained | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |

### 组合使用

复杂任务可以组合使用多种策略：

1. **MoveJ** → 快速接近目标区域
2. **MoveL** → 精确到达工作位置
3. **MoveC** → 执行曲线工作轨迹
4. **MoveJ** → 快速撤离到安全位置

## 故障处理

### 常见问题

1. **规划失败**
   - 检查目标可达性
   - 调整规划参数
   - 尝试其他策略

2. **奇点问题**
   - 使用JointConstrained避开奇点
   - 调整目标姿态
   - 分段规划

3. **约束冲突**
   - 放宽软约束权重
   - 增加规划尝试次数
   - 检查约束合理性

### 调试技巧

```bash
# 检查规划器状态
ros2 param get /trajectory_planning planning_time

# 监控规划结果
ros2 topic echo /planning_result

# 调整规划参数
ros2 param set /trajectory_planning velocity_scaling 0.1
```

---

**最佳实践**: 根据具体应用场景选择合适的策略，必要时组合使用多种策略以获得最佳效果。