# 轨迹规划系统 - 版本发布说明

## 📋 概述

轨迹规划系统采用领域驱动设计(DDD)架构，为工业机器人提供高性能、线程安全的运动规划和轨迹执行功能。

## 🚀 v1.0.0 - 首次正式发布 (2025年9月)

### ✨ 新功能

#### 核心规划策略
- **MoveJ (关节空间规划)**: 高效的关节插值算法，支持自由空间运动
- **MoveL (直线运动规划)**: 笛卡尔空间直线路径规划，确保末端轨迹线性
- **MoveC (圆弧运动规划)**: 三点圆弧插值，支持路径和方向两种圆弧模式
- **JointConstrained (关节约束规划)**: 基于优化的约束求解，支持硬/软约束

#### 架构特性
- **领域驱动设计**: 清晰的分层架构，应用服务模式
- **策略模式**: 可扩展的规划策略框架
- **双节点架构**: 统一节点(生产) + 分布式节点(开发)
- **线程安全**: 完整的并发安全保障

#### 技术集成
- **MoveIt 2集成**: 基于MoveIt2框架的运动规划
- **TracIK求解器**: 高性能逆运动学求解，提升规划成功率
- **QoS可靠传输**: Reliable QoS确保关键指令传输
- **硬件深度集成**: 直接对接轨迹控制器

### 🛡️ 安全特性
- **奇点保护**: 自动检测并处理机器人奇点，轨迹分段保护
- **碰撞检测**: 集成MoveIt碰撞检测，确保运动安全
- **约束验证**: 关节限位、速度、加速度约束验证
- **故障恢复**: 完善的错误处理和恢复机制

### 📊 性能指标
- **规划速度**: < 100ms (典型工况)
- **轨迹精度**: ±0.1mm 位置精度，±0.1° 姿态精度
- **采样频率**: 10Hz 轨迹点生成
- **成功率**: > 95% (无障碍环境)
- **内存占用**: < 200MB

### 🤖 支持的机器人
- **ARM380**: 6轴教育机械臂
- **ARM620**: 6轴仿生机械臂
- **通用6轴机械臂**: 符合标准DH参数的机械臂

### 💻 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- MoveIt 2 (源码编译推荐)
- TracIK (源码编译推荐)
- GCC 10+ (支持C++20)

## 🔄 版本兼容性

### v1.0.0
- **ROS2 Humble**: 完全支持
- **MoveIt 2**: Humble分支
- **TracIK**: 最新主分支
- **向后兼容**: 首个版本，无兼容性问题

## 📦 安装指南

### 快速安装
```bash
# 创建工作空间
mkdir -p ~/trajectory_planning_ws/src
cd ~/trajectory_planning_ws/src

# 克隆项目
git clone https://github.com/Ding-Kaiyue/trajectory-planning.git

# 编译安装
cd ~/trajectory_planning_ws
colcon build --symlink-install
source install/setup.bash
```

### 完整环境搭建
详见 [README.md](../README.md) 中的安装说明，包含MoveIt2和TracIK的源码编译。

## 🐛 已知问题

### v1.0.0 已知限制
1. **关节模组模式切换**: 位置/速度模式切换需要失能-使能操作
2. **奇点区域**: 接近奇点时轨迹会自动分段，属于保护机制
3. **规划超时**: 复杂场景下可能需要调整规划时间参数

### 解决方案
- 关节模式切换将在后续版本优化
- 奇点处理已经是最佳实践
- 规划参数可通过配置文件调整

## 🚀 使用入门

### 启动系统
```bash
# 统一节点模式（推荐）
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=trajectory_planning
```

### 基本测试
```bash
# 关节空间运动测试
ros2 topic pub --once /movej_goals sensor_msgs/msg/JointState \
  "{position: [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]}"
```

## 📞 技术支持

### 获取帮助
- **GitHub Issues**: [问题报告](https://github.com/Ding-Kaiyue/trajectory-planning/issues)
- **文档中心**: [docs/README.md](README.md)
- **邮件支持**: kaiyue.ding@raysense.com

### 贡献指南
欢迎提交Bug报告、功能请求和代码贡献。详见 [CONTRIBUTING.md](../.github/CONTRIBUTING.md)。

## 🔮 路线图

### v1.1.0 (计划中)
- 速度模式支持
- 更多机器人型号支持

### v1.2.0 (规划中)
- 动态避障算法
- 多机器人协调
- 云端轨迹规划

---

**📝 注意**: 本文档随版本更新，请关注最新发布信息。

**⭐ 喜欢这个项目？请给我们一个星标！**