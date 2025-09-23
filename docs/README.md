# 轨迹规划系统 - 文档中心

欢迎来到轨迹规划系统的文档中心！这里包含了项目的所有技术文档和指南。

## 📚 文档目录

### 🚀 用户指南
- **[README.md](../README.md)** - 项目主页，快速开始指南
- **[RELEASE_NOTES.md](RELEASE_NOTES.md)** - 版本发布说明
- **[CHANGELOG.md](CHANGELOG.md)** - 详细变更历史

### 👨‍💻 开发者文档
- **[DEVELOPER.md](DEVELOPER.md)** - 开发者指南，架构设计，开发流程
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - DDD架构文档和设计理念
- **[PLANNING_STRATEGIES.md](PLANNING_STRATEGIES.md)** - 四种规划策略详解

### 📊 架构图表
- **[系统架构图](diagrams/01_system_architecture.puml)** - DDD分层架构概览
- **[规划流程图](diagrams/02_planning_flow.puml)** - 轨迹规划流程
- **[节点架构图](diagrams/03_node_architecture.puml)** - 统一节点vs分布式节点对比
- **[策略模式图](diagrams/04_strategy_pattern.puml)** - 规划策略模式实现
- **[集成流程图](diagrams/05_integration_flow.puml)** - 系统集成时序流程
- **[执行流程图](diagrams/06_execution_flow.puml)** - 轨迹执行流程

### 🤝 社区指南
- **[CONTRIBUTING.md](../.github/CONTRIBUTING.md)** - 贡献指南
- **[CODE_OF_CONDUCT.md](../.github/CODE_OF_CONDUCT.md)** - 社区行为准则

### 🔒 安全与合规
- **[SECURITY.md](SECURITY.md)** - 安全政策和漏洞报告
- **[LICENSE](../LICENSE)** - MIT开源许可证

## 📖 快速导航

### 新用户
1. 阅读 [README.md](../README.md) 了解项目
2. 查看 [RELEASE_NOTES.md](RELEASE_NOTES.md) 了解最新功能
3. 查看 [src/trajectory_planning_v3/nodes/](../src/trajectory_planning_v3/nodes/) 学习API使用

### 开发者
1. 阅读 [DEVELOPER.md](DEVELOPER.md) 了解架构
2. 查看 [ARCHITECTURE.md](ARCHITECTURE.md) 了解DDD设计理念
3. 浏览 [架构图表](#-架构图表) 理解系统结构
4. 查看 [CONTRIBUTING.md](../.github/CONTRIBUTING.md) 了解贡献流程
5. 参考 [CHANGELOG.md](CHANGELOG.md) 了解版本变更

### 架构学习路径
1. **快速理解**: 查看 [系统架构图](diagrams/01_system_architecture/系统架构图.png)
2. **规划流程**: 学习 [规划流程图](diagrams/02_planning_flow/规划流程图.png)
3. **节点架构**: 了解 [节点架构图](diagrams/03_node_architecture/节点架构图.png)
4. **策略模式**: 深入 [策略模式图](diagrams/04_strategy_pattern/策略模式图.png)
5. **集成流程**: 理解 [集成流程图](diagrams/05_integration_flow/集成流程图.png)
6. **执行流程**: 掌握 [执行流程图](diagrams/06_execution_flow/轨迹执行流程.png)
7. **完整架构**: 阅读 [ARCHITECTURE.md](ARCHITECTURE.md) 获得全面理解

### 规划策略学习
1. **MoveJ策略**: 关节空间规划算法和实现
2. **MoveL策略**: 直线运动规划和笛卡尔路径
3. **MoveC策略**: 圆弧运动和曲线插值
4. **约束策略**: 关节约束规划和优化
5. 详见 [PLANNING_STRATEGIES.md](PLANNING_STRATEGIES.md)

### 维护者
1. 查看 [DEVELOPER.md](DEVELOPER.md) 了解发布流程
2. 参考 [SECURITY.md](SECURITY.md) 处理安全漏洞
3. 更新 [CHANGELOG.md](CHANGELOG.md) 记录变更

## 🔗 外部链接

### 项目相关
- **GitHub仓库**: https://github.com/Ding-Kaiyue/trajectory-planning
- **Issues**: https://github.com/Ding-Kaiyue/trajectory-planning/issues
- **Pull Requests**: https://github.com/Ding-Kaiyue/trajectory-planning/pulls
- **Releases**: https://github.com/Ding-Kaiyue/trajectory-planning/releases

### 相关项目
- **Hardware Driver Library**: https://github.com/Ding-Kaiyue/hardware-driver - 配套硬件驱动库

### 核心依赖
- **MoveIt 2**: https://github.com/ros-planning/moveit2 - 运动规划框架
- **TracIK**: https://github.com/aprotyas/trac_ik - 高性能逆运动学求解器
- **ROS2 Humble**: https://docs.ros.org/en/humble/ - 机器人操作系统

### 技术文档
- **ROS2官方文档**: https://docs.ros.org/en/humble/
- **MoveIt 2教程**: https://moveit.picknik.ai/humble/index.html
- **领域驱动设计**: https://en.wikipedia.org/wiki/Domain-driven_design

---

**💡 提示**: 如果您发现文档有错误或需要改进，欢迎提交Issue或Pull Request！