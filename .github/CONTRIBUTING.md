# 贡献指南

感谢您对轨迹规划系统的关注！我们欢迎各种形式的贡献。

## 贡献方式

### 报告问题
- 使用GitHub Issues报告Bug
- 提交功能请求和改进建议
- 报告文档错误或不清楚的地方

### 代码贡献
- 修复Bug
- 添加新功能
- 改进性能
- 完善文档

## 开发流程

### 1. 环境准备
```bash
# 参考开发者指南设置环境
# docs/DEVELOPER.md
```

### 2. Fork和分支
```bash
# Fork项目到你的账户
# 克隆到本地
git clone https://github.com/Ding-Kaiyue/trajectory-planning.git
cd trajectory-planning

# 创建功能分支
git checkout -b feature/your-feature-name
```

### 3. 开发
- 遵循项目代码规范
- 添加必要的测试
- 确保所有测试通过
- 更新相关文档

### 4. 提交
```bash
# 提交格式
git commit -m "feat: add new planning strategy

- Implement XYZ planning algorithm
- Add comprehensive unit tests
- Update documentation"
```

### 5. Pull Request
- 推送分支到你的Fork
- 创建Pull Request
- 填写PR描述模板
- 等待代码审查

## 代码规范

### 命名约定
- **类名**: PascalCase (`MotionPlanningService`)
- **函数名**: camelCase (`planJointMotion`)
- **变量名**: snake_case (`joint_positions_`)
- **文件名**: snake_case (`motion_planning_service.cpp`)

### 代码风格
```cpp
// 头文件保护
#pragma once

// 包含顺序：系统头文件 -> 第三方库 -> 项目头文件
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

// 命名空间
namespace trajectory_planning::application::services {

class MotionPlanningService {
public:
    /**
     * @brief 函数简要描述
     * @param goal 参数描述
     * @return 返回值描述
     */
    PlanningResult planJointMotion(const sensor_msgs::msg::JointState& goal);

private:
    // 私有成员以下划线结尾
    std::shared_ptr<MoveItAdapter> moveit_adapter_;
};

}  // namespace trajectory_planning::application::services
```

### 提交类型
- `feat`: 新功能
- `fix`: Bug修复
- `docs`: 文档更新
- `style`: 代码格式
- `refactor`: 重构
- `test`: 测试相关
- `chore`: 构建工具等

## 测试要求

### 单元测试
```cpp
#include <gtest/gtest.h>

TEST(MotionPlanningServiceTest, ShouldPlanJointMotion) {
    // Given
    auto service = createMotionPlanningService();
    sensor_msgs::msg::JointState goal;
    goal.position = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};

    // When
    auto result = service->planJointMotion(goal);

    // Then
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.trajectory.isValid());
}
```

### 集成测试
```bash
# 运行所有测试
colcon test --packages-select trajectory_planning_v3

# 启动系统测试
ros2 launch robot_bringup robot_real.launch.py planning_node_type:=trajectory_planning
```

## 文档要求

### 代码注释
- 公有接口必须有Doxygen注释
- 复杂算法需要详细说明
- 关键决策点添加注释

### 文档更新
- 新功能需要更新相关文档
- API变更需要更新API文档
- 重要变更需要更新CHANGELOG.md

## 审查流程

### 自动检查
- 代码格式检查
- 单元测试
- 构建测试
- 静态分析

### 人工审查
- 代码质量
- 架构设计
- 测试覆盖
- 文档完整性

## 发布流程

发布由项目维护者负责：

1. 版本号更新
2. CHANGELOG更新
3. 测试验证
4. 创建Release
5. 发布通知

## 社区行为准则

- 尊重他人，友善交流
- 专注技术讨论
- 欢迎新人参与
- 耐心回答问题

## 获取帮助

- **文档**: 查看[docs/](../docs/)目录
- **Issues**: 搜索现有问题或创建新Issue
- **邮件**: kaiyue.ding@raysense.com

## 许可证

贡献的代码将采用项目的MIT许可证。提交代码即表示同意此许可证条款。

---

**感谢您的贡献！** 🎉