# 代码风格指南

本文档基于项目现有代码总结出的代码风格规范，旨在保持代码一致性和可读性。

## 1. C++ 命名规范

### 1.1 类名（Class Names）
使用 **PascalCase**（大驼峰命名法）：
```cpp
class TrajectoryPlanningNode;
class TRAC_IKKinematicsPlugin;  // 第三方插件保持原有风格
class MotionPlanningService;
class SingleArmOrchestrator;
```

### 1.2 方法名（Method Names）
使用 **camelCase**（小驼峰命名法）：
```cpp
bool getPositionIK();
bool searchPositionIK();
void moveJCallback();
int getKDLSegmentIndex();
```

### 1.3 变量名（Variable Names）
使用 **snake_case**（下划线分隔）：
```cpp
std::vector<std::string> joint_names_;
std::vector<std::string> link_names_;
uint num_joints_;
bool active_;
KDL::Chain chain_;
bool position_ik_;
std::string solve_type_;
```

### 1.4 成员变量（Member Variables）
私有成员变量使用尾部下划线：
```cpp
private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    bool active_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_movej_;
```

### 1.5 常量和宏定义（Constants & Macros）
使用 **UPPER_CASE**（全大写下划线分隔）：
```cpp
#define TRAC_IK_KINEMATICS_PLUGIN_
const double DEFAULT_PLANNING_TIME = 5.0;
```

### 1.6 命名空间（Namespaces）
使用小写字母和下划线：
```cpp
namespace trac_ik_kinematics_plugin {
namespace trajectory_planning::application::services {
namespace trajectory_planning::infrastructure::execution {
```

## 2. 文件和目录命名

### 2.1 文件名
使用 **snake_case**：
```
collision_checker.hpp
collision_checker.cpp
trajectory_planning_node.cpp
movej_planning_strategy.hpp
hardware_trajectory_controller.cpp
```

### 2.2 目录结构
遵循清洁架构模式，使用小写字母和下划线：
```
src/
├── application/
│   ├── services/
│   ├── use_cases/
│   └── orchestrators/
├── domain/
│   ├── entities/
│   ├── services/
│   └── value_objects/
└── infrastructure/
    ├── adapters/
    ├── integration/
    └── planning/
        └── strategies/
```

## 3. ROS 2 特定规范

### 3.1 消息定义（Message Files）
使用 **PascalCase**：
```
MoveCRequest.msg
JointConstrainedRequest.msg
JointConstraint.msg
```

### 3.2 话题和服务名
使用 **snake_case**：
```cpp
"movej_goals_internal"
"movel_goals_internal"
"movec_goals_internal"
"joint_constrained_goals_internal"
```

### 3.3 参数名
使用 **snake_case**：
```cpp
this->declare_parameter<std::string>("move_group", "arm");
this->declare_parameter<double>("planning_time", 5.0);
this->declare_parameter<double>("velocity_scaling", 0.2);
```

## 4. 代码格式化

### 4.1 缩进
- 使用 **4个空格** 进行缩进（不使用tab）
- 访问修饰符（public/private/protected）不缩进

### 4.2 大括号风格
使用 **Allman 风格**：
```cpp
class TrajectoryPlanningNode : public rclcpp::Node
{
public:
    TrajectoryPlanningNode() : Node("trajectory_planning_node")
    {
        // 代码内容
    }

private:
    void someMethod()
    {
        if (condition)
        {
            // 执行代码
        }
    }
};
```

### 4.3 行长度
- 建议不超过 **100-120字符**
- 长行适当换行，保持可读性

### 4.4 空格使用
```cpp
// 操作符前后加空格
int result = a + b;
if (condition && other_condition) {

// 函数调用不在括号内加空格
function_call(param1, param2);

// 模板参数列表紧贴
std::vector<std::string> names;
```

## 5. 注释规范

### 5.1 文件头注释
使用 Doxygen 风格：
```cpp
/**
 * @brief 统一轨迹规划入口节点，支持 MoveJ / MoveL / MoveC / JointConstrained 四种规划策略
 * @note 重构后使用应用服务层，职责清晰，易于测试和维护
 * @author Ding Kaiyue
 * @date 2025-09-22
 * @version 2.0 (使用应用服务架构)
 */
```

### 5.2 方法注释
```cpp
/**
 * @brief 获取关节位置的正向运动学
 * @param link_names 连杆名称列表
 * @param joint_angles 关节角度
 * @param poses 输出的位姿列表
 * @return 是否成功
 */
bool getPositionFK(const std::vector<std::string> &link_names,
                   const std::vector<double> &joint_angles,
                   std::vector<geometry_msgs::msg::Pose> &poses) const override;
```

### 5.3 行内注释
使用中文注释说明复杂逻辑：
```cpp
// 获取URDF模型
const urdf::ModelInterface &urdf_model = *robot_model.getURDF();

// 使用第一个tip frame
const std::string &tip_frame = tip_frames[0];
```

## 6. 包含头文件顺序

### 6.1 包含顺序
1. 对应的头文件（.hpp）
2. C++ 标准库
3. 第三方库（ROS2、MoveIt等）
4. 项目内部头文件

```cpp
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematics_base/kinematics_base.h>

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
```

## 7. 自动化工具

### 7.1 推荐格式化工具
```bash
# 使用 clang-format 格式化单个文件
clang-format -i src/your_file.cpp

# 格式化整个项目
find src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

### 7.2 编辑器配置
推荐配置支持：
- 4空格缩进
- 自动去除行尾空格
- Unix 行结束符（LF）

## 8. 特殊情况

### 8.1 第三方代码
第三方插件（如trac_ik）保持原有命名风格：
```cpp
class TRAC_IKKinematicsPlugin;  // 保持原有风格
```

### 8.2 ROS 2 生成的代码
自动生成的消息和服务代码保持生成器的默认风格。

## 9. 注意事项

- **重点关注功能正确性，格式为辅**
- CI 不会因格式问题阻止合并
- 鼓励但不强制使用自动格式化
- 新代码尽量遵循本规范
- 修改现有代码时保持周围代码风格一致