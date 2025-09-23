#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/application/services/trajectory_execution_service.hpp"
#include "trajectory_planning_v3/infrastructure/execution/trajectory_executor.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movej_planning_strategy.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movel_planning_strategy.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movec_planning_strategy.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/joint_constrained_planning_strategy.hpp"

#include "robot_interfaces/msg/move_c_request.hpp"
#include "robot_interfaces/msg/joint_constrained_request.hpp"

using namespace trajectory_planning::application::services;
using namespace trajectory_planning::infrastructure::execution;
using namespace trajectory_planning::infrastructure::integration;
using namespace trajectory_planning::infrastructure::planning;
using namespace trajectory_planning::domain::entities;
using namespace trajectory_planning::domain::value_objects;

/**
 * @brief 统一轨迹规划入口节点，支持 MoveJ / MoveL / MoveC / JointConstrained 四种规划策略
 * @note 重构后使用应用服务层，职责清晰，易于测试和维护
 * @author Ding Kaiyue
 * @date 2025-09-22
 * @version 2.0 (使用应用服务架构)
 */
class TrajectoryPlanningNode : public rclcpp::Node {
public:
    TrajectoryPlanningNode() : Node("trajectory_planning_node") {
        // 声明参数
        this->declare_parameter<std::string>("move_group", "arm");
        this->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
        this->declare_parameter<double>("planning_time", 5.0);
        this->declare_parameter<double>("velocity_scaling", 0.2);
        this->declare_parameter<double>("acceleration_scaling", 0.2);
        this->declare_parameter<double>("goal_tolerance", 0.001);

        // 订阅不同规划请求
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        sub_movej_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "movej_goals_internal", qos,
            std::bind(&TrajectoryPlanningNode::moveJCallback, this, std::placeholders::_1)
        );
        sub_movel_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "movel_goals_internal", qos,
            std::bind(&TrajectoryPlanningNode::moveLCallback, this, std::placeholders::_1)
        );
        sub_movec_ = this->create_subscription<robot_interfaces::msg::MoveCRequest>(
            "movec_goals_internal", qos,
            std::bind(&TrajectoryPlanningNode::moveCCallback, this, std::placeholders::_1)
        );
        sub_joint_constrained_ = this->create_subscription<robot_interfaces::msg::JointConstrainedRequest>(
            "joint_constrained_goals_internal", qos,
            std::bind(&TrajectoryPlanningNode::jointConstrainedCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(),
            "TrajectoryPlanningNode 已启动，使用应用服务架构。订阅话题: \n"
            "   movej_goals_internal \n"
            "   movel_goals_internal \n"
            "   movec_goals_internal \n"
            "   joint_constrained_goals_internal"
        );
    }

    void initializeComponents() {
        if (!motion_planning_service_) {
            auto group_name = this->get_parameter("move_group").as_string();

            // 初始化基础设施层
            moveit_adapter_ = std::make_shared<MoveItAdapter>(shared_from_this(), group_name);
            trajectory_executor_ = std::make_shared<TrajectoryExecutor>(*moveit_adapter_);

            auto movej_strategy = std::make_shared<MoveJPlanningStrategy>(*moveit_adapter_);
            auto movel_strategy = std::make_shared<MoveLPlanningStrategy>(*moveit_adapter_);
            auto movec_strategy = std::make_shared<MoveCPlanningStrategy>(*moveit_adapter_);
            auto joint_constrained_strategy = std::make_shared<JointConstrainedPlanningStrategy>(*moveit_adapter_);

            // 初始化应用服务
            motion_planning_service_ = std::make_shared<MotionPlanningService>(
                movej_strategy, movel_strategy, movec_strategy, joint_constrained_strategy,
                moveit_adapter_, this->get_logger()
            );

            trajectory_execution_service_ = std::make_shared<TrajectoryExecutionService>(
                trajectory_executor_, this->get_logger()
            );

            RCLCPP_INFO(this->get_logger(), "应用服务初始化完成");
        }
    }

private:
    void moveJCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!checkReady("MoveJ")) return;

        auto result = motion_planning_service_->planJointMotion(*msg);

        if (result.success) {
            executeTrajectory(result.trajectory, "MoveJ");
        } else {
            RCLCPP_ERROR(this->get_logger(), "MoveJ 规划失败：%s", result.error_message.c_str());
        }
    }

    void moveLCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!checkReady("MoveL")) return;

        auto result = motion_planning_service_->planLinearMotion(*msg);

        if (result.success) {
            executeTrajectory(result.trajectory, "MoveL");
        } else {
            RCLCPP_ERROR(this->get_logger(), "MoveL 规划失败：%s", result.error_message.c_str());
        }
    }

    void moveCCallback(const robot_interfaces::msg::MoveCRequest::SharedPtr msg) {
        if (!checkReady("MoveC")) return;

        auto result = motion_planning_service_->planArcMotion(*msg);

        if (result.success) {
            executeTrajectory(result.trajectory, "MoveC");
        } else {
            RCLCPP_ERROR(this->get_logger(), "MoveC 规划失败：%s", result.error_message.c_str());
        }
    }

    void jointConstrainedCallback(const robot_interfaces::msg::JointConstrainedRequest::SharedPtr msg) {
        if (!checkReady("JointConstrained")) return;

        auto result = motion_planning_service_->planConstrainedMotion(*msg);

        if (result.success) {
            executeTrajectory(result.trajectory, "JointConstrained");
        } else {
            RCLCPP_ERROR(this->get_logger(), "约束规划失败：%s", result.error_message.c_str());
        }
    }

    // 工具方法
    bool checkReady(const std::string &tag) {
        if (!motion_planning_service_ || !trajectory_execution_service_) {
            RCLCPP_WARN(get_logger(), "[%s] 应用服务未初始化", tag.c_str());
            return false;
        }
        return true;
    }

    void executeTrajectory(const Trajectory &trajectory, const std::string &tag) {
        auto result = trajectory_execution_service_->execute(trajectory);
        if (!result.success) {
            RCLCPP_ERROR(get_logger(), "[%s] 执行失败：%s", tag.c_str(), result.error_message.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "[%s] 执行成功", tag.c_str());
        }
    }

    // 基础设施层
    std::shared_ptr<MoveItAdapter> moveit_adapter_;
    std::shared_ptr<TrajectoryExecutor> trajectory_executor_;

    // 应用服务层
    // std::shared_ptr<trajectory_planning::application::services::MotionPlanningService> motion_planning_service_;
    // std::shared_ptr<trajectory_planning::application::services::TrajectoryExecutionService> trajectory_execution_service_;
    std::shared_ptr<MotionPlanningService> motion_planning_service_;
    std::shared_ptr<TrajectoryExecutionService> trajectory_execution_service_;

    // ROS订阅者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_movej_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_movel_;
    rclcpp::Subscription<robot_interfaces::msg::MoveCRequest>::SharedPtr sub_movec_;
    rclcpp::Subscription<robot_interfaces::msg::JointConstrainedRequest>::SharedPtr sub_joint_constrained_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlanningNode>();
    node->initializeComponents();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}