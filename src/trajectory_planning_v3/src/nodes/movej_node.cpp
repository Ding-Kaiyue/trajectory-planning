#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/application/services/trajectory_execution_service.hpp"
#include "trajectory_planning_v3/infrastructure/execution/trajectory_executor.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movej_planning_strategy.hpp"

using namespace trajectory_planning::application::services;
using namespace trajectory_planning::infrastructure::execution;
using namespace trajectory_planning::infrastructure::integration;
using namespace trajectory_planning::infrastructure::planning;
using namespace trajectory_planning::domain::entities;
using namespace trajectory_planning::domain::value_objects;

// ros2 topic pub --once /movej_goals sensor_msgs/msg/JointState "{position:
// [0.0, 0.5, -1.0, 1.2, 0.8, -0.3]}"

/**
 * @brief MoveJ节点，订阅目标关节位置，规划并执行轨迹
 * @note 该节点依赖MoveIt进行规划，需确保MoveIt配置正确
 * @author Ding Kaiyue
 * @date 2025-09-13
 * @version 1.0
 */
class MoveJNode : public rclcpp::Node {
public:
	MoveJNode() : Node("movej_node") {
		// ---- 1. 声明参数 ----
		this->declare_parameter<std::string>("move_group", "arm");
		this->declare_parameter<std::string>("planner_id",
		                                     "RRTConnectkConfigDefault");
		this->declare_parameter<double>("planning_time", 5.0);
		this->declare_parameter<double>("velocity_scaling", 0.2);
		this->declare_parameter<double>("acceleration_scaling", 0.2);
		this->declare_parameter<double>("goal_tolerance", 0.001);

		// ---- 2. 订阅 /movej_goals ----
		rclcpp::QoS qos(10);
		qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
		goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
		    "/movej_goals", qos,
		    std::bind(&MoveJNode::goalCallback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(),
		            "MoveJ Node ready. Publish to /movej_goals");
	}

	void initializeComponents() {
		if (!motion_planning_service_) {
			// 创建 MoveItAdapter, MoveJPlanningStrategy 和 TrajectoryExecutor
			auto group_name = this->get_parameter("move_group").as_string();

			// 初始化基础设施层
			moveit_adapter_ =
			    std::make_shared<MoveItAdapter>(shared_from_this(), group_name);
			trajectory_executor_ =
			    std::make_shared<TrajectoryExecutor>(*moveit_adapter_);

			auto movej_strategy =
			    std::make_shared<MoveJPlanningStrategy>(*moveit_adapter_);

			// 初始化应用服务
			motion_planning_service_ = std::make_shared<MotionPlanningService>(
			    moveit_adapter_, shared_from_this());

			// 注册需要的策略
			motion_planning_service_->registerMoveJStrategy(movej_strategy);

			trajectory_execution_service_ =
			    std::make_shared<TrajectoryExecutionService>(
			        trajectory_executor_, this->get_logger());

			RCLCPP_INFO(this->get_logger(),
			            "MoveItAdapter, MoveJPlanningStrategy and "
			            "TrajectoryExecutor initialized.");
		}
	}

private:
	void goalCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
		if (!checkReady("MoveJ")) return;

		auto result = motion_planning_service_->planJointMotion(*msg);

		if (result.success) {
			executeTrajectory(result.trajectory, "MoveJ");
		} else {
			RCLCPP_ERROR(this->get_logger(), "MoveJ 规划失败：%s",
			             result.error_message.c_str());
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

	void executeTrajectory(const Trajectory &trajectory,
	                       const std::string &tag) {
		auto result = trajectory_execution_service_->execute(trajectory);
		if (!result.success) {
			RCLCPP_ERROR(get_logger(), "[%s] 执行失败：%s", tag.c_str(),
			             result.error_message.c_str());
		} else {
			RCLCPP_INFO(get_logger(), "[%s] 执行成功", tag.c_str());
		}
	}

	// 基础设施层
	std::shared_ptr<MoveItAdapter> moveit_adapter_;
	std::shared_ptr<TrajectoryExecutor> trajectory_executor_;

	// 应用服务层
	std::shared_ptr<MotionPlanningService> motion_planning_service_;
	std::shared_ptr<TrajectoryExecutionService> trajectory_execution_service_;

	// ROS订阅者
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<MoveJNode>();
	// 初始化组件（延迟初始化）
	node->initializeComponents();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
