#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "robot_interfaces/msg/move_c_request.hpp"
#include "robot_interfaces/msg/move_c_test_request.hpp"
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/application/services/trajectory_execution_service.hpp"
#include "trajectory_planning_v3/infrastructure/execution/trajectory_executor.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movec_planning_strategy.hpp"

using namespace trajectory_planning::application::services;
using namespace trajectory_planning::infrastructure::execution;
using namespace trajectory_planning::infrastructure::integration;
using namespace trajectory_planning::infrastructure::planning;
using namespace trajectory_planning::domain::entities;
using namespace trajectory_planning::domain::value_objects;

/*
  # RPY测试话题 (完全自定义)：
  # 圆弧轨迹测试 - 每个点可以设置不同的RPY
  ros2 topic pub /movec_goals_test   robot_interfaces/msg/MoveCTestRequest
  "{route_type: 0, route_name: 'arc_test', waypoints: [ {x: 0.4, y: 0.0, z: 0.4,
  roll: -120.0, pitch: -20.0, yaw: -90.0}, {x: 0.35, y: 0.12, z: 0.45, roll:
  -50.0, pitch: 0.0, yaw: -90.0}
    ]}" -1
  */

/**
 * @brief MoveC节点，订阅MoveCRequest消息，规划并执行圆弧轨迹
 * @note 该节点依赖MoveIt进行规划，需确保MoveIt配置正确
 * @author Ding Kaiyue
 * @date 2025-09-17
 * @version 1.0
 */
class MoveCNode : public rclcpp::Node {
public:
	MoveCNode()
	    : Node("movec_node"),
	      tf_buffer_(this->get_clock()),
	      tf_listener_(tf_buffer_) {
		// ---- 1. 声明参数 ----
		this->declare_parameter<std::string>("move_group", "arm");
		this->declare_parameter<std::string>("planner_id",
		                                     "RRTConnectkConfigDefault");
		this->declare_parameter<double>("planning_time", 5.0);
		this->declare_parameter<double>("velocity_scaling", 0.2);
		this->declare_parameter<double>("acceleration_scaling", 0.2);
		this->declare_parameter<double>("goal_tolerance", 0.001);

		// ---- 2. 订阅 /movec_goals 和 /movec_goals_test ----
		rclcpp::QoS qos(10);
		qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
		goal_sub_ =
		    this->create_subscription<robot_interfaces::msg::MoveCRequest>(
		        "/movec_goals", qos,
		        std::bind(&MoveCNode::goalCallback, this,
		                  std::placeholders::_1));

		// 订阅更灵活的RPY测试话题
		goal_test_sub_ =
		    this->create_subscription<robot_interfaces::msg::MoveCTestRequest>(
		        "/movec_goals_test", qos,
		        std::bind(&MoveCNode::goalTestCallback, this,
		                  std::placeholders::_1));

		RCLCPP_INFO(
		    this->get_logger(),
		    "MoveC Node ready. Publish to:\n"
		    "  /movec_goals(MoveCRequest) - Full message\n"
		    "  /movec_goals_test(MoveCTestRequest) - RPY test message\n");
	}

	void initializeComponents() {
		if (!motion_planning_service_) {
			// 创建 MoveItAdapter, MoveCPlanningStrategy 和 TrajectoryExecutor
			auto group_name = this->get_parameter("move_group").as_string();

			// 初始化基础设施层
			moveit_adapter_ =
			    std::make_shared<MoveItAdapter>(shared_from_this(), group_name);
			trajectory_executor_ =
			    std::make_shared<TrajectoryExecutor>(*moveit_adapter_);

			auto movec_strategy =
			    std::make_shared<MoveCPlanningStrategy>(*moveit_adapter_);

			// 初始化应用服务
			motion_planning_service_ = std::make_shared<MotionPlanningService>(
			    moveit_adapter_, shared_from_this());

			// 注册需要的策略
			motion_planning_service_->registerMoveCStrategy(movec_strategy);

			trajectory_execution_service_ =
			    std::make_shared<TrajectoryExecutionService>(
			        trajectory_executor_, this->get_logger());

			RCLCPP_INFO(this->get_logger(),
			            "MoveItAdapter, MoveCPlanningStrategy and "
			            "TrajectoryExecutor initialized.");
		}
	}

private:
	void goalCallback(
	    const robot_interfaces::msg::MoveCRequest::SharedPtr msg) {
		if (!checkReady("MoveC")) return;

		auto result = motion_planning_service_->planArcMotion(*msg);

		if (result.success) {
			executeTrajectory(result.trajectory, "MoveC");
		} else {
			RCLCPP_ERROR(this->get_logger(), "MoveC 规划失败：%s",
			             result.error_message.c_str());
		}
	}

	void goalTestCallback(
	    const robot_interfaces::msg::MoveCTestRequest::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(),
		            "Received test request: route_type=%d, route_name='%s', "
		            "waypoints=%zu",
		            msg->route_type, msg->route_name.c_str(),
		            msg->waypoints.size());

		if (msg->waypoints.empty()) {
			RCLCPP_ERROR(this->get_logger(),
			             "No waypoints received in test request!");
			return;
		}

		// 转换为标准的MoveCRequest消息
		auto movec_msg =
		    std::make_shared<robot_interfaces::msg::MoveCRequest>();
		movec_msg->route_type = msg->route_type;
		movec_msg->route_name = msg->route_name;

		// 转换每个测试航点到标准位姿
		for (const auto& test_waypoint : msg->waypoints) {
			geometry_msgs::msg::Pose pose;

			// 设置位置
			pose.position.x = test_waypoint.x;
			pose.position.y = test_waypoint.y;
			pose.position.z = test_waypoint.z;

			// 将RPY转换为四元数
			tf2::Quaternion q;
			q.setRPY(test_waypoint.roll * M_PI / 180.0,
			         test_waypoint.pitch * M_PI / 180.0,
			         test_waypoint.yaw * M_PI / 180.0);

			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
			pose.orientation.w = q.w();

			movec_msg->waypoints.push_back(pose);
		}

		// 调用原有的回调函数
		goalCallback(movec_msg);
	}

	// 工具方法
	bool checkReady(const std::string& tag) {
		if (!motion_planning_service_ || !trajectory_execution_service_) {
			RCLCPP_WARN(get_logger(), "[%s] 应用服务未初始化", tag.c_str());
			return false;
		}
		return true;
	}

	void executeTrajectory(const Trajectory& trajectory,
	                       const std::string& tag) {
		auto result = trajectory_execution_service_->execute(trajectory);
		if (!result.success) {
			RCLCPP_ERROR(get_logger(), "[%s] 执行失败：%s", tag.c_str(),
			             result.error_message.c_str());
		} else {
			RCLCPP_INFO(get_logger(), "[%s] 执行成功", tag.c_str());
		}
	}

	geometry_msgs::msg::Pose createPose(
	    double x, double y, double z,
	    const geometry_msgs::msg::Pose::_orientation_type& orientation) {
		geometry_msgs::msg::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		pose.orientation = orientation;
		return pose;
	}

	// 基础设施层
	std::shared_ptr<MoveItAdapter> moveit_adapter_;
	std::shared_ptr<TrajectoryExecutor> trajectory_executor_;

	// 应用服务层
	std::shared_ptr<MotionPlanningService> motion_planning_service_;
	std::shared_ptr<TrajectoryExecutionService> trajectory_execution_service_;

	// ROS订阅者
	rclcpp::Subscription<robot_interfaces::msg::MoveCRequest>::SharedPtr
	    goal_sub_;
	rclcpp::Subscription<robot_interfaces::msg::MoveCTestRequest>::SharedPtr
	    goal_test_sub_;

	// TF相关
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<MoveCNode>();
	// 初始化组件（延迟初始化）
	node->initializeComponents();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}