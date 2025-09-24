#include "trajectory_planning_v3/application/services/trajectory_execution_service.hpp"

namespace trajectory_planning::application::services {

TrajectoryExecutionService::TrajectoryExecutionService(
    std::shared_ptr<TrajectoryExecutor> trajectory_executor,
    rclcpp::Logger logger)
    : trajectory_executor_(trajectory_executor), logger_(logger) {}

ExecutionResult TrajectoryExecutionService::execute(
    const Trajectory& trajectory) {
	if (!trajectory_executor_) {
		return createFailureResult("Trajectory executor not initialized");
	}

	if (trajectory.empty()) {
		return createFailureResult("Cannot execute empty trajectory");
	}

	try {
		RCLCPP_INFO(logger_, "执行轨迹：%zu个点，持续时间：%.2f秒",
		            trajectory.size(), trajectory.total_duration().seconds());

		bool success = trajectory_executor_->execute(trajectory);

		if (success) {
			return createSuccessResult("轨迹执行成功");
		} else {
			return createFailureResult("轨迹执行失败");
		}
	} catch (const std::exception& e) {
		return createFailureResult(std::string("执行过程中发生异常：") +
		                           e.what());
	}
}

bool TrajectoryExecutionService::isExecuting() const {
	if (!trajectory_executor_) {
		return false;
	}

	// 目前 TrajectoryExecutor 不支持状态查询，默认返回 false
	// 在实际应用中，可以扩展 TrajectoryExecutor 添加状态管理
	RCLCPP_DEBUG(logger_, "检查执行状态 - 当前不支持状态查询");
	return false;
}

void TrajectoryExecutionService::stop() {
	if (!trajectory_executor_) {
		RCLCPP_WARN(logger_, "无法停止 - 轨迹执行器未初始化");
		return;
	}

	try {
		// 目前 TrajectoryExecutor 不支持停止功能
		// 在实际应用中，可以扩展 TrajectoryExecutor 添加停止功能
		RCLCPP_WARN(
		    logger_,
		    "停止功能暂未实现 - 当前 TrajectoryExecutor 不支持停止操作");
		RCLCPP_INFO(logger_, "轨迹执行停止请求已记录");
	} catch (const std::exception& e) {
		RCLCPP_ERROR(logger_, "停止轨迹执行时发生错误：%s", e.what());
	}
}

ExecutionResult TrajectoryExecutionService::createFailureResult(
    const std::string& reason) {
	RCLCPP_ERROR(logger_, "轨迹执行失败：%s", reason.c_str());
	return ExecutionResult(false, reason);
}

ExecutionResult TrajectoryExecutionService::createSuccessResult(
    const std::string& message) {
	if (!message.empty()) {
		RCLCPP_INFO(logger_, "%s", message.c_str());
	}
	return ExecutionResult(true, message);
}

}  // namespace trajectory_planning::application::services