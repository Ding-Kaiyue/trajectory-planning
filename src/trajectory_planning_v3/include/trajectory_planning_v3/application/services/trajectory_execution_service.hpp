#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/infrastructure/execution/trajectory_executor.hpp"

namespace trajectory_planning::application::services {

using namespace trajectory_planning::domain::entities;
using namespace trajectory_planning::infrastructure::execution;

struct ExecutionResult {
    bool success;
    std::string error_message;

    ExecutionResult(bool ok, const std::string& msg = "")
        : success(ok), error_message(msg) {}
};

class TrajectoryExecutionService {
public:
    TrajectoryExecutionService(
        std::shared_ptr<TrajectoryExecutor> trajectory_executor,
        rclcpp::Logger logger
    );

    // 执行轨迹
    ExecutionResult execute(const Trajectory& trajectory);

    // 检查执行状态
    bool isExecuting() const;

    // 停止执行
    void stop();

private:
    std::shared_ptr<TrajectoryExecutor> trajectory_executor_;
    rclcpp::Logger logger_;

    // 辅助方法
    ExecutionResult createFailureResult(const std::string& reason);
    ExecutionResult createSuccessResult(const std::string& message = "");
};

} // namespace trajectory_planning::application::services