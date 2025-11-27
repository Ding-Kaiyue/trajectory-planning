#pragma once

#include <optional>
#include <vector>

#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"
#include "trajectory_planning_v3/domain/value_objects/duration.hpp"

namespace trajectory_planning::domain::entities
{

/**
 * @brief 单条关节空间或笛卡尔空间的速度命令
 */
struct VelocityCommand {
    value_objects::JointVelocity velocity;  ///< 速度指令
    value_objects::Duration duration;       ///< 持续时间 (控制周期)，例如 0.01s

    // 相对进度百分比 [0,1]
    double progress_ratio = 0.0;
};

class VelocityCommandSequence {
public:
    enum class State { Idle, Running, Paused, Resumed, Cancelled, Completed };

    VelocityCommandSequence() = default;

    void add_command(const VelocityCommand& cmd);

    std::optional<VelocityCommand> command_at(size_t index) const;
    

    const std::vector<VelocityCommand>& commands() const;

    bool empty() const;
    size_t size() const;

    value_objects::Duration total_duration() const;

    /**
     * @brief 自动计算所有 progress_ratio
     */
    void compute_progress_ratios();

    // 状态管理 (与 Trajectory 一致)
    void set_state(State s);
    State state() const;

private:
    std::vector<VelocityCommand> commands_;
    State state_{State::Idle};
};

}  // namespace trajectory_planning::domain::entities