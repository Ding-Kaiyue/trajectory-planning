#include "trajectory_planning_v3/domain/entities/velocity_command.hpp"

namespace trajectory_planning::domain::entities {

void VelocityCommandSequence::add_command(const VelocityCommand& cmd) {
    commands_.push_back(cmd);
}

std::optional<VelocityCommand> VelocityCommandSequence::command_at(size_t index) const {
    if (index < commands_.size()) {
        return commands_[index];
    }
    return std::nullopt;
}

const std::vector<VelocityCommand>& VelocityCommandSequence::commands() const {
    return commands_;
}

bool VelocityCommandSequence::empty() const { return commands_.empty(); }

size_t VelocityCommandSequence::size() const { return commands_.size(); }

value_objects::Duration VelocityCommandSequence::total_duration() const {
    if (commands_.empty()) return value_objects::Duration(0.0);

    // 累加所有命令的持续时间
    double total_seconds = 0.0;
    for (const auto& cmd : commands_) {
        total_seconds += cmd.duration.seconds();
    }
    return value_objects::Duration(total_seconds);
}

void VelocityCommandSequence::compute_progress_ratios() {
    if (commands_.empty()) return;

    double total = total_duration().seconds();
    if (total <= 0.0) return;

    double accumulated_time = 0.0;
    for (auto& cmd : commands_) {
        accumulated_time += cmd.duration.seconds();
        cmd.progress_ratio = accumulated_time / total;
    }
}

void VelocityCommandSequence::set_state(State s) { state_ = s; }

VelocityCommandSequence::State VelocityCommandSequence::state() const { return state_; }

}  // namespace trajectory_planning::domain::entities
