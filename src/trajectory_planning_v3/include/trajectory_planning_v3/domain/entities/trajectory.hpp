#pragma once

#include <vector>
#include <optional>
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_acceleration.hpp"
#include "trajectory_planning_v3/domain/value_objects/duration.hpp"

namespace trajectory_planning::domain::entities {

/**
 * @brief 单个轨迹点
 */
struct TrajectoryPoint {
    value_objects::JointPosition position;
    value_objects::JointVelocity velocity;
    value_objects::JointAcceleration acceleration;
    value_objects::Duration time_from_start;

    // 相对进度百分比 [0,1]
    double progress_ratio = 0.0;
};

/**
 * @brief 整个轨迹
 */
class Trajectory {
public:
    enum class State {
        Idle,
        Running,
        Paused,
        Resumed,
        Cancelled,
        Completed
    };

    Trajectory() = default;

    void add_point(const TrajectoryPoint& point);
    std::optional<TrajectoryPoint> point_at(size_t index) const;
    const std::vector<TrajectoryPoint>& points() const;

    bool empty() const;
    size_t size() const;

    value_objects::Duration total_duration() const;
    void compute_progress_ratios();

    // 状态管理
    void set_state(State s);
    State state() const;

private:
    std::vector<TrajectoryPoint> points_;
    State state_{State::Idle};
};

} // namespace trajectory_planning_v3::domain::entities
