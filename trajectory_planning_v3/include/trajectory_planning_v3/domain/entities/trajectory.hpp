#pragma once

#include <optional>
#include <vector>

#include "trajectory_planning_v3/domain/value_objects/duration.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_acceleration.hpp"

namespace trajectory_planning::domain::entities {

/**
 * @brief 轨迹点数据结构（包含所有必要的运动学信息）
 */
struct TrajectoryPoint {
	value_objects::JointPosition position;
	value_objects::JointVelocity velocity;
	value_objects::JointAcceleration acceleration;
	value_objects::Duration time_from_start;
	double progress_ratio = 0.0;  // 轨迹进度比（0-1）
};

/**
 * @brief 整个轨迹
 */
class Trajectory {
public:
	enum class State { Idle, Running, Paused, Resumed, Cancelled, Completed };

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

}  // namespace trajectory_planning::domain::entities
