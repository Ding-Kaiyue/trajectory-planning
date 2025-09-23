#pragma once

#include <vector>
#include <string>
#include <limits>

namespace trajectory_planning::domain::entities {

/**
 * @brief 表示机器人运动的约束（速度、加速度、位置范围等）
 */
class Constraint {
public:
    Constraint() = default;

    Constraint(const std::vector<double>& pos_limits_min,
               const std::vector<double>& pos_limits_max,
               const std::vector<double>& vel_limits,
               const std::vector<double>& acc_limits);

    const std::vector<double>& position_limits_min() const;
    const std::vector<double>& position_limits_max() const;
    const std::vector<double>& velocity_limits() const;
    const std::vector<double>& acceleration_limits() const;

    bool is_within_limits(const std::vector<double>& positions,
                          const std::vector<double>& velocities,
                          const std::vector<double>& accelerations) const;

private:
    std::vector<double> pos_limits_min_;
    std::vector<double> pos_limits_max_;
    std::vector<double> vel_limits_;
    std::vector<double> acc_limits_;
};

}  // namespace trajectory_planning::domain::entities
