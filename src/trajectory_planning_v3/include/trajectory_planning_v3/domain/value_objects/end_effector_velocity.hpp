#pragma once
#include <array>
#include <sstream>
#include <string>

namespace trajectory_planning::domain::value_objects {

/**
 * @brief 末端执行器速度
 * 包含线速度 (vx, vy, vz) 和角速度 (wx, wy, wz)
 */
class EndEffectorVelocity {
public:
    EndEffectorVelocity(const std::array<double, 6>& velocity)
        : velocity_(velocity) {}

    EndEffectorVelocity() : velocity_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}
    
    const std::array<double, 6>& velocity() const { return velocity_; }

    std::string to_string() const {
        std::ostringstream oss;
        oss << "EndEffectorVelocity([";
        for (size_t i = 0; i < 6; ++i) {
            oss << velocity_[i];
            if (i < 5) oss << ", ";
        }
        oss << "])";
        return oss.str();
    }

private:
    std::array<double, 6> velocity_;
};

} // namespace trajectory_planning::domain::value_objects
