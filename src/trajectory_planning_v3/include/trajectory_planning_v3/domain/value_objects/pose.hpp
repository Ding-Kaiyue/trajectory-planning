#pragma once

#include <array>
#include <sstream>
#include <string>

namespace trajectory_planning::domain::value_objects {

/**
 * @brief 笛卡尔位姿 (位置 + 四元数)
 */
class Pose {
public:
    Pose(std::array<double, 3> position, std::array<double, 4> orientation)
        : position_(position), orientation_(orientation) {}
    
    Pose() 
        : position_({0.0, 0.0, 0.0}), orientation_({1.0, 0.0, 0.0, 0.0}) {}
        
    const std::array<double, 3>& position() const { return position_; }
    const std::array<double, 4>& orientation() const { return orientation_; }

    std::string to_string() const {
        std::ostringstream oss;
        oss << "Pose(pos=[" << position_[0] << ", " << position_[1] << ", " << position_[2]
            << "], quat=[" << orientation_[0] << ", " << orientation_[1] << ", "
            << orientation_[2] << ", " << orientation_[3] << "])";
        return oss.str();
    }

private:
    std::array<double, 3> position_;     ///< x, y, z
    std::array<double, 4> orientation_;  ///< x, y, z, w
};

} // namespace trajectory_planning::domain::value_objects
