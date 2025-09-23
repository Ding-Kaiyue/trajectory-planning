#pragma once

#include <stdexcept>
#include <string>
#include <sstream>

namespace trajectory_planning::domain::value_objects {

/**
 * @brief 表示时间持续长度 (单位: 秒)
 */
class Duration {
public:
    explicit Duration(double seconds) : seconds_(seconds) {
        if (seconds_ < 0.0) {
            throw std::invalid_argument("Duration cannot be negative");
        }
    }

    // 默认构造函数，初始化为0秒
    Duration() : seconds_(0.0) {}

    double seconds() const { return seconds_; }

    std::string to_string() const {
        std::ostringstream oss;
        oss << "Duration(" << seconds_ << "s)";
        return oss.str();
    }

private:
    double seconds_;
};

} // namespace trajectory_planning::domain::value_objects
