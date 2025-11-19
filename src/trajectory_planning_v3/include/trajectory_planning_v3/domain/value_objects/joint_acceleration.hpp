#pragma once

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace trajectory_planning::domain::value_objects {

/**
 * @brief 关节加速度 (rad/s²)
 */
class JointAcceleration {
public:
	explicit JointAcceleration(std::vector<double> values)
	    : values_(std::move(values)) {
		if (values_.empty()) {
			throw std::invalid_argument("JointAcceleration cannot be empty");
		}
	}

	JointAcceleration() = default;

	const std::vector<double>& values() const { return values_; }

	size_t size() const { return values_.size(); }

	double operator[](size_t idx) const { return values_.at(idx); }

	std::string to_string() const {
		std::ostringstream oss;
		oss << "JointAcceleration(";
		for (size_t i = 0; i < values_.size(); ++i) {
			oss << values_[i];
			if (i + 1 < values_.size()) oss << ", ";
		}
		oss << ")";
		return oss.str();
	}

private:
	std::vector<double> values_;
};

}  // namespace trajectory_planning::domain::value_objects
