# JointConstrainedTestRequest.msg
# Test message using RPY angles instead of quaternion for easier manual input

# Target end-effector position
geometry_msgs/Point goal_position

# Target end-effector orientation (Roll, Pitch, Yaw angles in radians)
float64 roll
float64 pitch
float64 yaw

# Planning type
uint8 JOINT = 0
uint8 CARTESIAN = 1
uint8 INTELLIGENT = 2
uint8 planning_type

# Joint constraints list
trajectory_planning_interfaces/JointConstraint[] joint_constraints

# Planning parameters
int32 max_attempts  # Maximum number of attempts, default 5