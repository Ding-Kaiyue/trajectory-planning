# MoveCTestWaypoint.msg
#
# MoveC测试用的简化航点定义
# 使用RPY角度代替四元数

# 位置
float64 x
float64 y
float64 z

# 姿态（RPY角度，单位：度）
float64 roll
float64 pitch
float64 yaw