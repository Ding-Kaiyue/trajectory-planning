# MoveCTestRequest.msg
#
# 用于MoveC测试的简化消息格式
# 使用RPY角度代替四元数，便于手动输入测试

# 轨迹类型
uint8 route_type    # 0: ARC 1: BEZIER 2: CIRCLE 3: CIRCLE3PT
string route_name   # 轨迹名称（可选）

# 简化的位姿点（使用RPY角度）
robot_interfaces/MoveCTestWaypoint[] waypoints