from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # 声明launch参数
    declared_arguments = [
        DeclareLaunchArgument(
            'planning_node_type',
            default_value='movej',
            description='Type of planning node to launch (movej/movel/movec/joint_constrained/trajectory_planning)',
            choices=['movej', 'movel', 'movec', 'joint_constrained', 'trajectory_planning']
        ),
    ]

    # 移除了配置文件的自动加载逻辑，简化参数传递

    # 节点定义
    nodes = [
        # 硬件轨迹控制器 - 总是启动
        Node(
            package='trajectory_planning_v3',
            executable='hardware_trajectory_controller_node',
            name='hardware_trajectory_controller',
            output='screen',
        ),

        # MoveJ节点 - 条件启动
        Node(
            package='trajectory_planning_v3',
            executable='movej_node',
            name='movej_node',
            output='screen',
            parameters=[{
                'planner_id': 'RRTConnectkConfigDefault',
                'planning_time': 5.0,
                'velocity_scaling': 0.2,
                'acceleration_scaling': 0.2,
                'goal_tolerance': 0.001,
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('planning_node_type'), "' == 'movej'"
                ])
            ),
        ),

        # MoveJ节点 - 条件启动
        Node(
            package='trajectory_planning_v3',
            executable='movel_node',
            name='movel_node',
            output='screen',
            parameters=[{
                'planner_id': 'RRTConnectkConfigDefault',
                'planning_time': 5.0,
                'velocity_scaling': 0.2,
                'acceleration_scaling': 0.2,
                'goal_tolerance': 0.001,
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('planning_node_type'), "' == 'movel'"
                ])
            ),
        ),

        # MoveC节点 - 条件启动
        Node(
            package='trajectory_planning_v3',
            executable='movec_node',
            name='movec_node',
            output='screen',
            parameters=[{
                'planner_id': 'RRTConnectkConfigDefault',
                'planning_time': 5.0,
                'velocity_scaling': 0.2,
                'acceleration_scaling': 0.2,
                'goal_tolerance': 0.001,
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('planning_node_type'), "' == 'movec'"
                ])
            ),
        ),

        # JointConstrainedPlanningStrategy节点 - 条件启动
        Node(
            package='trajectory_planning_v3',
            executable='joint_constrained_node',
            name='joint_constrained_node',
            output='screen',
            parameters=[{
                'planner_id': 'RRTConnectkConfigDefault',
                'planning_time': 5.0,
                'velocity_scaling': 0.2,
                'acceleration_scaling': 0.2,
                'goal_tolerance': 0.001,
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('planning_node_type'), "' == 'joint_constrained'"
                ])
            ),
        ),

        Node(
            package='trajectory_planning_v3',
            executable='trajectory_planning_node',
            name='trajectory_planning_node',
            output='screen',
            parameters=[{
                'planner_id': 'RRTConnectkConfigDefault',
                'planning_time': 5.0,
                'velocity_scaling': 0.2,
                'acceleration_scaling': 0.2,
                'goal_tolerance': 0.001,
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('planning_node_type'), "' == 'trajectory_planning'"
                ])
            ),
        ),
        
        # TODO: 其他规划节点可以在这里添加
        # Node(
        #     package='trajectory_planning_v3',
        #     executable='movel_node',
        #     name='movel_node',
        #     output='screen',
        #     condition=IfCondition(TextSubstitution(text=LaunchConfiguration('planning_node_type')).matches('movel')),
        # ),
    ]

    # 添加启动信息
    startup_info = [
        LogInfo(msg="=== Trajectory Planning Launch ==="),
        LogInfo(msg=["Planning Node Type: ", LaunchConfiguration('planning_node_type')]),
        LogInfo(msg="Hardware trajectory controller running on /arm_controller/follow_joint_trajectory"),
        LogInfo(msg="Note: Make sure robot_bringup launch file is running for robot_state_publisher, MoveIt, etc."),
    ]

    return LaunchDescription(declared_arguments + startup_info + nodes)
