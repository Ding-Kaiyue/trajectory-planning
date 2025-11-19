import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def prepare_robot_description(context, *args, **kwargs):
    robot_model_name = LaunchConfiguration('robot_model_name').perform(context)
    
    # 拼接xacro文件路径
    robot_description_path = os.path.join(
        FindPackageShare('robot_description').perform(context),
        'urdf',
        f'{robot_model_name}.urdf'
    )

    # 检查文件是否存在
    if not os.path.exists(robot_description_path):
        raise FileNotFoundError(f"❌ Robot model file does not exist: {robot_description_path}")

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', robot_description_path
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            respawn=True,
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model_name',
            default_value='arm620',
            description='Robot model name without extension (e.g., arm620 or arm380)'
        ),
        OpaqueFunction(function=prepare_robot_description)
    ])