from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # launch arg
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_navigator'),
            'params',
            'points7.yaml'
        ]),
        description='Path to navigator parameter file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    navigator_node = Node(
        package='robot_navigator',
        executable='navigator_node',          # setup.py entrypoint or installed exec name
        name='navigator_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        navigator_node
    ])
