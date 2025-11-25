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
            'points.yaml'
        ]),
        description='Path to navigator parameter file'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for this AMR (optional)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    navigator_node = Node(
        package='robot_navigator',
        executable='navigator',          # setup.py entrypoint or installed exec name
        name='navigator',
        namespace=namespace,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        # 필요하면 토픽 리맵
        remappings=[
            # ('destination', '/destination')  # 예: 전역 토픽으로 쓰고 싶을 때
        ],
        env={
            # AMR_NUMBER 환경변수 필요하면 런치에서 넣어줄 수도 있음
            'AMR_NUMBER': EnvironmentVariable('AMR_NUMBER')
        }
    )

    return LaunchDescription([
        params_file_arg,
        namespace_arg,
        use_sim_time_arg,
        navigator_node
    ])
