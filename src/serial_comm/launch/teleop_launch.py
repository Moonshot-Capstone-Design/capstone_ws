from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
        Node(
            package='serial_comm',        # 너의 패키지명으로 변경
            executable='md_teleop.py',      # 노드 파일 이름
            name='md_teleop_node',
            output='screen',
            parameters=[{'max_speed': 40}]
        ),
    ])
