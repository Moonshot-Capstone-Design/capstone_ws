from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    node_joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )
    ld.add_action(node_joy)


    # Path to the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    urdf_file = os.path.join(pkg_path, 'description', 'robot_description.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    ld.add_action(node_robot_state_publisher)

    return ld
