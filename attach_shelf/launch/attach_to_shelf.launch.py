from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            output='screen'),
        Node(
            package='attach_shelf',
            executable='pre_approach_v2_node',
            output='screen'),
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('attach_shelf'), 'rviz', 'config.rviz')]
            ),
    ])