from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    return LaunchDescription([
       DeclareLaunchArgument(
            'obstacle',
            default_value='0.3',
            description='Distance to stop before obstacle'
        ),
        DeclareLaunchArgument(
            'degrees',
            default_value='-90.0',
            description='Rotation in degrees'
        ),
        DeclareLaunchArgument(
            'final_approach',
            default_value='true',
            description='Whether to perform final 30cm approach'
        ),
        Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            output='screen'),
        Node(
            package='attach_shelf',
            executable='pre_approach_v2_node',
            output='screen',
            parameters=[{
                'obstacle': LaunchConfiguration('obstacle'),
                'degrees': LaunchConfiguration('degrees'),
                'final_approach': LaunchConfiguration('final_approach'),
            }]),
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('attach_shelf'), 'rviz', 'config.rviz')]
            ),
    ])