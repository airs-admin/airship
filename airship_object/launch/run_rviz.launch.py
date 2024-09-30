import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_rviz_file = LaunchConfiguration('config_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='config_rviz',
            default_value=os.path.join(get_package_share_directory('airship_object'), 'config', 'airship_object.rviz'),
            description='Full path to the RVIZ config file to use'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_rviz_file],
        ),
    ])

