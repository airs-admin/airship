import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(get_package_share_directory('airship_perception'), 'config/config.yaml'),
            description='Path to the config file'
        ),
        # seg_service_node
        Node(
            package='airship_perception',
            executable='seg_service_node',
            output='screen',
            parameters=[{'config': LaunchConfiguration('config')}]
        ),
    ])
