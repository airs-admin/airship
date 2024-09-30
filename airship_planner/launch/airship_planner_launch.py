import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

project_name = 'airship_planner'
object_map = 'airship_object'

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory(project_name),
                    'config',
                    project_name + '.yaml'
                    )
    semantic_map_dir = os.path.join(get_package_share_directory(object_map), 'map/') 
    llm_parameters = [{"semantic_map_dir": semantic_map_dir}, {"config": config_file}]

    start_llm_planner_cmd = Node(
            package='airship_planner',
            executable='llm_planner',
            name='airship_planner',
            parameters=llm_parameters)

    ld = LaunchDescription()
    ld.add_action(start_llm_planner_cmd)

    return ld
