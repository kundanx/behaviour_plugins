import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    behaviour_plugins = get_package_share_directory('behaviour_plugins')

    bt_node = Node(
        package="behaviour_plugins",
        executable="bt",
        name="myBt",
        parameters=[{
            "location_file": os.path.join(behaviour_plugins, "config","location_param.yaml")
        }]
    )

    ld = LaunchDescription()
    
    ld.add_action(bt_node)

    return ld