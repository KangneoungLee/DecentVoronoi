from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_virtual_agent_ros_node',
             name='decent_virtual_agent_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_common.yaml')]
            ),
        ]
        )
        
