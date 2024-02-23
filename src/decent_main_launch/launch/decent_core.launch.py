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
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent0_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent1_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent2_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent3_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent4_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent5_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent6_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent7_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent8_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        Node(
             package='decent_voronoi',
             namespace='',
             executable='decent_voronoi_ros_agent_node',
             name='decent_voronoi_agent9_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("decent_main_launch"), 'params', 'decent_core.yaml')]
            ),
        ]
        )
        
