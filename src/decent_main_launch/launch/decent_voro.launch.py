from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()
    load_nodes = GroupAction(
    actions=[
        IncludeLaunchDescription(
                  PythonLaunchDescriptionSource([
                       FindPackageShare("decent_main_launch"),'/launch','/decent_core.launch.py'])        
             ),
        IncludeLaunchDescription(
                  PythonLaunchDescriptionSource([
                       FindPackageShare("decent_main_launch"),'/launch','/decent_interface.launch.py'])        
             ),
        #IncludeLaunchDescription(
        #          PythonLaunchDescriptionSource([
        #               FindPackageShare("nri_plan_main_launch"),'/launch','/nri_plan_waypoint.launch.py'])        
        #     ),
        #IncludeLaunchDescription(
        #          PythonLaunchDescriptionSource([
        #               FindPackageShare("nri_plan_main_launch"),'/launch','/nri_plan_agent_websocket.launch.py'])        
        #     ),
      ],)
    
    
    #load_delay_nodes = TimerAction(period=5.0,
    #actions=[
    #     IncludeLaunchDescription(
    #              PythonLaunchDescriptionSource([
    #                   FindPackageShare("robot_launcher"), '/launch', '/navigation_launch.py'])        
    #         ),
    #  ],)
    
    ld.add_action(load_nodes)
    #ld.add_action(load_delay_nodes)
    return ld
    

        
