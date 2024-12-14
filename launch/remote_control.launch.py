import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux', 
            executable='joy_linux_node', 
            name='ps3_controller',   
            namespace='',           
            output='screen',       
            parameters=[]
        ),
        Node(
            package='beer_bot', 
            executable='arduino_driver', 
            name='arduino_driver',   
            namespace='',           
            output='screen',       
            parameters=[]
        ),
        Node(
            package='beer_bot', 
            executable='joy_to_twist', 
            name='joy_to_twist',   
            namespace='',           
            output='screen',       
            parameters=[]
        )
    ])
