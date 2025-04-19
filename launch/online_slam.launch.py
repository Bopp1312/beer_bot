from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('beer_bot'),
        'config',
        'slam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
            arguments=['--ros-args', '--log-level', 'slam_toolbox:=debug']

        )
    ])
