import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_params_path = os.path.join(
        get_package_share_directory('beer_bot'),
        'config',
        'slam_params.yaml'
    )
    return LaunchDescription([
        # Joystick
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='ps3_controller',
            output='screen'
        ),

        # Arduino driver
        Node(
            package='beer_bot',
            executable='arduino_driver',
            name='arduino_driver',
            output='screen'
        ),

        # Joystick → Twist
        Node(
            package='beer_bot',
            executable='joy_to_twist',
            name='joy_to_twist',
            output='screen'
        ),
        # URDF visualization (robot_state_publisher, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('beer_bot'),
                    'launch',
                    'view_model.launch.py'
                ])
            )
        ),
        # LIDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'remote_sllidar_a1_launch.py'
                ])
            )
        ),
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[slam_params_path],  # <— load the YAML here
        # ),
    ])
