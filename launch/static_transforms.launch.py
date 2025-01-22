from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',   # Translation (x, y, z) in meters
                '0', '0', '0',   # Rotation (roll, pitch, yaw) in radians
                'base_link',     # Parent frame
                'laser'          # Child frame
            ],
        )
    ])

