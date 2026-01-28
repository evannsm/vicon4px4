from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only the visual_odometry_relay node."""
    
    visual_odometry_relay = Node(
        package='vicon_receiver',
        executable='visual_odometry_relay',
        output='screen',
    )

    return LaunchDescription([visual_odometry_relay])