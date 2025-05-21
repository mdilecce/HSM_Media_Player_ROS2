from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description with a single node."""
    media_control_node = Node(
        package='media_system',
        executable='media_control',
        name='media_control',
        output='screen',
    )
    return LaunchDescription([media_control_node])