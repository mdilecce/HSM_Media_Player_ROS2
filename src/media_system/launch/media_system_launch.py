from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with a composable node."""
    container = ComposableNodeContainer(
            name='media_system_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='media_system',
                    plugin='media_system::MediaListener',
                    name='media_listener'),
                ComposableNode(
                    package='media_system',
                    plugin='media_system::MediaPlayerSystem',
                    name='media_player_system')
            ],
            output='screen',
    )

    return LaunchDescription([container])