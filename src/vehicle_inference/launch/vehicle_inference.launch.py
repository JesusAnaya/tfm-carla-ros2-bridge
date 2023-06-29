from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    node = Node(
        package='vehicle_inference',
        executable='vehicle_inference_node',
        output='screen',
    )
    return LaunchDescription([node])
