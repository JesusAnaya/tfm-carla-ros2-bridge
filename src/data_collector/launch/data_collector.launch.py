from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Define the 'data_collector' launcher
    node = Node(
        package="data_collector", executable="data_collector_node", output="screen"
    )

    return LaunchDescription([node])
