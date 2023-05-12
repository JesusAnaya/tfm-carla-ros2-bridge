from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    # Declare the 'host' argument with a default value of 'localhost'
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='localhost',
        description='Host IP for connecting to the CARLA simulator')

    town_arg = DeclareLaunchArgument(
        'town',
        default_value='Town01_Opt',
        description='Town for connecting to the CARLA simulator')

    # Get the value of the 'host' argument
    host_value = LaunchConfiguration('host')
    town_value = LaunchConfiguration('town')

    # Define the 'carla_ros_node' with the 'host' parameter
    carla_ros_node = Node(
        package='custom_carla_bridge',
        executable='carla_ros_node',
        output='screen',
        parameters=[{
            'host': host_value,
            'town': town_value
        }])

    return LaunchDescription([host_arg, town_arg, carla_ros_node])
