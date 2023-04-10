from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sin_pub_node = Node(
        package='pub_cpp',
        executable='sin_pub',
        name='sin_pub_name'
    )

    plotting_node = Node(
        package='py_plotting',
        executable='sin_plotting',
        name='sin_plotting_name'
    )

    ld.add_action(sin_pub_node)
    ld.add_action(plotting_node)

    return ld