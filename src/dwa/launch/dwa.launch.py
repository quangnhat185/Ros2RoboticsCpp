import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('dwa'),'rviz', 'dwa.rviz')

    print(rviz_config_dir)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    )

    dwa_node = Node(
        package='dwa',
        executable='dwa_run',
        name='dwa_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(rviz_node)
    ld.add_action(dwa_node)

    return ld