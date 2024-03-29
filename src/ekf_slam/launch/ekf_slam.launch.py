import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('ekf_slam'),'rviz', 'ekf_slam.rviz')

    print(rviz_config_dir)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    )

    ekf_slam_node = Node(
        package='ekf_slam',
        executable='ekf_slam_run',
        name='ekf_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(rviz_node)
    ld.add_action(ekf_slam_node)

    return ld