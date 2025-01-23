from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory("hybrid_astar_model"),
        "urdf",
        "simple_car.urdf.xacro",
    )

    rviz_path = os.path.join(
        get_package_share_directory("hybrid_astar_model"), "rviz", "view.rviz"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro", " ", xacro_path])}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
    )

    planning_server = Node(
        package="hybrid_astar_planning_srv",
        executable="planning_server_node",
        name="planning_server_node",
    )

    hybrid_astar = Node(
        package="hybrid_astar_model",
        executable="hybrid_astar_run",
        name="hybrid_astar_node",
    )

    return LaunchDescription(
        [robot_state_publisher, rviz, planning_server, hybrid_astar]
    )
