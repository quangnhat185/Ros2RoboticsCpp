from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    save_img_arg = DeclareLaunchArgument(
        'save_img',
        default_value='false',
        description='Save plot as images'
    )

    sin_pub_node = Node(
        package='pub_cpp',
        executable='sin_pub',
        name='sin_pub_name',
    )

    plotting_node = Node(
        package='py_plotting',
        executable='sin_plotting',
        name='sin_plotting',
        parameters=[
            {'save_img': LaunchConfiguration('save_img')}
            # {'save_img' : 'true'}
        ]
    )

    ld.add_action(sin_pub_node)
    ld.add_action(save_img_arg)
    ld.add_action(plotting_node)

    return  ld