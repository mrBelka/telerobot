from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gui_node = Node(
        package='gui',
        executable='app',
        name='gui_app',
        output='screen'
    )
    return LaunchDescription([gui_node])