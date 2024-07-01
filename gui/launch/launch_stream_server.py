from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cam_driver_node = Node(
        package='cam_driver',
        executable='server',
        name='cam_driver_node',
    )
    audio_driver_node_1 = Node(
        package ='audio_driver',
        executable='server1',
        name='audio_driver_node_1',
    )

    return LaunchDescription([cam_driver_node, audio_driver_node_1])