from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    wheel_driver_node = Node(
        package='wheel_driver',
        executable='wheel_driver',
        name='wheel_driver_node',
        arguments=['--ros-args', '-p', 'dev:=/dev/ttyUSB1']
    )
    odometry = Node(
        package ='odometry1',
        executable='odometry1',
        name='odometry1',
    )

    return LaunchDescription([wheel_driver_node, odometry])