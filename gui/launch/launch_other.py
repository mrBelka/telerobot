from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    wheel_driver_node = Node(
        package='wheel_driver',
        executable='wheel_driver',
        name='wheel_driver',
        output='screen',
        parameters=[{'dev': '/dev/ttyUSB1'}]
    )
    odometry = Node(
        package ='odometry1',
        executable='odometry1',
        name='odometry1',
        output = 'screen'
    )
    battery_controller = Node(
        package='battery_controller',
        executable='battery_controller',
        name='battery_controller_node',
        output='screen',
        parameters=[{'dev': '/dev/ttyUSB1'}]
    )


    return LaunchDescription([wheel_driver_node, odometry, battery_controller])