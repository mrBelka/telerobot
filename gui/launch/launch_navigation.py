from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("urg_node2"),
                '/launch',
                '/urg_node2.launch.py'
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("linorobot2_description"),
                '/launch',
                '/description.launch.py'
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("linorobot2_navigation"),
                '/launch',
                '/slam.launch.py'
            ])
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            arguments=['--ros-args', '--params-file', '~/ekf.yaml']
        ),
        Node(
            package='hfi_a9_driver',
            executable='hfi_a9_driver',
            name='imu_node',
        )
    ])