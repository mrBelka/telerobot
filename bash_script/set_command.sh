#!/bin/bash

user="USER_NAME"
host="CURRENT_HOST"
password="PASSWORD"

commands=(
    "ros2 run wheel_driver wheel_driver --ros-args -p dev:=\"/dev/ttyUSB1\""
    "ros2 run odometry1 odometry1"
    "ros2 launch urg_node2 urg_node2.launch.py"
    "ros2 run hfi_a9_driver hfi_a9_driver"
    "ros2 run robot_localization ekf_node --ros-args --params-file ~/ekf.yaml"
    "ros2 launch linorobot2_description description.launch.py"
)

for command in "${commands[@]}"; do
    gnome-terminal -- bash -c "./ssh_command.sh $user $host $password \"$command\"; exec bash"
done

