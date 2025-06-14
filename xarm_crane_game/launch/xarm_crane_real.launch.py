#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.203')
    report_type = LaunchConfiguration('report_type', default='dev')
    dof = LaunchConfiguration('dof', default=6)
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    # robot moveit servo launch
    # xarm_moveit_servo/launch/_robot_moveit_servo.launch.py
    robot_moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_crane_game'), 'launch', '_xarm_crane_real.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'baud_checkset': baud_checkset,
            'default_gripper_baud': default_gripper_baud,
            'dof': dof,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'robot_type': robot_type,
            'ros2_control_plugin': 'uf_robot_hardware/UFRobotSystemHardware',
        }.items(),
    )

    # # robot driver launch
    # # xarm_api/launch/_robot_driver.launch.py
    # robot_driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
    #     launch_arguments={
    #         'robot_ip': robot_ip,
    #         'report_type': report_type,
    #         'dof': dof,
    #         'hw_ns': hw_ns,
    #         'add_gripper': add_gripper,
    #         'prefix': prefix,
    #         'baud_checkset': baud_checkset,
    #         'default_gripper_baud': default_gripper_baud,
    #         'robot_type': robot_type,
    #     }.items(),
    # )

    config_xarm_crane_game = os.path.join(
        get_package_share_directory('xarm_crane_game'),
        'config',
        'xarm_crane_game.yaml'
    )

    delayd_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='xarm_crane_game',
                executable='joy_to_servo_node',
                name='joy_to_servo_node',
                parameters=[config_xarm_crane_game]
            )
        ]
    )
    
    return LaunchDescription([
        robot_moveit_servo_launch,
        # robot_driver_launch,
        delayd_node
    ])