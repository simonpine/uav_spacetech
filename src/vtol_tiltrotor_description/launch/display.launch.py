#!/usr/bin/env python3
"""Load the tiltrotor into robot_state_publisher and drive the joints by hand.

    ros2 launch vtol_tiltrotor_description display.launch.py

Drag tilt_left_joint / tilt_right_joint from 0 to 1.57 in the
joint_state_publisher GUI to watch the hover -> cruise transition, and
rear_tilt_joint for the tricopter yaw vectoring.

This is the kinematics-only view. For flight, use:

    ros2 launch vtol_control vtol_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vtol_tiltrotor_description')
    xacro_file = os.path.join(pkg, 'urdf', 'vtol_tiltrotor.urdf.xacro')
    rviz_cfg = os.path.join(pkg, 'rviz', 'vtol.rviz')

    # use_gazebo:=false keeps the plugin block out of the RViz-only description.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_gazebo:=', LaunchConfiguration('use_gazebo'),
                 ' gz_flavor:=', LaunchConfiguration('gz_flavor')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('gz_flavor', default_value='fortress'),
        DeclareLaunchArgument('gui', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=IfCondition(
                ['not ', LaunchConfiguration('gui')]),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        ),
    ])
