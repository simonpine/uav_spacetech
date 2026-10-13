#!/usr/bin/env python3
"""Full simulation stack for the tri-motor tiltrotor.

    ros2 launch vtol_control vtol_sim.launch.py

Brings up Gazebo Fortress, spawns the aircraft, bridges the actuator and sensor
topics and starts the flight controller. It deliberately does NOT start the
keyboard pilot: that needs a real tty, and launch does not give its children
one. Start it yourself in a second terminal:

    ros2 run vtol_control keyboard_teleop

Useful arguments:
    gui:=false            headless, for tuning runs
    rviz:=true            add RViz alongside Gazebo
    render_engine:=ogre   fall back from ogre2 if the GPU driver chokes
    paused:=true          start paused, step with the GUI
    spawn_z:=0.25         drop height; 0.25 puts the skids on the ground
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'vtol_field'


def generate_launch_description():
    desc_pkg = get_package_share_directory('vtol_tiltrotor_description')
    ctrl_pkg = get_package_share_directory('vtol_control')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'vtol_tiltrotor.urdf.xacro')
    world_file = os.path.join(desc_pkg, 'worlds', f'{WORLD_NAME}.sdf')
    bridge_cfg = os.path.join(desc_pkg, 'config', 'gz_bridge.yaml')
    rviz_cfg = os.path.join(desc_pkg, 'rviz', 'vtol.rviz')
    ctrl_cfg = os.path.join(ctrl_pkg, 'config', 'flight_controller.yaml')

    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('render_engine', default_value='ogre2',
                              description='ogre2, or ogre if the driver cannot do it'),
        DeclareLaunchArgument('spawn_z', default_value='0.25'),
        DeclareLaunchArgument('controller', default_value='true'),
        DeclareLaunchArgument('publish_surfaces', default_value='true',
                              description='false = fly on rotors only, leaving the '
                                          'aerodynamic surfaces to something else '
                                          '(tools/measure_surfaces.py)'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=true gz_flavor:=fortress']),
        value_type=str)

    # "-r" runs immediately, "-s" is server-only, "-v 4" is verbose.
    gz_args = PythonExpression([
        "''",
        " + ('' if '", LaunchConfiguration('paused'), "' == 'true' else '-r ')",
        " + ('' if '", LaunchConfiguration('gui'), "' == 'true' else '-s ')",
        " + ('-v 4 ' if '", LaunchConfiguration('verbose'), "' == 'true' else '')",
        " + '--render-engine ", LaunchConfiguration('render_engine'), " '",
        " + '", world_file, "'",
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args, 'gz_version': '6'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # Spawns from /robot_description, so the URDF is only expanded once.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-world', WORLD_NAME,
                   '-topic', 'robot_description',
                   '-name', 'vtol',
                   '-allow_renaming', 'false',
                   '-x', '0', '-y', '0', '-z', LaunchConfiguration('spawn_z')])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vtol_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}])

    controller = Node(
        package='vtol_control',
        executable='flight_controller',
        name='vtol_flight_controller',
        output='screen',
        parameters=[ctrl_cfg,
                    {'publish_surfaces': ParameterValue(
                        LaunchConfiguration('publish_surfaces'), value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('controller')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    # The controller must not start publishing into a world that has no aircraft
    # in it yet, otherwise the JointPositionController plugins miss the first
    # commands and the tilt servos sit at their spawn angle.
    controller_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[controller]))

    hint = ExecuteProcess(
        cmd=['bash', '-c',
             'echo; echo "=========================================================";'
             'echo " Simulator up. Now, in a SECOND terminal:";'
             'echo;'
             'echo "   cd ~/uav && source install/setup.bash";'
             'echo "   ros2 run vtol_control keyboard_teleop";'
             'echo;'
             'echo " Then: ENTER to arm, T to take off, SPACE to switch modes.";'
             'echo "========================================================="; echo'],
        output='screen')

    return LaunchDescription(args + [
        # Fortress looks for world/mesh resources here.
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                               os.path.join(desc_pkg, 'worlds')),
        gazebo, robot_state_publisher, spawn, bridge, controller_after_spawn,
        rviz, hint,
    ])
