#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def is_valid_to_launch():
    path = '/sys/firmware/devicetree/base/model'
    if os.path.exists(path):
        return False
    else:
        return True


def generate_launch_description():
    if not is_valid_to_launch():
        print('Can not launch fake robot in Raspberry Pi')
        return LaunchDescription([])

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')

    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [FindPackageShare('open_manipulator_x_bringup'), 'worlds', 'empty_world.model']
        )
    )

    pose = {
        'x': LaunchConfiguration('x_pose', default='0.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00')
    }

    # Camera URDF
    camera_urdf_path = PathJoinSubstitution(
        [FindPackageShare('ros2_grasping'), 'urdf', 'camera.urdf']
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='false', description='Whether to execute RViz2'),
        DeclareLaunchArgument('prefix', default_value='""', description='Prefix of the joint and link names'),
        DeclareLaunchArgument('use_sim', default_value='true', description='Start robot in Gazebo simulation.'),
        DeclareLaunchArgument('world', default_value=world, description='Directory of Gazebo world file'),
        DeclareLaunchArgument('x_pose', default_value=pose['x'], description='position of open_manipulator_x'),
        DeclareLaunchArgument('y_pose', default_value=pose['y'], description='position of open_manipulator_x'),
        DeclareLaunchArgument('z_pose', default_value=pose['z'], description='position of open_manipulator_x'),
        DeclareLaunchArgument('roll', default_value=pose['R'], description='orientation of open_manipulator_x'),
        DeclareLaunchArgument('pitch', default_value=pose['P'], description='orientation of open_manipulator_x'),
        DeclareLaunchArgument('yaw', default_value=pose['Y'], description='orientation of open_manipulator_x'),

        # Robot State Publisher for Camera
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='camera_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro', ' ', camera_urdf_path])}]
        ),

        # Base launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('open_manipulator_x_bringup'), 'launch', 'base.launch.py'])]),
            launch_arguments={'start_rviz': start_rviz, 'prefix': prefix, 'use_sim': use_sim}.items(),
        ),

        # Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]),
            launch_arguments={'verbose': 'false', 'world': world}.items(),
        ),

        # Spawn Open Manipulator
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'open_manipulator_x_system',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
            ],
            output='screen',
        ),

        # Spawn Camera in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', camera_urdf_path,
                '-entity', 'camera_sensor',
                '-x', '0.2',  # Position along X-axis
                '-y', '0',  # Position along Y-axis
                '-z', '0.6',  # Position along Z-axis (height)
                '-R', '0',  # Roll angle
                '-P', '1.57',  # Pitch angle
                '-Y', '0'  # Yaw angle],
            ],
            output='screen',
        ),

        # Launch attacher_action.py
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros2_grasping', 'attacher_action.py'],
            output='screen'
        ),
    ])
