#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Package paths
    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    gazebo_ros_dir = get_package_share_directory(
        'gazebo_ros'
    )

    # TurtleBot3 House world
    world_path = os.path.join(
        turtlebot3_gazebo_dir,
        'worlds',
        'turtlebot3_house.world'
    )

    # TurtleBot3 Waffle model
    model_path = os.path.join(
        turtlebot3_gazebo_dir,
        'models',
        'turtlebot3_waffle',
        'model.sdf'
    )

    # Initial pose
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='-6.5',
        description='Initial X position'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='-3.0',
        description='Initial Y position'
    )

    declare_yaw_pose = DeclareLaunchArgument(
        'yaw_pose',
        default_value='1.5708',
        description='Initial yaw angle in radians'
    )

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    # Gazebo GUI
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gzclient.launch.py'
            )
        )
    )

    # TurtleBot3 robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_dir,
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # Spawn TurtleBot3
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-file', model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw_pose,
        ],
        output='screen'
    )

    return LaunchDescription([

        # Project uses TurtleBot3 Waffle
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='waffle'
        ),

        declare_x_pose,
        declare_y_pose,
        declare_yaw_pose,

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
    ])