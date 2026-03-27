#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    train = LaunchConfiguration('train', default='false')

    pkg = get_package_share_directory('segway')
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'segway_world.world.sdf')
    bridge_params = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')

    xacro_file = os.path.join(pkg, 'urdf', 'robot', 'segway.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    gz_sim_launch = os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')

    return LaunchDescription([

        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': world + ' -r -v 3 --render-engine ogre2'}.items(),
            condition=IfCondition(train),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': world + ' -v 3 --render-engine ogre2'}.items(),
            condition=UnlessCondition(train),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='segway',
            executable='optimize_pid.py',
            output='screen',
            condition=IfCondition(train),
        ),

        Node(package='segway',
             executable='segway_control.py',
             output='screen',
             condition=UnlessCondition(train),
        ),
    ])