# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Author: Manuel Beschi <manuel.beschi@unibs.it>

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('surface_raster_planner'), 'config', 'raster_planner.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('input_mesh', default_value='/cloud_merger_node/mesh',
                              description='Mesh published by node 2 (shape_msgs/Mesh)'),
        Node(
            package='surface_raster_planner',
            executable='raster_planner_node',
            name='raster_planner_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('mesh', LaunchConfiguration('input_mesh'))],
        ),
    ])
