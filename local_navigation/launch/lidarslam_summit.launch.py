"""
This module contains the launch description for the local navigation package.

It includes the necessary nodes and configurations to run
the local navigation demo.
"""

# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory

import launch

import launch_ros.actions


def generate_launch_description():
    """
    Generate the launch description for the local navigation package.

    This function retrieves the package share directory and constructs the
    path to the parameter configuration file for the local navigation demo.

    :return: The launch description.
    """
    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('local_navigation'),
            'config',
            'lidarslam_summit.yaml',
        ),
    )

    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('local_navigation'), 'config',
            'local_nav.rviz'
        ),
    )

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[('/input_cloud', '/front_laser/points')],
        output='screen',
    )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen',
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        parameters=[main_param_dir],
        arguments=['-d', rviz_param_dir],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                'main_param_dir',
                default_value=main_param_dir,
                description='Full path to main parameter file to load',
            ),
            mapping,
            graphbasedslam,
            rviz,
        ]
    )
