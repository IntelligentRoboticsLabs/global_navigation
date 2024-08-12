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

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('local_navigation')
    param_file = os.path.join(pkg_dir, 'config', 'params_go2.yaml')

    local_navigation_cmd = Node(package='local_navigation',
                                executable='local_navigation_program',
                                output='screen',
                                parameters=[param_file],
                                # prefix=['xterm -e gdb -ex run  --args'],
                                # prefix=['perf record --call-graph dwarf -o perf.data'],
                                arguments=[],
                                remappings=[])

    mapping = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[param_file],
        remappings=[
            ('/input_cloud', '/lidar_points'),
            # ('/imu','/imu'),
            ],
        output='log'
        )

    graphbasedslam = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[param_file],
        output='log'
        )

    ld = LaunchDescription()
    ld.add_action(local_navigation_cmd)
    ld.add_action(mapping)
    ld.add_action(graphbasedslam)

    return ld
