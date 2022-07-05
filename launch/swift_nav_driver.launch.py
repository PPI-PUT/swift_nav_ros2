# Copyright 2022 Perception for Physical Interaction Laboratory at Poznan University of Technology
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

import os.path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    swift_pkg_prefix = get_package_share_directory('swift_nav_ros2')

    swift_param_file = os.path.join(swift_pkg_prefix, 'param/defaults.param.yaml')

    swift_param = DeclareLaunchArgument(
        'swift_nav_driver_node_param_file',
        default_value=swift_param_file,
        description='Path to config file to Swift Nav Driver'
    )

    # Node
    swift_nav_driver_node = Node(
        package='swift_nav_ros2',
        executable='swift_nav_ros2_exe',
        name='swift_nav_driver',
        namespace='gnss',
        output='screen',
        parameters=[LaunchConfiguration('swift_nav_driver_node_param_file')]
    )

    return LaunchDescription([
        swift_param,
        swift_nav_driver_node,
    ])
