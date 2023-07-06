# Copyright (c) 2022，Horizon Robotics.
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    config_path = os.path.join(
        get_package_prefix('mipi_cam'),
        'lib/mipi_cam/config')

    config_path_launch_arg = DeclareLaunchArgument(
        "config_path", default_value=TextSubstitution(text="null")
    )
    
    return LaunchDescription([
        config_path_launch_arg,
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"config_path": [config_path, "/",
                                  LaunchConfiguration('config_path')]},
                {"camera_calibration_file_path": "/opt/tros/lib/mipi_cam/config/F37_calibration.yaml"},
                {"out_format": "nv12"},
                {"image_width": 960},
                {"image_height": 544},
                {"io_method": "shared_mem"},
                {"video_device": ""}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
