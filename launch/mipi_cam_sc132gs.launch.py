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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"camera_calibration_file_path": "/opt/tros/lib/mipi_cam/config/SC132GS_calibration.yaml"},
                {"out_format": "nv12"},
                {"image_width": 544},
                {"image_height": 640},
                {"io_method": "ros"},
                {"video_device": "sc132gs"},
                {"gdc_file_path": "/opt/tros/lib/mipi_cam/config/sc132gs_gdc.bin"},
                {"rotate_degree": 90}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
