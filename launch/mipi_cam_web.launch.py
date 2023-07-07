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
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # include web launch file
    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket_service.launch.py'))
    )

    # args that can be set from the command line or a default will be used
    image_width_launch_arg = DeclareLaunchArgument(
        "image_width", default_value=TextSubstitution(text="1920")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "image_height", default_value=TextSubstitution(text="1080")
    )
    config_path_launch_arg = DeclareLaunchArgument(
        "config_path", default_value=TextSubstitution(text="")
    )

    config_path = os.path.join(
        get_package_prefix('mipi_cam'),
        './lib/mipi_cam/config'
    )

    # mipi cam图片发布pkg
    mipi_node = Node(
        package='mipi_cam',
        executable='mipi_cam',
        output='screen',
        parameters=[
            {"config_path": [config_path, "/",
                                  LaunchConfiguration('config_path')]},
            {"image_width": LaunchConfiguration('image_width')},
            {"image_height": LaunchConfiguration('image_height')},
            {"out_format": "nv12"},
            {"io_method": "shared_mem"},
            {"video_device": ""} # 支持的配置项为"F37"和"GC4663"
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )


    # jpeg图片编码&发布pkg
    jpeg_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "shared_mem"},
            {"in_format": "nv12"},
            {"out_mode": "ros"},
            {"out_format": "jpeg"},
            {"sub_topic": "/hbmem_img"},
            {"pub_topic": "/image"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # nv12图片解码&发布pkg
    nv12_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "ros"},
            {"in_format": "jpeg"},
            {"out_mode": "shared_mem"},
            {"out_format": "nv12"},
            {"sub_topic": "/image"},
            {"pub_topic": "/hbmem_img"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # web展示pkg
    web_node = Node(
        package='websocket',
        executable='websocket',
        output='screen',
        parameters=[
            {"image_topic": "/image"},
            {"only_show_image": True},
            {"image_type": "mjpeg"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    return LaunchDescription([
        web_service_launch_include,
        image_width_launch_arg,
        image_height_launch_arg,
        config_path_launch_arg,
        # 图片发布pkg
        mipi_node,
        # 图片编解码&发布pkg
        jpeg_codec_node,
        # 启动web展示pkg
        web_node
    ])
