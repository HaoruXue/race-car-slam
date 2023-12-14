# Copyright 2022 AI Racing Tech
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = "pointcloud_preprocessor"

    # declare launch arguments
    input_points_raw_param = DeclareLaunchArgument(
        "input_points_raw", default_value="/livox/lidar/front_left"
    )

    tf_output_frame_param = DeclareLaunchArgument("tf_output_frame", default_value="base_link")

    param_file = DeclareLaunchArgument(
        "lidar_preprocessor_param_file",
        default_value=os.path.join(
            get_package_share_directory("cmu_16833_project"),
            "param/lidar_preprocessor.param.yaml",
        ),
    )

    # # set ground filter as a component
    # ground_filter_component = ComposableNode(
    #     package="ground_segmentation",
    #     plugin="ground_segmentation::ScanGroundFilterComponent",
    #     name="scan_ground_filter",
    #     namespace=LaunchConfiguration("input_points_raw"),
    #     remappings=[
    #         ("input", LaunchConfiguration("input_points_raw")),
    #         ("output", "points_raw/ground_filtered"),
    #     ],
    #     parameters=[LaunchConfiguration("lidar_preprocessor_param_file")],
    # )

    # set downsampling filter as a component
    downsample_component = ComposableNode(
        package=pkg,
        plugin="pointcloud_preprocessor::ApproximateDownsampleFilterComponent",
        name="downsampler",
        namespace=LaunchConfiguration("input_points_raw"),
        remappings=[
            # ("input", "points_raw/ground_filtered"),
            ("input", LaunchConfiguration("input_points_raw")),
            # ("output", "points_raw/downsampled"),
            ("output", "filtered"),
        ],
        parameters=[LaunchConfiguration("lidar_preprocessor_param_file"),
                    {"use_sim_time": True},
                    ],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="pointcloud_preprocessor_container",
        namespace=LaunchConfiguration("input_points_raw"),
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            downsample_component,
            # ground_filter_component,
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [input_points_raw_param, tf_output_frame_param, param_file, container]
    )