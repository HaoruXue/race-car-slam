import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch


def generate_launch_description():
    launch_dir = get_package_share_directory("cmu_16833_project")

    lidar_topics = [
        "/vehicle_8/luminar_right_points",
        "/vehicle_8/luminar_left_points",
        "/vehicle_8/luminar_front_points",
    ]
    image_namespaces = [
        "/vimba_front_left",
        "/vimba_front_right",
        "/vimba_front_left_center",
        "/vimba_front_right_center",
        "/vimba_rear_left",
        "/vimba_rear_right",
    ]

    launches = []
    lidar_preprocessed_topics = "['" + ", '".join(lidar_topics) + "]"
    lidar_concat_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "launch", "lidar_concatenation.launch.py")
        ),
    )
    launches.append(lidar_concat_launch)

    for lidar_topic in lidar_topics:
        lidar_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "launch", "lidar_preprocessor.launch.py")
            ),
            launch_arguments=[("input_points_raw", lidar_topic)],
        )
        launches.append(lidar_launch)

    odom_to_tf_node = launch_ros.actions.Node(
        package="odom_to_tf_ros2",
        executable="odom_to_tf",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"odom_topic": "/vehicle_8/local_odometry"},
        ],
    )
    launches.append(odom_to_tf_node)

    # lidar_preprocessed_topics = "['" + "/filtered', '".join(lidar_topics) + "/filtered']"
    # lidar_detection_launch = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, "launch", "lidar_euclidean_clustering.launch.py")
    #     ),
    #     launch_arguments=[("input_points_raw_list", lidar_preprocessed_topics)],
    # )
    # tracker_launch = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, "launch", "tracking.launch.py")
    #     )
    # )
    # launches.append(lidar_detection_launch)
    # launches.append(tracker_launch)

    # for image_namespace in image_namespaces:
    #     image_detection_launch = launch.actions.IncludeLaunchDescription(
    #         launch.launch_description_sources.PythonLaunchDescriptionSource(
    #             os.path.join(launch_dir, "launch", "image_detection.launch.py")
    #         ),
    #         launch_arguments=[
    #             ("input_topic", image_namespace + "/image"),
    #             ("output_detection", image_namespace + "/detection"),
    #             ("output_image", image_namespace + "/detection_vis")
    #         ],
    #     )
    #     launches.append(image_detection_launch)

    return LaunchDescription(launches)
