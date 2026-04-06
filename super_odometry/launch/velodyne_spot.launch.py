import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    config_path = get_share_file(
        package_name="super_odometry",
        file_name="config/velodyne_spot.yaml")
    home_directory = os.path.expanduser("~")

    config_path_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to config file for super_odometry"
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="integrated_to_init"
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="map",
    )
    world_frame_rot_arg = DeclareLaunchArgument(
        "world_frame_rot",
        default_value="map_rot",
    )
    imu_frame_rectified_arg = DeclareLaunchArgument(
        "imu_frame_rectified",
        default_value="imu_frame_rectified",
    )
    lidar_frame_rectified_arg = DeclareLaunchArgument(
        "lidar_frame_rectified",
        default_value="lidar_frame_rectified",
    )
    dashboard_arg = DeclareLaunchArgument(
        "dashboard",
        default_value="true",
        description="Launch the live terminal dashboard",
    )

    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[LaunchConfiguration("config_file")],
    )

    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[
            LaunchConfiguration("config_file"),
            {"map_dir": os.path.join(home_directory, "/path/to/your/pcd")},
        ],
        remappings=[
            ("laser_odom_to_init", LaunchConfiguration("odom_topic")),
        ]
    )

    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[LaunchConfiguration("config_file")],
    )

    dashboard_node = Node(
        package="super_odometry",
        executable="superodom_dashboard.py",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        emulate_tty=True,
        parameters=[LaunchConfiguration("config_file")],
        condition=IfCondition(LaunchConfiguration("dashboard")),
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name="use_sim_time", value="true"),
        config_path_arg,
        odom_topic_arg,
        world_frame_arg,
        world_frame_rot_arg,
        lidar_frame_rectified_arg,
        imu_frame_rectified_arg,
        dashboard_arg,
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
        dashboard_node,
    ])
