import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    config_path = get_share_file(
        package_name="super_odometry",
        file_name="config/livox_mid360_alphatruck.yaml",
    )
    calib_path = get_share_file(
        package_name="super_odometry",
        file_name="config/livox/livox_mid360_calibration.yaml",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to the SuperOdom config file.",
    )
    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib_path,
        description="Legacy calibration file path. Ignored when use_tf_alignment=true.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="integrated_to_init",
        description="Optional remap target for laser odometry.",
    )

    common_parameters = [
        LaunchConfiguration("config_file"),
        {"calibration_file": LaunchConfiguration("calibration_file")},
    ]

    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=common_parameters,
    )

    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=common_parameters,
        remappings=[
            ("laser_odom_to_init", LaunchConfiguration("odom_topic")),
        ],
    )

    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=common_parameters,
    )

    return LaunchDescription([
        config_file_arg,
        calibration_file_arg,
        use_sim_time_arg,
        odom_topic_arg,
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
    ])
