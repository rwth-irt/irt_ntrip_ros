import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    logger = LaunchConfiguration("log_level")

    config = os.path.join(
        get_package_share_directory('ntrip_ros'),
        'config',
        'station_tracker_config.yaml'
    )

    node = Node(
        package='ntrip_ros',
        name='ref_station_tracker',
        executable='ref_station_tracker_node.py',
        parameters=[config],
        arguments=['--ros-args', '--log-level', logger],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    ), node])
