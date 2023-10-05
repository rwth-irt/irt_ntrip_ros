import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "ntrip_ros"


def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory(PKG), "config", cfg)


def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)


def get_params(cfg):
    return load_yaml(get_cfg_path(cfg))


def get_override_params():
    try:
        m1com_override_path = os.environ['PARAM_OVERRIDES_PATH']
    except KeyError:  # No overrides specified.
        return {}

    return load_yaml(m1com_override_path)


def arg(name, default_value, description):
    return DeclareLaunchArgument(name=name, description=description, default_value=default_value)


def generate_launch_description():
    logger = LaunchConfiguration("log_level")
    node = Node(
        package=PKG,
        namespace='irt/',
        name='ntrip_ros',
        executable='ntrip_ros_node',
        parameters=[
            {"ntrip_server_uri": "sapos-nw-ntrip.de"},
            {"ntrip_server_port": 2101},
            {"ntrip_username": "nw-710989"},
            {"ntrip_password": "railir"},
            {"ntrip_mountpoint": "VRS_3_3G_NW"},
            {"update_GGA_period": 10},
            {"start_llh": [50.776298, 6.083714, 220.3385]},
            {"bachmann_udp_ip": "192.168.31.12"},
            {"bachmann_udp_port": 10021}
        ],
        arguments=['--ros-args', '--log-level', logger],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    ), node])
