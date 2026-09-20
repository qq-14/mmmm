import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("localization_health")
    default_config = os.path.join(pkg_share, "config", "localization_health.yaml")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")

    declare_namespace = DeclareLaunchArgument("namespace", default_value="")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_config = DeclareLaunchArgument("config_file", default_value=default_config)

    # 注意：本 launch 通常被包含在已应用 PushRosNamespace 的组里，
    # 因此不再给节点单独设置 namespace，避免命名空间叠加两层。
    health_node = Node(
        package="localization_health",
        executable="health_monitor_node",
        name="localization_health_monitor",
        output="screen",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [declare_namespace, declare_use_sim_time, declare_config, health_node]
    )
