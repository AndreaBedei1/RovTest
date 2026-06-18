from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    config_file = PathJoinSubstitution(
        [FindPackageShare("rov_peripherals"), "config", "peripherals.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="rov",
                description="ROS namespace for peripheral services.",
            ),
            Node(
                package="rov_peripherals",
                executable="peripherals_node",
                name="peripherals",
                namespace=namespace,
                output="screen",
                parameters=[config_file],
            ),
        ]
    )

