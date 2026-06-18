from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    config_file = PathJoinSubstitution(
        [FindPackageShare("rov_control_manager"), "config", "control_manager.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="rov",
                description="ROS namespace for public control services.",
            ),
            Node(
                package="rov_control_manager",
                executable="control_manager_node",
                name="control_manager",
                namespace=namespace,
                output="screen",
                parameters=[config_file],
            ),
        ]
    )

