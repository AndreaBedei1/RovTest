from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    endpoint = LaunchConfiguration("mavlink_endpoint")
    heartbeat_timeout_s = LaunchConfiguration("heartbeat_timeout_s")

    config_file = PathJoinSubstitution(
        [FindPackageShare("rov_mavlink_bridge"), "config", "mavlink_bridge.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mavlink_endpoint",
                default_value="udpin:0.0.0.0:14550",
                description="pymavlink connection endpoint.",
            ),
            DeclareLaunchArgument(
                "heartbeat_timeout_s",
                default_value="15.0",
                description="Seconds to wait for the first vehicle heartbeat.",
            ),
            Node(
                package="rov_mavlink_bridge",
                executable="mavlink_bridge_node",
                name="mavlink_bridge",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "mavlink_endpoint": endpoint,
                        "heartbeat_timeout_s": ParameterValue(
                            heartbeat_timeout_s,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )

