from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    endpoint = LaunchConfiguration("mavlink_endpoint")
    heartbeat_timeout_s = LaunchConfiguration("heartbeat_timeout_s")
    telemetry_publish_hz = LaunchConfiguration("telemetry_publish_hz")

    config_file = PathJoinSubstitution(
        [FindPackageShare("rov_bringup"), "config", "stage2.yaml"]
    )

    bridge_remaps = [
        ("arm", "internal/arm"),
        ("disarm", "internal/disarm"),
        ("set_flight_mode", "internal/set_flight_mode"),
        ("mavlink/set_servo", "internal/mavlink/set_servo"),
        ("mavlink/set_relay", "internal/mavlink/set_relay"),
        ("mavlink/set_rc_override", "internal/mavlink/set_rc_override"),
        ("mavlink/set_mount_mode", "internal/mavlink/set_mount_mode"),
        ("mavlink/set_mount_control", "internal/mavlink/set_mount_control"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="rov",
                description="ROS namespace for the ROV stack.",
            ),
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
            DeclareLaunchArgument(
                "telemetry_publish_hz",
                default_value="5.0",
                description="Telemetry publish frequency.",
            ),
            Node(
                package="rov_mavlink_bridge",
                executable="mavlink_bridge_node",
                name="mavlink_bridge",
                namespace=namespace,
                output="screen",
                parameters=[
                    config_file,
                    {
                        "mavlink_endpoint": endpoint,
                        "heartbeat_timeout_s": ParameterValue(
                            heartbeat_timeout_s,
                            value_type=float,
                        ),
                        "telemetry_publish_hz": ParameterValue(
                            telemetry_publish_hz,
                            value_type=float,
                        ),
                    },
                ],
                remappings=bridge_remaps,
            ),
            Node(
                package="rov_peripherals",
                executable="peripherals_node",
                name="peripherals",
                namespace=namespace,
                output="screen",
                parameters=[config_file],
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

