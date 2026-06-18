from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    config_file = PathJoinSubstitution(
        [FindPackageShare("rov_peripherals"), "config", "hardware_smoke_test.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="rov",
                description="ROS namespace containing /control peripheral services.",
            ),
            DeclareLaunchArgument(
                "enable_laser_test",
                default_value="false",
                description="Opt-in laser ON smoke test.",
            ),
            DeclareLaunchArgument(
                "enable_gripper_motion",
                default_value="false",
                description="Opt-in gripper open/close smoke test.",
            ),
            DeclareLaunchArgument(
                "lights_percent",
                default_value="5.0",
                description="Low light percentage used during smoke test.",
            ),
            Node(
                package="rov_peripherals",
                executable="peripheral_smoke_test",
                name="peripheral_smoke_test",
                namespace=namespace,
                output="screen",
                parameters=[
                    config_file,
                    {
                        "enable_laser_test": ParameterValue(
                            LaunchConfiguration("enable_laser_test"),
                            value_type=bool,
                        ),
                        "enable_gripper_motion": ParameterValue(
                            LaunchConfiguration("enable_gripper_motion"),
                            value_type=bool,
                        ),
                        "lights_percent": ParameterValue(
                            LaunchConfiguration("lights_percent"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )

