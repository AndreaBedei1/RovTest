from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    endpoint = LaunchConfiguration("mavlink_endpoint")
    enable_laser_test = LaunchConfiguration("enable_laser_test")
    enable_gripper_motion = LaunchConfiguration("enable_gripper_motion")
    lights_percent = LaunchConfiguration("lights_percent")

    stage2_launch = PathJoinSubstitution(
        [FindPackageShare("rov_bringup"), "launch", "rov_stage2.launch.py"]
    )
    smoke_config = PathJoinSubstitution(
        [FindPackageShare("rov_peripherals"), "config", "hardware_smoke_test.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="rov"),
            DeclareLaunchArgument("mavlink_endpoint", default_value="udpin:0.0.0.0:14550"),
            DeclareLaunchArgument("enable_laser_test", default_value="false"),
            DeclareLaunchArgument("enable_gripper_motion", default_value="false"),
            DeclareLaunchArgument("lights_percent", default_value="5.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stage2_launch),
                launch_arguments={
                    "namespace": namespace,
                    "mavlink_endpoint": endpoint,
                }.items(),
            ),
            Node(
                package="rov_peripherals",
                executable="peripheral_smoke_test",
                name="peripheral_smoke_test",
                namespace=namespace,
                output="screen",
                parameters=[
                    smoke_config,
                    {
                        "enable_laser_test": ParameterValue(enable_laser_test, value_type=bool),
                        "enable_gripper_motion": ParameterValue(
                            enable_gripper_motion,
                            value_type=bool,
                        ),
                        "lights_percent": ParameterValue(lights_percent, value_type=float),
                    },
                ],
            ),
        ]
    )

