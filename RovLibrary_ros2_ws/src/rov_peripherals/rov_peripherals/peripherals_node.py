"""ROS 2 services for custom ROV peripherals."""

from __future__ import annotations

import time
from typing import Any

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rov_msgs.srv import (
    GetCameraTilt,
    GripperCommand,
    SetCameraTilt,
    SetLaser,
    SetLights,
    SetMountControl,
    SetMountMode,
    SetRcOverride,
    SetRelay,
    SetServo,
)

from .backend import CommandResult, PeripheralBackend

_PARAM_DEFAULT_UNSET = object()


def _coerce_int_array_param(value: Any, default: list[int]) -> list[int]:
    if value is None:
        return list(default)
    if isinstance(value, tuple):
        value = list(value)
    if isinstance(value, list):
        return [int(item) for item in value]
    return [int(value)]


class RosMavlinkCommandPort:
    """Adapter from peripheral backend commands to internal MAVLink services."""

    def __init__(
        self,
        node: Node,
        service_prefix: str,
        timeout_s: float,
        callback_group: ReentrantCallbackGroup | None = None,
    ) -> None:
        self.node = node
        self.service_prefix = service_prefix.rstrip("/")
        self.timeout_s = max(0.1, float(timeout_s))
        self._set_servo = node.create_client(
            SetServo,
            f"{self.service_prefix}/set_servo",
            callback_group=callback_group,
        )
        self._set_relay = node.create_client(
            SetRelay,
            f"{self.service_prefix}/set_relay",
            callback_group=callback_group,
        )
        self._set_rc_override = node.create_client(
            SetRcOverride,
            f"{self.service_prefix}/set_rc_override",
            callback_group=callback_group,
        )
        self._set_mount_mode = node.create_client(
            SetMountMode,
            f"{self.service_prefix}/set_mount_mode",
            callback_group=callback_group,
        )
        self._set_mount_control = node.create_client(
            SetMountControl,
            f"{self.service_prefix}/set_mount_control",
            callback_group=callback_group,
        )

    def set_servo_group(
        self,
        servo_numbers: list[int],
        pwm: int,
        repeat: int = 1,
        interval_s: float = 0.05,
    ) -> CommandResult:
        request = SetServo.Request()
        request.servo_numbers = [int(value) for value in servo_numbers]
        request.pwm = int(pwm)
        request.repeat = int(repeat)
        request.interval_s = float(interval_s)
        return self._call(self._set_servo, request, "set_servo")

    def set_relay(self, relay_number: int, enabled: bool) -> CommandResult:
        request = SetRelay.Request()
        request.relay_number = int(relay_number)
        request.enabled = bool(enabled)
        return self._call(self._set_relay, request, "set_relay")

    def send_rc_override(
        self,
        overrides: dict[int, int | None],
        repeat: int = 3,
        rate_hz: float = 8.0,
    ) -> CommandResult:
        request = SetRcOverride.Request()
        request.channels = [int(channel) for channel in overrides]
        request.pwm_values = [0 if pwm is None else int(pwm) for pwm in overrides.values()]
        request.repeat = int(repeat)
        request.rate_hz = float(rate_hz)
        return self._call(self._set_rc_override, request, "set_rc_override")

    def set_mount_mode(self, mode: int) -> CommandResult:
        request = SetMountMode.Request()
        request.mode = int(mode)
        return self._call(self._set_mount_mode, request, "set_mount_mode")

    def set_mount_pitch(
        self,
        pitch_centideg: int,
        repeat: int = 4,
        rate_hz: float = 20.0,
    ) -> CommandResult:
        request = SetMountControl.Request()
        request.pitch_centideg = int(pitch_centideg)
        request.repeat = int(repeat)
        request.rate_hz = float(rate_hz)
        return self._call(self._set_mount_control, request, "set_mount_control")

    def _call(self, client: Any, request: Any, label: str) -> CommandResult:
        if not client.wait_for_service(timeout_sec=self.timeout_s):
            return CommandResult(False, f"Internal MAVLink service '{label}' is not available.")

        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return CommandResult(False, f"Internal MAVLink service '{label}' timed out.")

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return CommandResult(False, f"Internal MAVLink service '{label}' failed: {exc}")
        if response is None:
            return CommandResult(False, f"Internal MAVLink service '{label}' returned no response.")

        return CommandResult(
            bool(response.success),
            str(response.message),
            int(getattr(response, "commands_sent", 0)),
        )


class PeripheralsNode(Node):
    """Internal services for lights, laser, gripper, and camera tilt."""

    def __init__(self) -> None:
        super().__init__("peripherals")

        self._declare_parameters()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._mavlink_client_group = ReentrantCallbackGroup()
        cfg = self._build_config()
        service_prefix = str(self.get_parameter("mavlink_service_prefix").value)
        timeout_s = float(self.get_parameter("service_timeout_s").value)

        command_port = RosMavlinkCommandPort(
            self,
            service_prefix,
            timeout_s,
            self._mavlink_client_group,
        )
        self._command_port = command_port
        self._backend = PeripheralBackend(cfg, command_port)

        self._services = [
            self.create_service(
                SetLights,
                "internal/peripherals/lights/set",
                self._handle_set_lights,
                callback_group=self._service_group,
            ),
            self.create_service(
                SetLaser,
                "internal/peripherals/laser/set",
                self._handle_set_laser,
                callback_group=self._service_group,
            ),
            self.create_service(
                GripperCommand,
                "internal/peripherals/gripper/command",
                self._handle_gripper_command,
                callback_group=self._service_group,
            ),
            self.create_service(
                SetCameraTilt,
                "internal/peripherals/camera_tilt/set",
                self._handle_set_camera_tilt,
                callback_group=self._service_group,
            ),
            self.create_service(
                GetCameraTilt,
                "internal/peripherals/camera_tilt/get",
                self._handle_get_camera_tilt,
                callback_group=self._service_group,
            ),
        ]

        self.get_logger().info("Peripheral services ready.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("mavlink_service_prefix", "/rov/internal/mavlink")
        self.declare_parameter("service_timeout_s", 3.0)
        self.declare_parameter("lights.default_profile", "out_of_water")
        self.declare_parameter("lights.default_servos", [13])
        self.declare_parameter("lights.off_pwm", 1100)
        self.declare_parameter("lights.out_of_water.max_on_seconds", 2.0)
        self.declare_parameter("lights.in_water.max_on_seconds", 600.0)
        self.declare_parameter("laser.control_mode", "servo")
        self.declare_parameter("laser.relay_numbers", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("laser.servo_outputs", [14])
        self.declare_parameter("laser.on_pwm", 1900)
        self.declare_parameter("laser.off_pwm", 1100)
        self.declare_parameter("laser.command_rate_hz", 20.0)
        self.declare_parameter("laser.command_repeat", 6)
        self.declare_parameter("laser.off_repeat", 10)
        self.declare_parameter("laser.default_profile", "out_of_water")
        self.declare_parameter("laser.out_of_water.max_on_seconds", 10.0)
        self.declare_parameter("laser.in_water.max_on_seconds", 600.0)
        self.declare_parameter("gripper.control_mode", "servo")
        self.declare_parameter("gripper.servo_output", 9)
        self.declare_parameter("gripper.open_pwm", 1900)
        self.declare_parameter("gripper.close_pwm", 1100)
        self.declare_parameter("gripper.neutral_pwm", 1500)
        self.declare_parameter("gripper.neutral_hold_seconds", 0.4)
        self.declare_parameter("gripper.command_rate_hz", 20.0)
        self.declare_parameter("gripper.command_repeat", 6)
        self.declare_parameter("camera.tilt_control_mode", "mount")
        self.declare_parameter("camera.tilt_rc_channel", 8)
        self.declare_parameter("camera.tilt_min_pwm", 1100)
        self.declare_parameter("camera.tilt_max_pwm", 1900)
        self.declare_parameter("camera.tilt_neutral_pwm", 1500)
        self.declare_parameter("camera.tilt_step_pwm", 30)
        self.declare_parameter("camera.tilt_min_centideg", -4500)
        self.declare_parameter("camera.tilt_max_centideg", 4500)
        self.declare_parameter("camera.tilt_neutral_centideg", 0)
        self.declare_parameter("camera.tilt_step_centideg", 300)
        self.declare_parameter("camera.invert_tilt", False)
        self.declare_parameter("camera.send_repeat", 4)
        self.declare_parameter("camera.send_rate_hz", 20.0)

    def _build_config(self) -> dict[str, Any]:
        return {
            "lights": {
                "default_profile": self._param("lights.default_profile"),
                "default_servos": self._int_array_param("lights.default_servos", [13]),
                "off_pwm": self._param("lights.off_pwm"),
                "safety_profiles": {
                    "out_of_water": {
                        "max_on_seconds": self._param("lights.out_of_water.max_on_seconds"),
                    },
                    "in_water": {
                        "max_on_seconds": self._param("lights.in_water.max_on_seconds"),
                    },
                },
            },
            "laser": {
                "control_mode": self._param("laser.control_mode"),
                "relay_numbers": self._int_array_param("laser.relay_numbers", []),
                "servo_outputs": self._int_array_param("laser.servo_outputs", [14]),
                "on_pwm": self._param("laser.on_pwm"),
                "off_pwm": self._param("laser.off_pwm"),
                "command_rate_hz": self._param("laser.command_rate_hz"),
                "command_repeat": self._param("laser.command_repeat"),
                "off_repeat": self._param("laser.off_repeat"),
                "default_profile": self._param("laser.default_profile"),
                "safety_profiles": {
                    "out_of_water": {
                        "max_on_seconds": self._param("laser.out_of_water.max_on_seconds"),
                    },
                    "in_water": {
                        "max_on_seconds": self._param("laser.in_water.max_on_seconds"),
                    },
                },
            },
            "gripper": {
                "control_mode": self._param("gripper.control_mode"),
                "servo_output": self._param("gripper.servo_output"),
                "open_pwm": self._param("gripper.open_pwm"),
                "close_pwm": self._param("gripper.close_pwm"),
                "neutral_pwm": self._param("gripper.neutral_pwm"),
                "neutral_hold_seconds": self._param("gripper.neutral_hold_seconds"),
                "command_rate_hz": self._param("gripper.command_rate_hz"),
                "command_repeat": self._param("gripper.command_repeat"),
            },
            "camera": {
                "tilt_control_mode": self._param("camera.tilt_control_mode"),
                "tilt_rc_channel": self._param("camera.tilt_rc_channel"),
                "tilt_min_pwm": self._param("camera.tilt_min_pwm"),
                "tilt_max_pwm": self._param("camera.tilt_max_pwm"),
                "tilt_neutral_pwm": self._param("camera.tilt_neutral_pwm"),
                "tilt_step_pwm": self._param("camera.tilt_step_pwm"),
                "tilt_min_centideg": self._param("camera.tilt_min_centideg"),
                "tilt_max_centideg": self._param("camera.tilt_max_centideg"),
                "tilt_neutral_centideg": self._param("camera.tilt_neutral_centideg"),
                "tilt_step_centideg": self._param("camera.tilt_step_centideg"),
                "invert_tilt": self._param("camera.invert_tilt"),
                "send_repeat": self._param("camera.send_repeat"),
                "send_rate_hz": self._param("camera.send_rate_hz"),
            },
        }

    def _param(self, name: str, default: Any = _PARAM_DEFAULT_UNSET) -> Any:
        try:
            return self.get_parameter(name).value
        except ParameterUninitializedException:
            if default is _PARAM_DEFAULT_UNSET:
                raise
            self.get_logger().debug(
                f"Parameter '{name}' is uninitialized; using default {default!r}."
            )
            return default

    def _int_array_param(self, name: str, default: list[int]) -> list[int]:
        value = self._param(name, list(default))
        try:
            return _coerce_int_array_param(value, default)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Parameter '{name}' is not a valid integer array ({exc}); "
                f"using default {default!r}."
            )
            return list(default)

    def _handle_set_lights(
        self,
        request: SetLights.Request,
        response: SetLights.Response,
    ) -> SetLights.Response:
        result = self._backend.set_lights_percent(
            float(request.percent),
            profile=request.profile.strip() or None,
        )
        response.success = result.success
        response.message = result.message
        response.applied_percent = self._backend.state.lights_percent
        return response

    def _handle_set_laser(
        self,
        request: SetLaser.Request,
        response: SetLaser.Response,
    ) -> SetLaser.Response:
        result = self._backend.set_laser(
            bool(request.enabled),
            hold_seconds=float(request.hold_seconds),
            profile=request.profile.strip() or None,
        )
        response.success = result.success
        response.message = result.message
        return response

    def _handle_gripper_command(
        self,
        request: GripperCommand.Request,
        response: GripperCommand.Response,
    ) -> GripperCommand.Response:
        result = self._backend.gripper_command(
            str(request.command),
            pulse_seconds=float(request.pulse_seconds),
        )
        response.success = result.success
        response.message = result.message
        response.state = self._backend.state.gripper_state
        return response

    def _handle_set_camera_tilt(
        self,
        request: SetCameraTilt.Request,
        response: SetCameraTilt.Response,
    ) -> SetCameraTilt.Response:
        result, tilt = self._backend.set_camera_tilt(str(request.command), int(request.value))
        response.success = result.success
        response.message = result.message
        response.mode = tilt.mode
        response.tilt_pwm = tilt.tilt_pwm
        response.tilt_centideg = tilt.tilt_centideg
        return response

    def _handle_get_camera_tilt(
        self,
        _request: GetCameraTilt.Request,
        response: GetCameraTilt.Response,
    ) -> GetCameraTilt.Response:
        tilt = self._backend.get_camera_tilt()
        response.success = True
        response.message = "Camera tilt state returned."
        response.mode = tilt.mode
        response.tilt_pwm = tilt.tilt_pwm
        response.tilt_centideg = tilt.tilt_centideg
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PeripheralsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
