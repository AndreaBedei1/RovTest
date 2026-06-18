"""Public safe command router for vehicle and peripheral control."""

from __future__ import annotations

import threading
import time
from typing import Any

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rov_msgs.msg import ConnectionStatus, VehicleState
from rov_msgs.srv import (
    GetCameraTilt,
    GripperCommand,
    SetCameraTilt,
    SetFlightMode,
    SetLaser,
    SetLights,
)
from std_srvs.srv import Trigger

from .safety import (
    SafetySnapshot,
    camera_tilt_requires_armed,
    gripper_requires_armed,
    laser_requires_armed,
    lights_requires_armed,
)


class ControlManagerNode(Node):
    """Only public node that forwards high-level control requests."""

    def __init__(self) -> None:
        super().__init__("control_manager")

        self._declare_parameters()
        self._lock = threading.Lock()
        self._subscription_group = MutuallyExclusiveCallbackGroup()
        self._public_service_group = MutuallyExclusiveCallbackGroup()
        self._internal_client_group = ReentrantCallbackGroup()
        self._safety = SafetySnapshot()
        self._timeout_s = float(self.get_parameter("service_timeout_s").value)
        self._require_armed_lights = bool(self.get_parameter("require_armed_for_lights_on").value)
        self._require_armed_laser = bool(self.get_parameter("require_armed_for_laser_on").value)
        self._require_armed_gripper = bool(self.get_parameter("require_armed_for_gripper_motion").value)
        self._require_armed_tilt = bool(self.get_parameter("require_armed_for_camera_tilt").value)

        self.create_subscription(
            ConnectionStatus,
            str(self.get_parameter("connection_status_topic").value),
            self._on_connection_status,
            10,
            callback_group=self._subscription_group,
        )
        self.create_subscription(
            VehicleState,
            str(self.get_parameter("vehicle_state_topic").value),
            self._on_vehicle_state,
            10,
            callback_group=self._subscription_group,
        )

        self._arm_client = self.create_client(
            Trigger,
            str(self.get_parameter("internal_arm_service").value),
            callback_group=self._internal_client_group,
        )
        self._disarm_client = self.create_client(
            Trigger,
            str(self.get_parameter("internal_disarm_service").value),
            callback_group=self._internal_client_group,
        )
        self._set_mode_client = self.create_client(
            SetFlightMode,
            str(self.get_parameter("internal_set_flight_mode_service").value),
            callback_group=self._internal_client_group,
        )
        prefix = str(self.get_parameter("internal_peripheral_service_prefix").value).rstrip("/")
        self._lights_client = self.create_client(
            SetLights,
            f"{prefix}/lights/set",
            callback_group=self._internal_client_group,
        )
        self._laser_client = self.create_client(
            SetLaser,
            f"{prefix}/laser/set",
            callback_group=self._internal_client_group,
        )
        self._gripper_client = self.create_client(
            GripperCommand,
            f"{prefix}/gripper/command",
            callback_group=self._internal_client_group,
        )
        self._tilt_set_client = self.create_client(
            SetCameraTilt,
            f"{prefix}/camera_tilt/set",
            callback_group=self._internal_client_group,
        )
        self._tilt_get_client = self.create_client(
            GetCameraTilt,
            f"{prefix}/camera_tilt/get",
            callback_group=self._internal_client_group,
        )

        self._public_services = [
            self.create_service(
                Trigger,
                "control/arm",
                self._handle_arm,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                Trigger,
                "control/disarm",
                self._handle_disarm,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                SetFlightMode,
                "control/set_flight_mode",
                self._handle_set_flight_mode,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                SetLights,
                "control/lights/set",
                self._handle_set_lights,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                SetLaser,
                "control/laser/set",
                self._handle_set_laser,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                Trigger,
                "control/gripper/open",
                self._handle_gripper_open,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                Trigger,
                "control/gripper/close",
                self._handle_gripper_close,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                Trigger,
                "control/gripper/stop",
                self._handle_gripper_stop,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                SetCameraTilt,
                "control/camera_tilt/set",
                self._handle_set_camera_tilt,
                callback_group=self._public_service_group,
            ),
            self.create_service(
                GetCameraTilt,
                "control/camera_tilt/get",
                self._handle_get_camera_tilt,
                callback_group=self._public_service_group,
            ),
        ]

        self.get_logger().info("Control manager ready. Public command services are under control/*.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("service_timeout_s", 3.0)
        self.declare_parameter("connection_status_topic", "/rov/connection_status")
        self.declare_parameter("vehicle_state_topic", "/rov/vehicle_state")
        self.declare_parameter("internal_arm_service", "/rov/internal/arm")
        self.declare_parameter("internal_disarm_service", "/rov/internal/disarm")
        self.declare_parameter("internal_set_flight_mode_service", "/rov/internal/set_flight_mode")
        self.declare_parameter("internal_peripheral_service_prefix", "/rov/internal/peripherals")
        self.declare_parameter("require_armed_for_lights_on", False)
        self.declare_parameter("require_armed_for_laser_on", False)
        self.declare_parameter("require_armed_for_gripper_motion", False)
        self.declare_parameter("require_armed_for_camera_tilt", False)
        self.declare_parameter("default_gripper_pulse_seconds", 0.15)

    def _on_connection_status(self, msg: ConnectionStatus) -> None:
        with self._lock:
            self._safety.connection_seen = True
            self._safety.connected = bool(msg.connected)
            self._safety.connection_text = msg.status_text

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        with self._lock:
            self._safety.vehicle_seen = True
            self._safety.armed = bool(msg.armed)

    def _snapshot(self) -> SafetySnapshot:
        with self._lock:
            return SafetySnapshot(
                connection_seen=self._safety.connection_seen,
                connected=self._safety.connected,
                connection_text=self._safety.connection_text,
                vehicle_seen=self._safety.vehicle_seen,
                armed=self._safety.armed,
            )

    def _handle_arm(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        decision = self._snapshot().require_connected("arm")
        if not decision.allowed:
            return _trigger_reject(response, decision.message)
        forwarded = self._forward(self._arm_client, Trigger.Request(), "arm")
        response.success = forwarded.success
        response.message = forwarded.message
        return response

    def _handle_disarm(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        decision = self._snapshot().require_connected("disarm")
        if not decision.allowed:
            return _trigger_reject(response, decision.message)
        forwarded = self._forward(self._disarm_client, Trigger.Request(), "disarm")
        response.success = forwarded.success
        response.message = forwarded.message
        return response

    def _handle_set_flight_mode(
        self,
        request: SetFlightMode.Request,
        response: SetFlightMode.Response,
    ) -> SetFlightMode.Response:
        decision = self._snapshot().require_connected("set flight mode")
        if not decision.allowed:
            response.success = False
            response.message = decision.message
            return response
        forwarded = self._forward(self._set_mode_client, request, "set flight mode")
        response.success = forwarded.success
        response.message = forwarded.message
        return response

    def _handle_set_lights(
        self,
        request: SetLights.Request,
        response: SetLights.Response,
    ) -> SetLights.Response:
        decision = self._snapshot().check(
            "set lights",
            require_armed=lights_requires_armed(request.percent, self._require_armed_lights),
        )
        if not decision.allowed:
            response.success = False
            response.message = decision.message
            response.applied_percent = 0.0
            return response
        forwarded = self._forward(self._lights_client, request, "set lights")
        response.success = forwarded.success
        response.message = forwarded.message
        response.applied_percent = float(getattr(forwarded.response, "applied_percent", 0.0))
        return response

    def _handle_set_laser(
        self,
        request: SetLaser.Request,
        response: SetLaser.Response,
    ) -> SetLaser.Response:
        decision = self._snapshot().check(
            "set laser",
            require_armed=laser_requires_armed(request.enabled, self._require_armed_laser),
        )
        if not decision.allowed:
            response.success = False
            response.message = decision.message
            return response
        forwarded = self._forward(self._laser_client, request, "set laser")
        response.success = forwarded.success
        response.message = forwarded.message
        return response

    def _handle_gripper_open(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        return self._forward_gripper_trigger("open", response)

    def _handle_gripper_close(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        return self._forward_gripper_trigger("close", response)

    def _handle_gripper_stop(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        return self._forward_gripper_trigger("stop", response)

    def _forward_gripper_trigger(self, command: str, response: Trigger.Response) -> Trigger.Response:
        decision = self._snapshot().check(
            f"gripper {command}",
            require_armed=gripper_requires_armed(command, self._require_armed_gripper),
        )
        if not decision.allowed:
            return _trigger_reject(response, decision.message)

        request = GripperCommand.Request()
        request.command = command
        request.pulse_seconds = float(self.get_parameter("default_gripper_pulse_seconds").value)
        forwarded = self._forward(self._gripper_client, request, f"gripper {command}")
        response.success = forwarded.success
        response.message = forwarded.message
        return response

    def _handle_set_camera_tilt(
        self,
        request: SetCameraTilt.Request,
        response: SetCameraTilt.Response,
    ) -> SetCameraTilt.Response:
        decision = self._snapshot().check(
            "set camera tilt",
            require_armed=camera_tilt_requires_armed(self._require_armed_tilt),
        )
        if not decision.allowed:
            response.success = False
            response.message = decision.message
            return response
        forwarded = self._forward(self._tilt_set_client, request, "set camera tilt")
        response.success = forwarded.success
        response.message = forwarded.message
        response.mode = str(getattr(forwarded.response, "mode", ""))
        response.tilt_pwm = int(getattr(forwarded.response, "tilt_pwm", 0))
        response.tilt_centideg = int(getattr(forwarded.response, "tilt_centideg", 0))
        return response

    def _handle_get_camera_tilt(
        self,
        request: GetCameraTilt.Request,
        response: GetCameraTilt.Response,
    ) -> GetCameraTilt.Response:
        forwarded = self._forward(self._tilt_get_client, request, "get camera tilt")
        response.success = forwarded.success
        response.message = forwarded.message
        response.mode = str(getattr(forwarded.response, "mode", ""))
        response.tilt_pwm = int(getattr(forwarded.response, "tilt_pwm", 0))
        response.tilt_centideg = int(getattr(forwarded.response, "tilt_centideg", 0))
        return response

    def _forward(self, client: Any, request: Any, label: str) -> "_ForwardResult":
        if not client.wait_for_service(timeout_sec=self._timeout_s):
            return _ForwardResult(False, f"Internal service for {label} is not available.", None)

        future = client.call_async(request)
        deadline = time.monotonic() + self._timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return _ForwardResult(False, f"Internal service for {label} timed out.", None)

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return _ForwardResult(False, f"Internal service for {label} failed: {exc}", None)
        if response is None:
            return _ForwardResult(False, f"Internal service for {label} returned no response.", None)
        return _ForwardResult(bool(response.success), str(response.message), response)


class _ForwardResult:
    def __init__(self, success: bool, message: str, response: Any) -> None:
        self.success = success
        self.message = message
        self.response = response


def _trigger_reject(response: Trigger.Response, message: str) -> Trigger.Response:
    response.success = False
    response.message = message
    return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ControlManagerNode()
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
