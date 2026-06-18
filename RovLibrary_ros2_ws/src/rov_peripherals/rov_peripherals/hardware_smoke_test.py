"""Hardware smoke test for peripheral-only validation.

This node intentionally calls only public ``/rov/control/...`` peripheral
services exposed by ``rov_control_manager``. It does not arm, change modes, or
send thruster/motion commands.
"""

from __future__ import annotations

import time
from typing import Any

import rclpy
from rclpy.node import Node
from rov_msgs.msg import ConnectionStatus
from rov_msgs.srv import GetCameraTilt, SetCameraTilt, SetLaser, SetLights
from std_srvs.srv import Trigger


class PeripheralSmokeTest(Node):
    """Run a short real-rover peripheral validation sequence."""

    def __init__(self) -> None:
        super().__init__("peripheral_smoke_test")
        self._declare_parameters()
        prefix = str(self.get_parameter("control_service_prefix").value).rstrip("/")
        self._timeout_s = float(self.get_parameter("service_timeout_s").value)
        self._connected = False
        self._connection_text = "no connection status received"

        self._lights = self.create_client(SetLights, f"{prefix}/lights/set")
        self._laser = self.create_client(SetLaser, f"{prefix}/laser/set")
        self._gripper_open = self.create_client(Trigger, f"{prefix}/gripper/open")
        self._gripper_close = self.create_client(Trigger, f"{prefix}/gripper/close")
        self._gripper_stop = self.create_client(Trigger, f"{prefix}/gripper/stop")
        self._tilt_set = self.create_client(SetCameraTilt, f"{prefix}/camera_tilt/set")
        self._tilt_get = self.create_client(GetCameraTilt, f"{prefix}/camera_tilt/get")
        self.create_subscription(
            ConnectionStatus,
            str(self.get_parameter("connection_status_topic").value),
            self._on_connection_status,
            10,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("control_service_prefix", "/rov/control")
        self.declare_parameter("connection_status_topic", "/rov/connection_status")
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("connection_wait_s", 30.0)
        self.declare_parameter("settle_seconds", 0.4)
        self.declare_parameter("lights_percent", 5.0)
        self.declare_parameter("lights_hold_seconds", 0.7)
        self.declare_parameter("enable_laser_test", False)
        self.declare_parameter("laser_hold_seconds", 0.25)
        self.declare_parameter("enable_gripper_motion", False)
        self.declare_parameter("enable_camera_tilt_motion", True)

    def run(self) -> int:
        self.get_logger().info("Starting peripheral-only hardware smoke test.")
        self.get_logger().warning(
            "This test uses only /rov/control peripheral services. "
            "It does not arm, set mode, or command thrusters."
        )

        required = [self._lights, self._gripper_stop, self._tilt_get]
        if bool(self.get_parameter("enable_camera_tilt_motion").value):
            required.append(self._tilt_set)
        if bool(self.get_parameter("enable_laser_test").value):
            required.append(self._laser)
        if bool(self.get_parameter("enable_gripper_motion").value):
            required.extend([self._gripper_open, self._gripper_close])

        if not self._wait_for_clients(required):
            return 2
        if not self._wait_for_connection():
            return 2

        steps = [
            self._check_tilt_state,
            self._test_lights,
            self._test_camera_tilt,
            self._test_gripper,
            self._test_laser,
            self._final_safe_state,
        ]

        for step in steps:
            if not step():
                self.get_logger().error("Peripheral smoke test failed.")
                return 1

        self.get_logger().info("Peripheral smoke test completed successfully.")
        return 0

    def _wait_for_clients(self, clients: list[Any]) -> bool:
        ok = True
        for client in clients:
            if not client.wait_for_service(timeout_sec=self._timeout_s):
                self.get_logger().error(f"Service not available: {client.srv_name}")
                ok = False
        return ok

    def _on_connection_status(self, msg: ConnectionStatus) -> None:
        self._connected = bool(msg.connected)
        self._connection_text = msg.status_text

    def _wait_for_connection(self) -> bool:
        wait_s = max(0.0, float(self.get_parameter("connection_wait_s").value))
        deadline = time.monotonic() + wait_s
        while rclpy.ok() and time.monotonic() < deadline:
            if self._connected:
                self.get_logger().info("MAVLink connection is ready for peripheral smoke test.")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            f"Timed out waiting for MAVLink connection: {self._connection_text}"
        )
        return False

    def _check_tilt_state(self) -> bool:
        request = GetCameraTilt.Request()
        response = self._call(self._tilt_get, request, "camera tilt get")
        return response is not None and bool(response.success)

    def _test_lights(self) -> bool:
        percent = max(0.0, min(20.0, float(self.get_parameter("lights_percent").value)))
        hold_s = max(0.0, float(self.get_parameter("lights_hold_seconds").value))
        self.get_logger().info(f"Testing lights at {percent:.1f}% for {hold_s:.2f}s.")

        on = SetLights.Request()
        on.percent = percent
        on.profile = "out_of_water"
        response = self._call(self._lights, on, "lights on")
        if response is None or not response.success:
            return False

        time.sleep(hold_s)
        off = SetLights.Request()
        off.percent = 0.0
        off.profile = "out_of_water"
        response = self._call(self._lights, off, "lights off")
        return response is not None and bool(response.success)

    def _test_camera_tilt(self) -> bool:
        if not bool(self.get_parameter("enable_camera_tilt_motion").value):
            self.get_logger().info("Camera tilt motion smoke test disabled.")
            return True

        self.get_logger().info("Testing camera tilt center/up/down/center.")
        for command in ["center", "up", "down", "center"]:
            request = SetCameraTilt.Request()
            request.command = command
            request.value = 0
            response = self._call(self._tilt_set, request, f"camera tilt {command}")
            if response is None or not response.success:
                return False
            time.sleep(float(self.get_parameter("settle_seconds").value))
        return True

    def _test_gripper(self) -> bool:
        if not bool(self.get_parameter("enable_gripper_motion").value):
            self.get_logger().info("Gripper motion disabled; sending stop only.")
            response = self._call(self._gripper_stop, Trigger.Request(), "gripper stop")
            return response is not None and bool(response.success)

        self.get_logger().warning("Testing gripper open/close/stop.")
        for label, client in [
            ("gripper open", self._gripper_open),
            ("gripper close", self._gripper_close),
            ("gripper stop", self._gripper_stop),
        ]:
            response = self._call(client, Trigger.Request(), label)
            if response is None or not response.success:
                return False
            time.sleep(float(self.get_parameter("settle_seconds").value))
        return True

    def _test_laser(self) -> bool:
        if not bool(self.get_parameter("enable_laser_test").value):
            self.get_logger().info("Laser smoke test disabled; sending laser OFF only.")
            request = SetLaser.Request()
            request.enabled = False
            request.hold_seconds = 0.0
            request.profile = "out_of_water"
            response = self._call(self._laser, request, "laser off") if self._laser.service_is_ready() else None
            return response is None or bool(response.success)

        hold_s = max(0.05, float(self.get_parameter("laser_hold_seconds").value))
        self.get_logger().warning(f"Testing laser ON for {hold_s:.2f}s, then OFF.")
        request = SetLaser.Request()
        request.enabled = True
        request.hold_seconds = hold_s
        request.profile = "out_of_water"
        response = self._call(self._laser, request, "laser timed on")
        return response is not None and bool(response.success)

    def _final_safe_state(self) -> bool:
        self.get_logger().info("Sending final peripheral safe state: lights off, gripper stop, laser off if available.")
        ok = True

        lights = SetLights.Request()
        lights.percent = 0.0
        lights.profile = "out_of_water"
        response = self._call(self._lights, lights, "final lights off")
        ok = ok and response is not None and bool(response.success)

        response = self._call(self._gripper_stop, Trigger.Request(), "final gripper stop")
        ok = ok and response is not None and bool(response.success)

        if self._laser.service_is_ready():
            laser = SetLaser.Request()
            laser.enabled = False
            laser.hold_seconds = 0.0
            laser.profile = "out_of_water"
            response = self._call(self._laser, laser, "final laser off")
            ok = ok and response is not None and bool(response.success)

        return ok

    def _call(self, client: Any, request: Any, label: str) -> Any | None:
        future = client.call_async(request)
        deadline = time.monotonic() + self._timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

        if not future.done():
            self.get_logger().error(f"{label} timed out.")
            return None

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{label} failed: {exc}")
            return None

        if response is None:
            self.get_logger().error(f"{label} returned no response.")
            return None

        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        if success:
            self.get_logger().info(f"{label}: {message}")
        else:
            self.get_logger().error(f"{label}: {message}")
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PeripheralSmokeTest()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
