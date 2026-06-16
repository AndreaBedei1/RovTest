"""ROS 2 node that publishes MAVLink telemetry and exposes safe services."""

from __future__ import annotations

import math
from typing import Any

from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.node import Node
from rov_msgs.msg import ConnectionStatus, VehicleState
from rov_msgs.srv import SetFlightMode
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from .backend import MavlinkBackend


DEFAULT_ALLOWED_MODES = [
    "MANUAL",
    "STABILIZE",
    "DEPTH_HOLD",
    "POSHOLD",
    "ALT_HOLD",
    "SURFACE",
]


class MavlinkBridgeNode(Node):
    """Bridge MAVLink vehicle state into ROS 2 topics and services."""

    def __init__(self) -> None:
        super().__init__("mavlink_bridge")

        self.declare_parameter("mavlink_endpoint", "udpin:0.0.0.0:14550")
        self.declare_parameter("heartbeat_timeout_s", 15.0)
        self.declare_parameter("source_system", 255)
        self.declare_parameter("source_component", 190)
        self.declare_parameter("telemetry_publish_hz", 5.0)
        self.declare_parameter("connection_publish_hz", 1.0)
        self.declare_parameter("connect_on_startup", True)
        self.declare_parameter("allowed_modes", DEFAULT_ALLOWED_MODES)

        endpoint = str(self.get_parameter("mavlink_endpoint").value)
        heartbeat_timeout_s = float(self.get_parameter("heartbeat_timeout_s").value)
        source_system = int(self.get_parameter("source_system").value)
        source_component = int(self.get_parameter("source_component").value)
        telemetry_publish_hz = float(self.get_parameter("telemetry_publish_hz").value)
        connection_publish_hz = float(self.get_parameter("connection_publish_hz").value)
        connect_on_startup = bool(self.get_parameter("connect_on_startup").value)
        self._allowed_modes = [
            str(mode).strip().upper()
            for mode in self.get_parameter("allowed_modes").value
            if str(mode).strip()
        ]

        self._backend = MavlinkBackend(
            endpoint=endpoint,
            heartbeat_timeout=heartbeat_timeout_s,
            source_system=source_system,
            source_component=source_component,
            log_callback=self._backend_log,
        )

        self._connection_pub = self.create_publisher(
            ConnectionStatus,
            "connection_status",
            10,
        )
        self._vehicle_state_pub = self.create_publisher(VehicleState, "vehicle_state", 10)
        self._battery_pub = self.create_publisher(BatteryState, "battery", 10)
        self._attitude_pub = self.create_publisher(Imu, "attitude", 10)
        self._depth_pub = self.create_publisher(Float32, "depth", 10)
        self._altitude_pub = self.create_publisher(Float32, "altitude", 10)

        self.create_service(Trigger, "arm", self._handle_arm)
        self.create_service(Trigger, "disarm", self._handle_disarm)
        self.create_service(SetFlightMode, "set_flight_mode", self._handle_set_flight_mode)

        self.create_timer(
            1.0 / max(0.1, telemetry_publish_hz),
            self._publish_telemetry,
        )
        self.create_timer(
            1.0 / max(0.1, connection_publish_hz),
            self._publish_connection_status,
        )

        self.get_logger().info(
            "MAVLink bridge ready. "
            f"endpoint={endpoint}, telemetry_publish_hz={telemetry_publish_hz}"
        )

        if connect_on_startup:
            self._connect_backend()
        else:
            self.get_logger().warning("connect_on_startup is false; telemetry will stay disconnected.")

    def destroy_node(self) -> bool:
        self._backend.close()
        return super().destroy_node()

    def _backend_log(self, level: str, message: str) -> None:
        logger = self.get_logger()
        if level == "debug":
            logger.debug(message)
        elif level in {"warn", "warning"}:
            logger.warning(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.info(message)

    def _connect_backend(self) -> None:
        try:
            self._backend.connect()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"MAVLink connection failed: {exc}")

    def _publish_connection_status(self) -> None:
        stamp = self.get_clock().now().to_msg()
        status = self._backend.connection_status()

        msg = ConnectionStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = "rov"
        msg.connected = bool(status["connected"])
        msg.heartbeat_seen = bool(status["heartbeat_seen"])
        msg.endpoint = str(status["endpoint"])
        msg.status_text = str(status["status_text"])
        msg.last_heartbeat_age_s = float(status["last_heartbeat_age_s"])
        msg.messages_total = int(status["messages_total"])
        self._connection_pub.publish(msg)

    def _publish_telemetry(self) -> None:
        status = self._backend.connection_status()
        if not status["heartbeat_seen"]:
            return

        stamp = self.get_clock().now().to_msg()
        metrics = self._backend.metrics()
        latest = metrics.get("latest", {})

        self._publish_vehicle_state(stamp)
        self._publish_battery(stamp, metrics)

        attitude = latest.get("ATTITUDE", {})
        if attitude:
            self._publish_attitude(stamp, attitude)

        altitude_m = metrics.get("altitude_m")
        if altitude_m is not None:
            self._altitude_pub.publish(Float32(data=float(altitude_m)))

        depth_m = metrics.get("depth_m")
        if depth_m is not None:
            self._depth_pub.publish(Float32(data=float(depth_m)))

    def _publish_vehicle_state(self, stamp: Any) -> None:
        state = self._backend.current_vehicle_state()

        msg = VehicleState()
        msg.header.stamp = stamp
        msg.header.frame_id = "rov"
        msg.armed = bool(state["armed"])
        msg.mode = str(state["mode"])
        msg.mav_type = int(state["mav_type"])
        msg.system_status = int(state["system_status"])
        msg.base_mode = int(state["base_mode"])
        msg.custom_mode = int(state["custom_mode"])
        msg.target_system = int(state["target_system"])
        msg.target_component = int(state["target_component"])
        self._vehicle_state_pub.publish(msg)

    def _publish_battery(self, stamp: Any, metrics: dict[str, Any]) -> None:
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = "rov"
        msg.voltage = _float_or_nan(metrics.get("voltage_v"))
        msg.current = _float_or_nan(metrics.get("current_a"))
        remaining = metrics.get("battery_remaining")
        msg.percentage = float(remaining) / 100.0 if remaining is not None else math.nan
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self._battery_pub.publish(msg)

    def _publish_attitude(self, stamp: Any, attitude: dict[str, Any]) -> None:
        roll = _maybe_float(attitude.get("roll"))
        pitch = _maybe_float(attitude.get("pitch"))
        yaw = _maybe_float(attitude.get("yaw"))
        if roll is None or pitch is None or yaw is None:
            return

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "rov/base_link"
        msg.orientation = _quaternion_from_euler(roll, pitch, yaw)
        msg.angular_velocity.x = _float_or_zero(attitude.get("rollspeed"))
        msg.angular_velocity.y = _float_or_zero(attitude.get("pitchspeed"))
        msg.angular_velocity.z = _float_or_zero(attitude.get("yawspeed"))
        self._attitude_pub.publish(msg)

    def _handle_arm(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().warning("Arm service requested.")
        try:
            self._backend.arm_vehicle()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"Arm command failed: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = "Arm command sent."
        return response

    def _handle_disarm(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self.get_logger().warning("Disarm service requested.")
        try:
            self._backend.disarm_vehicle()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"Disarm command failed: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = "Disarm command sent."
        return response

    def _handle_set_flight_mode(
        self,
        request: SetFlightMode.Request,
        response: SetFlightMode.Response,
    ) -> SetFlightMode.Response:
        requested = request.mode.strip().upper()
        if not requested:
            response.success = False
            response.message = "Flight mode cannot be empty."
            return response

        if self._allowed_modes and requested not in self._allowed_modes:
            response.success = False
            response.message = (
                f"Flight mode '{requested}' is not in the configured allow-list: "
                f"{', '.join(self._allowed_modes)}"
            )
            self.get_logger().warning(response.message)
            return response

        self.get_logger().warning(f"Flight mode service requested: {requested}.")
        try:
            self._backend.set_flight_mode(requested)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"Flight mode command failed: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Flight mode command sent: {requested}."
        return response


def _maybe_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:  # noqa: BLE001
        return None


def _float_or_nan(value: Any) -> float:
    parsed = _maybe_float(value)
    return parsed if parsed is not None else math.nan


def _float_or_zero(value: Any) -> float:
    parsed = _maybe_float(value)
    return parsed if parsed is not None else 0.0


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

