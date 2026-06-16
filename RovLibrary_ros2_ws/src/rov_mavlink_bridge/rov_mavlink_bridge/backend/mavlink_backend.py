"""Reusable MAVLink backend for the ROS 2 bridge.

The commander code is adapted from the original RovLibrary ``mavlink_core`` so
the connection behavior and target-system discovery stay familiar.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

from pymavlink import mavutil

from .telemetry import TelemetryReceiver, TelemetryState


LogCallback = Callable[[str, str], None]


class BlueRovCommander:
    """Small MAVLink command facade for BlueROV2 state actions."""

    def __init__(
        self,
        endpoint: str,
        heartbeat_timeout: float,
        source_system: int,
        source_component: int,
        log_callback: LogCallback | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.heartbeat_timeout = heartbeat_timeout
        self.source_system = source_system
        self.source_component = source_component
        self.log_callback = log_callback
        self.master: Any | None = None

    def _log(self, level: str, message: str) -> None:
        if self.log_callback is not None:
            self.log_callback(level, message)

    def connect(self) -> dict[str, Any]:
        """Open MAVLink connection, wait for vehicle heartbeat, and return it."""
        self._log("info", f"Connecting MAVLink: {self.endpoint}")
        self.master = mavutil.mavlink_connection(
            self.endpoint,
            source_system=self.source_system,
            source_component=self.source_component,
            autoreconnect=True,
        )
        timeout = max(1.0, float(self.heartbeat_timeout))
        deadline = time.monotonic() + timeout
        hb = None
        fallback = None
        while time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
            if msg is None:
                continue
            if getattr(msg, "type", None) == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            fallback = msg
            src_comp = msg.get_srcComponent()
            src_autopilot = getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID)
            if (
                src_comp == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                or src_autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                hb = msg
                self.master.target_system = msg.get_srcSystem()
                self.master.target_component = msg.get_srcComponent()
                break
        if hb is None and fallback is not None:
            hb = fallback
            self.master.target_system = fallback.get_srcSystem()
            self.master.target_component = fallback.get_srcComponent()
            self._log(
                "warning",
                "Autopilot heartbeat not found in time; "
                f"using component {self.master.target_component}.",
            )
        if hb is None:
            raise TimeoutError(
                f"Vehicle heartbeat timeout after {self.heartbeat_timeout}s on {self.endpoint}"
            )
        self._log(
            "info",
            "Connected. "
            f"target_system={self.master.target_system}, "
            f"target_component={self.master.target_component}",
        )
        payload = hb.to_dict()
        payload.pop("mavpackettype", None)
        return payload

    def ensure_connected(self) -> None:
        if self.master is None:
            raise RuntimeError("Not connected. Call connect() first.")

    def arm(self, armed: bool) -> None:
        """Send a MAVLink arm or disarm command."""
        self.ensure_connected()
        assert self.master is not None
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if armed else 0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def set_flight_mode(self, mode_name: str) -> None:
        """Set the autopilot mode by name using pymavlink's mode mapping."""
        self.ensure_connected()
        assert self.master is not None
        requested = mode_name.strip().upper()
        mapping = self.master.mode_mapping() or {}
        if requested not in mapping:
            available = ", ".join(sorted(mapping)) if mapping else "none reported"
            raise ValueError(f"Unsupported flight mode '{requested}'. Available: {available}")
        self.master.set_mode(mapping[requested])


class MavlinkBackend:
    """Connection, telemetry, and safe state-command facade."""

    def __init__(
        self,
        endpoint: str,
        heartbeat_timeout: float = 15.0,
        source_system: int = 255,
        source_component: int = 190,
        log_callback: LogCallback | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.heartbeat_timeout = heartbeat_timeout
        self.state = TelemetryState()
        self._log_callback = log_callback
        self._command_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._receiver: TelemetryReceiver | None = None
        self._connected = False
        self.commander = BlueRovCommander(
            endpoint=endpoint,
            heartbeat_timeout=heartbeat_timeout,
            source_system=source_system,
            source_component=source_component,
            log_callback=log_callback,
        )

    def _log(self, level: str, message: str) -> None:
        if self._log_callback is not None:
            self._log_callback(level, message)

    def connect(self) -> None:
        """Connect MAVLink and start telemetry collection."""
        with self._command_lock:
            initial_heartbeat = self.commander.connect()
            self.state.update("HEARTBEAT", initial_heartbeat)
            self._connected = True
            self._stop_event.clear()
            assert self.commander.master is not None
            self._receiver = TelemetryReceiver(
                self.commander.master,
                self.state,
                self._stop_event,
                on_message=self._on_message,
            )
            self._receiver.start()
        self._log("info", "MAVLink telemetry receiver started.")

    def close(self) -> None:
        """Stop telemetry and close the pymavlink connection when possible."""
        self._stop_event.set()
        if self._receiver is not None:
            self._receiver.join(timeout=2.0)
            self._receiver = None
        master = self.commander.master
        if master is not None and hasattr(master, "close"):
            master.close()
        self._connected = False

    def _on_message(self, message_type: str, payload: dict[str, Any]) -> None:
        if message_type == "HEARTBEAT":
            mode = self.current_vehicle_state().get("mode", "UNKNOWN")
            self._log("debug", f"Heartbeat received, mode={mode}.")

    @property
    def connected_once(self) -> bool:
        return self._connected

    def metrics(self) -> dict[str, Any]:
        return self.state.build_metrics()

    def latest(self, message_type: str) -> dict[str, Any]:
        return self.state.snapshot()["latest"].get(message_type, {})

    def connection_status(self) -> dict[str, Any]:
        metrics = self.metrics()
        heartbeat_age = self.state.age_s("HEARTBEAT")
        heartbeat_seen = heartbeat_age is not None
        heartbeat_fresh = heartbeat_seen and heartbeat_age <= max(1.0, self.heartbeat_timeout * 2.0)
        connected = self._connected and heartbeat_fresh
        if connected:
            status_text = "connected"
        elif self._connected and heartbeat_seen:
            status_text = "heartbeat stale"
        elif self._connected:
            status_text = "waiting for heartbeat"
        else:
            status_text = "disconnected"

        return {
            "connected": connected,
            "heartbeat_seen": heartbeat_seen,
            "endpoint": self.endpoint,
            "status_text": status_text,
            "last_heartbeat_age_s": heartbeat_age if heartbeat_age is not None else -1.0,
            "messages_total": int(metrics.get("messages_total", 0)),
        }

    def current_vehicle_state(self) -> dict[str, Any]:
        heartbeat = self.latest("HEARTBEAT")
        base_mode = _as_int(heartbeat.get("base_mode"), 0)
        custom_mode = _as_int(heartbeat.get("custom_mode"), 0)
        mav_type = _as_int(heartbeat.get("type"), 0)
        system_status = _as_int(heartbeat.get("system_status"), 0)

        target_system = 0
        target_component = 0
        if self.commander.master is not None:
            target_system = _as_int(getattr(self.commander.master, "target_system", 0), 0)
            target_component = _as_int(getattr(self.commander.master, "target_component", 0), 0)

        return {
            "armed": bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
            "mode": self._mode_name_from_custom_mode(custom_mode),
            "mav_type": mav_type,
            "system_status": system_status,
            "base_mode": base_mode,
            "custom_mode": custom_mode,
            "target_system": target_system,
            "target_component": target_component,
        }

    def _mode_name_from_custom_mode(self, custom_mode: int) -> str:
        master = self.commander.master
        if master is None:
            return "UNKNOWN"

        mapping = master.mode_mapping() or {}
        for name, mode_id in mapping.items():
            if int(mode_id) == int(custom_mode):
                return str(name)

        flightmode = getattr(master, "flightmode", None)
        return str(flightmode) if flightmode else "UNKNOWN"

    def arm_vehicle(self) -> None:
        with self._command_lock:
            self.commander.arm(True)
        self._log("info", "Arm command sent.")

    def disarm_vehicle(self) -> None:
        with self._command_lock:
            self.commander.arm(False)
        self._log("info", "Disarm command sent.")

    def set_flight_mode(self, mode_name: str) -> None:
        requested = mode_name.strip().upper()
        with self._command_lock:
            self.commander.set_flight_mode(requested)
        self._log("info", f"Flight mode command sent: {requested}.")


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:  # noqa: BLE001
        return default

