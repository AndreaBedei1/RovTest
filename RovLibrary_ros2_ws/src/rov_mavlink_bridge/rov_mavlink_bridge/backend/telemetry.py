"""Telemetry receiver thread and snapshot helpers.

This is adapted from the original RovLibrary ``rovlib.telemetry`` module and
kept independent from ROS 2 so it can be reused and tested directly.
"""

from __future__ import annotations

from collections import Counter
import threading
import time
from typing import Any, Callable


def _safe_float(value: Any, scale: float = 1.0) -> float | None:
    try:
        if value is None:
            return None
        return float(value) * scale
    except Exception:  # noqa: BLE001
        return None


def _valid_battery_remaining(value: Any) -> int | None:
    try:
        remaining = int(value)
    except Exception:  # noqa: BLE001
        return None
    if remaining < 0 or remaining > 100:
        return None
    return remaining


def _battery_voltage_from_cells(payload: dict[str, Any]) -> float | None:
    voltages = payload.get("voltages")
    if not isinstance(voltages, list):
        return None

    valid_mv = []
    for raw in voltages:
        try:
            value = int(raw)
        except Exception:  # noqa: BLE001
            continue
        if 0 < value < 65535:
            valid_mv.append(value)

    if not valid_mv:
        return None
    return sum(valid_mv) * 0.001


class TelemetryState:
    """Thread-safe container for latest MAVLink messages."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.latest: dict[str, dict[str, Any]] = {}
        self.counts: Counter[str] = Counter()
        self.received_monotonic: dict[str, float] = {}
        self.started_ns = time.time_ns()

    def update(self, message_type: str, payload: dict[str, Any]) -> None:
        """Store latest payload and increase per-type counter."""
        with self._lock:
            self.latest[message_type] = dict(payload)
            self.counts[message_type] += 1
            self.received_monotonic[message_type] = time.monotonic()

    def snapshot(self) -> dict[str, Any]:
        """Return a deep-enough snapshot for ROS message building."""
        with self._lock:
            latest_copy = {k: dict(v) for k, v in self.latest.items()}
            counts_copy = dict(self.counts)
            received_copy = dict(self.received_monotonic)
        uptime_s = max((time.time_ns() - self.started_ns) / 1e9, 1e-6)
        return {
            "latest": latest_copy,
            "counts": counts_copy,
            "received_monotonic": received_copy,
            "uptime_s": uptime_s,
        }

    def age_s(self, message_type: str) -> float | None:
        """Return seconds since the last message of this type."""
        with self._lock:
            received = self.received_monotonic.get(message_type)
        if received is None:
            return None
        return max(0.0, time.monotonic() - received)

    def build_metrics(self) -> dict[str, Any]:
        """Derive commonly-used human-readable metrics."""
        snap = self.snapshot()
        latest = snap["latest"]
        sys_status = latest.get("SYS_STATUS", {})
        attitude = latest.get("ATTITUDE", {})
        vfr = latest.get("VFR_HUD", {})
        battery = latest.get("BATTERY_STATUS", {})
        pressure = latest.get("SCALED_PRESSURE", {})
        pressure2 = latest.get("SCALED_PRESSURE2", {})
        rc = latest.get("RC_CHANNELS", {})

        voltage_v = _safe_float(sys_status.get("voltage_battery"), scale=0.001)
        if voltage_v is None:
            voltage_v = _battery_voltage_from_cells(battery)

        current_a = _safe_float(sys_status.get("current_battery"), scale=0.01)
        if current_a is None:
            current_a = _safe_float(battery.get("current_battery"), scale=0.01)

        roll_rad = _safe_float(attitude.get("roll"))
        pitch_rad = _safe_float(attitude.get("pitch"))
        yaw_rad = _safe_float(attitude.get("yaw"))
        roll_deg = _safe_float(attitude.get("roll"), scale=57.2957795)
        pitch_deg = _safe_float(attitude.get("pitch"), scale=57.2957795)
        yaw_deg = _safe_float(attitude.get("yaw"), scale=57.2957795)

        altitude_m = _safe_float(vfr.get("alt")) if "alt" in vfr else None
        depth_m = max(0.0, -altitude_m) if altitude_m is not None else None

        water_temp_c = _safe_float(pressure2.get("temperature"), scale=0.01)
        internal_temp_c = _safe_float(pressure.get("temperature"), scale=0.01)
        battery_remaining = _valid_battery_remaining(battery.get("battery_remaining"))

        return {
            "uptime_s": snap["uptime_s"],
            "message_types": len(snap["counts"]),
            "messages_total": int(sum(snap["counts"].values())),
            "voltage_v": voltage_v,
            "current_a": current_a,
            "altitude_m": altitude_m,
            "depth_m": depth_m,
            "roll_rad": roll_rad,
            "pitch_rad": pitch_rad,
            "yaw_rad": yaw_rad,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "yaw_deg": yaw_deg,
            "heading": vfr.get("heading"),
            "groundspeed": vfr.get("groundspeed"),
            "water_temp_c": water_temp_c,
            "internal_temp_c": internal_temp_c,
            "battery_remaining": battery_remaining,
            "rc": rc,
            "latest": latest,
            "counts": snap["counts"],
        }


class TelemetryReceiver(threading.Thread):
    """Background thread that consumes MAVLink messages from one connection."""

    def __init__(
        self,
        master: Any,
        state: TelemetryState,
        stop_event: threading.Event,
        on_message: Callable[[str, dict[str, Any]], None] | None = None,
    ) -> None:
        super().__init__(daemon=True, name="telemetry-receiver")
        self.master = master
        self.state = state
        self.stop_event = stop_event
        self.on_message = on_message

    def run(self) -> None:
        while not self.stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type in {"BAD_DATA", "UNKNOWN"}:
                continue

            payload = msg.to_dict()
            payload.pop("mavpackettype", None)
            self.state.update(msg_type, payload)

            if self.on_message is not None:
                try:
                    self.on_message(msg_type, payload)
                except Exception:  # noqa: BLE001
                    pass

