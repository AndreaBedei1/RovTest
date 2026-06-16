"""Reusable MAVLink backend code for the ROS 2 bridge."""

from .telemetry import TelemetryReceiver, TelemetryState

__all__ = ["MavlinkBackend", "TelemetryReceiver", "TelemetryState"]


def __getattr__(name: str):
    if name == "MavlinkBackend":
        from .mavlink_backend import MavlinkBackend

        return MavlinkBackend
    raise AttributeError(name)
