"""Small safety helpers for command routing."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SafetyDecision:
    allowed: bool
    message: str


@dataclass
class SafetySnapshot:
    connection_seen: bool = False
    connected: bool = False
    connection_text: str = "no connection status received"
    vehicle_seen: bool = False
    armed: bool = False

    def require_connected(self, action: str) -> SafetyDecision:
        if not self.connection_seen:
            return SafetyDecision(False, f"Rejected {action}: no connection status received yet.")
        if not self.connected:
            return SafetyDecision(False, f"Rejected {action}: vehicle is not connected ({self.connection_text}).")
        return SafetyDecision(True, f"{action} allowed: vehicle connected.")

    def require_armed(self, action: str) -> SafetyDecision:
        if not self.vehicle_seen:
            return SafetyDecision(False, f"Rejected {action}: no vehicle state received yet.")
        if not self.armed:
            return SafetyDecision(False, f"Rejected {action}: vehicle is not armed.")
        return SafetyDecision(True, f"{action} allowed: vehicle armed.")

    def check(self, action: str, require_armed: bool) -> SafetyDecision:
        connected = self.require_connected(action)
        if not connected.allowed:
            return connected
        if require_armed:
            return self.require_armed(action)
        return connected


def lights_requires_armed(percent: float, policy_value: bool = True) -> bool:
    return bool(policy_value and float(percent) > 0.0)


def laser_requires_armed(enabled: bool, policy_value: bool = True) -> bool:
    return bool(policy_value and enabled)


def gripper_requires_armed(command: str, policy_value: bool = True) -> bool:
    return bool(policy_value and command.strip().lower() in {"open", "close"})


def camera_tilt_requires_armed(policy_value: bool = True) -> bool:
    return bool(policy_value)

