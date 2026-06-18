"""Reusable low-level peripheral command logic.

This module ports the actuator and camera-tilt behavior from the original
RovLibrary ``actuators.py`` and ``pilot.py`` files, but talks through a small
command-port interface instead of owning the MAVLink connection directly.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import time
from typing import Any, Protocol


PWM_MIN = 1100
PWM_MAX = 1900
MAX_LIGHTS_ON_SECONDS = 2.0
MAV_MOUNT_MODE_MAVLINK_TARGETING = 2


def clamp_pwm(pwm: int) -> int:
    return max(900, min(2100, int(pwm)))


def percent_to_pwm(percent: float) -> int:
    p = max(0.0, min(100.0, float(percent)))
    return int(round(PWM_MIN + ((PWM_MAX - PWM_MIN) * p / 100.0)))


@dataclass
class CommandResult:
    success: bool
    message: str
    commands_sent: int = 0


@dataclass
class TiltState:
    mode: str = "mount"
    tilt_pwm: int = 1500
    tilt_centideg: int = 0


@dataclass
class PeripheralState:
    lights_percent: float = 0.0
    laser_enabled: bool = False
    gripper_state: str = "stop"
    gripper_steps: int = 0
    tilt: TiltState | None = None


class MavlinkCommandPort(Protocol):
    def set_servo_group(
        self,
        servo_numbers: list[int],
        pwm: int,
        repeat: int = 1,
        interval_s: float = 0.05,
    ) -> CommandResult:
        ...

    def set_relay(self, relay_number: int, enabled: bool) -> CommandResult:
        ...

    def send_rc_override(
        self,
        overrides: dict[int, int | None],
        repeat: int = 3,
        rate_hz: float = 8.0,
    ) -> CommandResult:
        ...

    def set_mount_mode(self, mode: int) -> CommandResult:
        ...

    def set_mount_pitch(
        self,
        pitch_centideg: int,
        repeat: int = 4,
        rate_hz: float = 20.0,
    ) -> CommandResult:
        ...


DEFAULT_LASER_CFG: dict[str, Any] = {
    "control_mode": "relay",
    "relay_numbers": [],
    "servo_outputs": [],
    "on_pwm": 1900,
    "off_pwm": 1100,
    "command_rate_hz": 20.0,
    "command_repeat": 6,
    "off_repeat": 10,
    "default_profile": "out_of_water",
    "safety_profiles": {
        "out_of_water": {"max_on_seconds": 10.0},
        "in_water": {"max_on_seconds": 600.0},
    },
}


DEFAULT_CAMERA_CFG: dict[str, Any] = {
    "tilt_control_mode": "mount",
    "tilt_rc_channel": 8,
    "tilt_min_pwm": 1100,
    "tilt_max_pwm": 1900,
    "tilt_neutral_pwm": 1500,
    "tilt_step_pwm": 30,
    "tilt_min_centideg": -4500,
    "tilt_max_centideg": 4500,
    "tilt_neutral_centideg": 0,
    "tilt_step_centideg": 300,
    "invert_tilt": False,
    "send_repeat": 4,
    "send_rate_hz": 20.0,
}


def _deep_copy(data: dict[str, Any]) -> dict[str, Any]:
    return json.loads(json.dumps(data))


def _deep_merge(base: dict[str, Any], incoming: dict[str, Any]) -> dict[str, Any]:
    for key, value in incoming.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def _profile_max_seconds(section: dict[str, Any], profile_override: str | None, default: float) -> tuple[str, float]:
    default_profile = str(section.get("default_profile", "out_of_water"))
    profile_name = profile_override or default_profile
    profiles = section.get("safety_profiles", {})
    if not isinstance(profiles, dict):
        profiles = {}
    profile_cfg = profiles.get(profile_name)
    if not isinstance(profile_cfg, dict):
        profile_name = default_profile
        profile_cfg = profiles.get(profile_name, {})
    if not isinstance(profile_cfg, dict):
        profile_cfg = {}
    return profile_name, max(0.0, float(profile_cfg.get("max_on_seconds", default)))


class PeripheralBackend:
    """High-level peripheral commands backed by reusable MAVLink primitives."""

    def __init__(self, cfg: dict[str, Any], command_port: MavlinkCommandPort) -> None:
        self.cfg = cfg
        self.command_port = command_port
        self.state = PeripheralState(tilt=TiltState())

        self._lights_cfg = cfg.get("lights", {}) if isinstance(cfg.get("lights"), dict) else {}
        self._laser_cfg = _deep_copy(DEFAULT_LASER_CFG)
        if isinstance(cfg.get("laser"), dict):
            _deep_merge(self._laser_cfg, cfg["laser"])
        self._gripper_cfg = cfg.get("gripper", {}) if isinstance(cfg.get("gripper"), dict) else {}
        self._camera_cfg = _deep_copy(DEFAULT_CAMERA_CFG)
        if isinstance(cfg.get("camera"), dict):
            _deep_merge(self._camera_cfg, cfg["camera"])

        self._tilt_initialized = False
        self._tilt_channel = int(self._camera_cfg.get("tilt_rc_channel", 8))
        self._tilt_min_pwm = clamp_pwm(int(self._camera_cfg.get("tilt_min_pwm", 1100)))
        self._tilt_max_pwm = clamp_pwm(int(self._camera_cfg.get("tilt_max_pwm", 1900)))
        self._tilt_step_pwm = max(1, int(self._camera_cfg.get("tilt_step_pwm", 30)))
        self._tilt_neutral_pwm = clamp_pwm(int(self._camera_cfg.get("tilt_neutral_pwm", 1500)))
        self._tilt_min_centideg = int(self._camera_cfg.get("tilt_min_centideg", -4500))
        self._tilt_max_centideg = int(self._camera_cfg.get("tilt_max_centideg", 4500))
        self._tilt_step_centideg = max(1, int(self._camera_cfg.get("tilt_step_centideg", 300)))
        self._tilt_neutral_centideg = int(self._camera_cfg.get("tilt_neutral_centideg", 0))
        self._tilt_send_repeat = max(1, int(self._camera_cfg.get("send_repeat", 4)))
        self._tilt_send_rate_hz = max(1.0, float(self._camera_cfg.get("send_rate_hz", 20.0)))
        self._invert_tilt = bool(self._camera_cfg.get("invert_tilt", False))

        assert self.state.tilt is not None
        self.state.tilt.mode = str(self._camera_cfg.get("tilt_control_mode", "mount")).lower()
        self.state.tilt.tilt_pwm = self._tilt_neutral_pwm
        self.state.tilt.tilt_centideg = self._tilt_neutral_centideg

    def set_lights_percent(self, percent: float, profile: str | None = None) -> CommandResult:
        profile_name, max_on_seconds, servos, _off_pwm = self._resolve_lights_policy(profile)
        p = max(0.0, min(100.0, float(percent)))
        if p > 0.0 and max_on_seconds <= 0.0:
            return CommandResult(False, f"Lights profile '{profile_name}' does not allow lights on.")

        pwm = percent_to_pwm(p)
        result = self.command_port.set_servo_group(servos, pwm, repeat=2, interval_s=0.03)
        if not result.success:
            return result
        self.state.lights_percent = p
        return CommandResult(True, f"Lights set to {p:.1f}% using profile '{profile_name}'.", result.commands_sent)

    def lights_off(self) -> CommandResult:
        _profile, _max_on_seconds, servos, off_pwm = self._resolve_lights_policy(None)
        result = self.command_port.set_servo_group(servos, off_pwm, repeat=10, interval_s=0.03)
        if not result.success:
            return result
        self.state.lights_percent = 0.0
        return CommandResult(True, "Lights off.", result.commands_sent)

    def set_laser(
        self,
        enabled: bool,
        hold_seconds: float = 0.0,
        profile: str | None = None,
    ) -> CommandResult:
        ready = self._validate_laser_outputs()
        if not ready.success:
            return ready

        profile_name, max_on_seconds = _profile_max_seconds(
            self._laser_cfg,
            profile,
            default=10.0,
        )
        hold_seconds = max(0.0, float(hold_seconds))
        if enabled and hold_seconds > max_on_seconds:
            return CommandResult(
                False,
                f"Laser hold {hold_seconds:.2f}s exceeds profile '{profile_name}' limit "
                f"of {max_on_seconds:.2f}s.",
            )

        repeats = int(self._laser_cfg.get("command_repeat", 6))
        if not enabled:
            repeats = int(self._laser_cfg.get("off_repeat", 10))
        rate_hz = max(1.0, float(self._laser_cfg.get("command_rate_hz", 20.0)))

        if enabled and hold_seconds > 0.0:
            sent = self._hold_laser_state(True, hold_seconds, rate_hz, repeats)
            off = self._hold_laser_state(
                False,
                0.0,
                rate_hz,
                int(self._laser_cfg.get("off_repeat", 10)),
            )
            if not off.success:
                return off
            self.state.laser_enabled = False
            return CommandResult(
                True,
                f"Laser ON for {hold_seconds:.2f}s using profile '{profile_name}', then OFF.",
                sent.commands_sent + off.commands_sent,
            )

        result = self._hold_laser_state(enabled, 0.0, rate_hz, repeats)
        if not result.success:
            return result
        self.state.laser_enabled = bool(enabled)
        return CommandResult(
            True,
            f"Laser {'ON' if enabled else 'OFF'} using profile '{profile_name}'.",
            result.commands_sent,
        )

    def gripper_command(self, command: str, pulse_seconds: float = 0.15) -> CommandResult:
        normalized = command.strip().lower()
        if normalized == "stop":
            return self._gripper_stop()
        if normalized not in {"open", "close"}:
            return CommandResult(False, "Gripper command must be 'open', 'close', or 'stop'.")

        mode = str(self._gripper_cfg.get("control_mode", "servo")).lower().strip()
        if mode != "servo":
            return CommandResult(False, f"Unsupported gripper control_mode '{mode}'.")

        servo_output = int(self._gripper_cfg.get("servo_output", 9))
        open_pwm = clamp_pwm(int(self._gripper_cfg.get("open_pwm", 1900)))
        close_pwm = clamp_pwm(int(self._gripper_cfg.get("close_pwm", 1100)))
        neutral_pwm = clamp_pwm(int(self._gripper_cfg.get("neutral_pwm", 1500)))
        rate_hz = max(1.0, float(self._gripper_cfg.get("command_rate_hz", 20.0)))
        repeat = max(1, int(self._gripper_cfg.get("command_repeat", 6)))
        target_pwm = open_pwm if normalized == "open" else close_pwm

        first = self._send_servo_hold(
            servo_output,
            target_pwm,
            hold_seconds=max(0.02, float(pulse_seconds)),
            rate_hz=rate_hz,
            repeat=repeat,
        )
        if not first.success:
            return first
        neutral = self._send_servo_hold(
            servo_output,
            neutral_pwm,
            hold_seconds=max(0.02, float(self._gripper_cfg.get("neutral_hold_seconds", 0.4))),
            rate_hz=rate_hz,
            repeat=max(2, repeat // 2),
        )
        if not neutral.success:
            return neutral

        self.state.gripper_state = normalized
        self.state.gripper_steps += 1
        return CommandResult(
            True,
            f"Gripper {normalized} pulse sent.",
            first.commands_sent + neutral.commands_sent,
        )

    def set_camera_tilt(self, command: str, value: int = 0) -> tuple[CommandResult, TiltState]:
        normalized = command.strip().lower()
        if normalized not in {"up", "down", "center", "set"}:
            return CommandResult(False, "Camera tilt command must be 'up', 'down', 'center', or 'set'."), self.get_camera_tilt()

        init = self._ensure_tilt_initialized()
        if not init.success:
            return init, self.get_camera_tilt()

        tilt = self.get_camera_tilt()
        if tilt.mode == "mount":
            if normalized == "center":
                target = self._tilt_neutral_centideg
            elif normalized == "set":
                target = int(value)
            else:
                step = self._tilt_step_centideg
                if normalized == "down":
                    step = -step
                if self._invert_tilt:
                    step = -step
                target = tilt.tilt_centideg + step
            result = self._send_tilt_mount(target)
        else:
            if normalized == "center":
                target = self._tilt_neutral_pwm
            elif normalized == "set":
                target = int(value)
            else:
                step = self._tilt_step_pwm if normalized == "up" else -self._tilt_step_pwm
                target = tilt.tilt_pwm + step
            result = self._send_tilt_rc(target)

        if not result.success:
            return result, self.get_camera_tilt()
        return CommandResult(True, f"Camera tilt {normalized} command sent.", result.commands_sent), self.get_camera_tilt()

    def get_camera_tilt(self) -> TiltState:
        assert self.state.tilt is not None
        return TiltState(
            mode=self.state.tilt.mode,
            tilt_pwm=self.state.tilt.tilt_pwm,
            tilt_centideg=self.state.tilt.tilt_centideg,
        )

    def _resolve_lights_policy(
        self,
        profile_override: str | None,
    ) -> tuple[str, float, list[int], int]:
        profile_name, max_on_seconds = _profile_max_seconds(
            self._lights_cfg,
            profile_override,
            default=MAX_LIGHTS_ON_SECONDS,
        )
        default_servos = self._lights_cfg.get("default_servos", [13])
        if not isinstance(default_servos, list) or not default_servos:
            default_servos = [13]
        servos = [int(value) for value in default_servos]
        off_pwm = clamp_pwm(int(self._lights_cfg.get("off_pwm", 1100)))
        return profile_name, max_on_seconds, servos, off_pwm

    def _validate_laser_outputs(self) -> CommandResult:
        mode = str(self._laser_cfg.get("control_mode", "relay")).lower().strip()
        if mode == "relay":
            relays = self._laser_cfg.get("relay_numbers", [])
            if not isinstance(relays, list) or not relays:
                return CommandResult(False, "Laser config invalid: relay mode has no relay_numbers.")
            return CommandResult(True, "Laser relay outputs valid.")
        if mode == "servo":
            servos = self._laser_cfg.get("servo_outputs", [])
            if not isinstance(servos, list) or not servos:
                return CommandResult(False, "Laser config invalid: servo mode has no servo_outputs.")
            return CommandResult(True, "Laser servo outputs valid.")
        return CommandResult(False, f"Unsupported laser control_mode '{mode}'.")

    def _send_laser_once(self, enabled: bool) -> CommandResult:
        mode = str(self._laser_cfg.get("control_mode", "relay")).lower().strip()
        if mode == "relay":
            total = 0
            for relay_number in [int(value) for value in self._laser_cfg.get("relay_numbers", [])]:
                result = self.command_port.set_relay(relay_number, enabled)
                if not result.success:
                    return result
                total += 1
            return CommandResult(True, "Laser relay command sent.", total)

        servos = [int(value) for value in self._laser_cfg.get("servo_outputs", [])]
        pwm = clamp_pwm(int(self._laser_cfg.get("on_pwm" if enabled else "off_pwm")))
        return self.command_port.set_servo_group(servos, pwm, repeat=1, interval_s=0.0)

    def _hold_laser_state(
        self,
        enabled: bool,
        hold_seconds: float,
        rate_hz: float,
        fallback_repeat: int,
    ) -> CommandResult:
        hold_seconds = max(0.0, float(hold_seconds))
        rate_hz = max(1.0, float(rate_hz))
        fallback_repeat = max(1, int(fallback_repeat))
        sent = 0

        if hold_seconds <= 0.0:
            for _ in range(fallback_repeat):
                result = self._send_laser_once(enabled)
                if not result.success:
                    return result
                sent += result.commands_sent
                time.sleep(1.0 / rate_hz)
            return CommandResult(True, "Laser command cycle sent.", sent)

        end = time.monotonic() + hold_seconds
        while time.monotonic() < end:
            result = self._send_laser_once(enabled)
            if not result.success:
                return result
            sent += result.commands_sent
            time.sleep(1.0 / rate_hz)
        return CommandResult(True, "Laser hold command sent.", sent)

    def _send_servo_hold(
        self,
        servo_output: int,
        pwm: int,
        hold_seconds: float,
        rate_hz: float,
        repeat: int,
    ) -> CommandResult:
        hold_seconds = max(0.0, float(hold_seconds))
        rate_hz = max(1.0, float(rate_hz))
        repeat = max(1, int(repeat))
        sent = 0

        if hold_seconds <= 0.0:
            return self.command_port.set_servo_group([int(servo_output)], clamp_pwm(pwm), repeat=repeat, interval_s=1.0 / rate_hz)

        end = time.monotonic() + hold_seconds
        while time.monotonic() < end:
            result = self.command_port.set_servo_group([int(servo_output)], clamp_pwm(pwm), repeat=1, interval_s=0.0)
            if not result.success:
                return result
            sent += result.commands_sent
            time.sleep(1.0 / rate_hz)
        return CommandResult(True, "Servo hold sent.", sent)

    def _gripper_stop(self) -> CommandResult:
        mode = str(self._gripper_cfg.get("control_mode", "servo")).lower().strip()
        if mode != "servo":
            return CommandResult(False, f"Unsupported gripper control_mode '{mode}'.")
        result = self._send_servo_hold(
            int(self._gripper_cfg.get("servo_output", 9)),
            clamp_pwm(int(self._gripper_cfg.get("neutral_pwm", 1500))),
            hold_seconds=max(0.02, float(self._gripper_cfg.get("neutral_hold_seconds", 0.4))),
            rate_hz=max(1.0, float(self._gripper_cfg.get("command_rate_hz", 20.0))),
            repeat=max(1, int(self._gripper_cfg.get("command_repeat", 6))),
        )
        if not result.success:
            return result
        self.state.gripper_state = "stop"
        return CommandResult(True, "Gripper stop command sent.", result.commands_sent)

    def _ensure_tilt_initialized(self) -> CommandResult:
        if self._tilt_initialized:
            return CommandResult(True, "Camera tilt already initialized.")
        tilt = self.get_camera_tilt()
        if tilt.mode == "mount":
            mode_result = self.command_port.set_mount_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING)
            if not mode_result.success:
                return mode_result
            center_result = self._send_tilt_mount(self._tilt_neutral_centideg)
            if not center_result.success:
                return center_result
        self._tilt_initialized = True
        return CommandResult(True, "Camera tilt initialized.")

    def _send_tilt_rc(self, pwm: int) -> CommandResult:
        pwm = max(self._tilt_min_pwm, min(self._tilt_max_pwm, clamp_pwm(pwm)))
        result = self.command_port.send_rc_override(
            {self._tilt_channel: pwm},
            repeat=self._tilt_send_repeat,
            rate_hz=self._tilt_send_rate_hz,
        )
        if result.success:
            assert self.state.tilt is not None
            self.state.tilt.tilt_pwm = pwm
        return result

    def _send_tilt_mount(self, centideg: int) -> CommandResult:
        centideg = max(self._tilt_min_centideg, min(self._tilt_max_centideg, int(centideg)))
        result = self.command_port.set_mount_pitch(
            centideg,
            repeat=self._tilt_send_repeat,
            rate_hz=self._tilt_send_rate_hz,
        )
        if result.success:
            assert self.state.tilt is not None
            self.state.tilt.tilt_centideg = centideg
        return result

