from rov_peripherals.backend import CommandResult, PeripheralBackend


class FakePort:
    def __init__(self):
        self.calls = []

    def set_servo_group(self, servo_numbers, pwm, repeat=1, interval_s=0.05):
        self.calls.append(("servo", list(servo_numbers), int(pwm), int(repeat)))
        return CommandResult(True, "ok", len(servo_numbers) * int(repeat))

    def set_relay(self, relay_number, enabled):
        self.calls.append(("relay", int(relay_number), bool(enabled)))
        return CommandResult(True, "ok", 1)

    def send_rc_override(self, overrides, repeat=3, rate_hz=8.0):
        self.calls.append(("rc", dict(overrides), int(repeat)))
        return CommandResult(True, "ok", int(repeat))

    def set_mount_mode(self, mode):
        self.calls.append(("mount_mode", int(mode)))
        return CommandResult(True, "ok", 1)

    def set_mount_pitch(self, pitch_centideg, repeat=4, rate_hz=20.0):
        self.calls.append(("mount_pitch", int(pitch_centideg), int(repeat)))
        return CommandResult(True, "ok", int(repeat))


def test_lights_percent_maps_to_configured_servo():
    port = FakePort()
    backend = PeripheralBackend({"lights": {"default_servos": [13]}}, port)

    result = backend.set_lights_percent(50.0)

    assert result.success
    assert backend.state.lights_percent == 50.0
    assert port.calls[-1] == ("servo", [13], 1500, 2)


def test_laser_requires_configured_outputs():
    port = FakePort()
    backend = PeripheralBackend({"laser": {"control_mode": "relay", "relay_numbers": []}}, port)

    result = backend.set_laser(True)

    assert not result.success
    assert "relay_numbers" in result.message


def test_gripper_open_pulses_then_neutral():
    port = FakePort()
    backend = PeripheralBackend({}, port)

    result = backend.gripper_command("open", pulse_seconds=0.02)

    assert result.success
    assert backend.state.gripper_state == "open"
    assert port.calls[0][0] == "servo"
    assert port.calls[-1][0] == "servo"
    assert port.calls[-1][2] == 1500


def test_camera_tilt_mount_initializes_and_steps():
    port = FakePort()
    backend = PeripheralBackend({}, port)

    result, tilt = backend.set_camera_tilt("up")

    assert result.success
    assert tilt.mode == "mount"
    assert tilt.tilt_centideg == 300
    assert ("mount_mode", 2) in port.calls


def test_smoke_test_node_source_exists():
    from pathlib import Path

    package_dir = Path(__file__).resolve().parents[1]

    assert (package_dir / "rov_peripherals" / "hardware_smoke_test.py").exists()
    assert (package_dir / "launch" / "hardware_smoke_test.launch.py").exists()
    assert (package_dir / "config" / "hardware_smoke_test.yaml").exists()
