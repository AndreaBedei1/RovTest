from pathlib import Path


def test_required_interfaces_are_present():
    package_dir = Path(__file__).resolve().parents[1]

    assert (package_dir / "msg" / "ConnectionStatus.msg").exists()
    assert (package_dir / "msg" / "VehicleState.msg").exists()
    assert (package_dir / "srv" / "GetCameraTilt.srv").exists()
    assert (package_dir / "srv" / "GripperCommand.srv").exists()
    assert (package_dir / "srv" / "SetCameraTilt.srv").exists()
    assert (package_dir / "srv" / "SetFlightMode.srv").exists()
    assert (package_dir / "srv" / "SetLaser.srv").exists()
    assert (package_dir / "srv" / "SetLights.srv").exists()
    assert (package_dir / "srv" / "SetServo.srv").exists()


def test_set_flight_mode_has_mode_request_and_success_response():
    package_dir = Path(__file__).resolve().parents[1]
    text = (package_dir / "srv" / "SetFlightMode.srv").read_text(encoding="utf-8")

    assert "string mode" in text
    assert "---" in text
    assert "bool success" in text
    assert "string message" in text


def test_peripheral_services_return_clear_messages():
    package_dir = Path(__file__).resolve().parents[1]
    for service_name in [
        "SetLights",
        "SetLaser",
        "GripperCommand",
        "SetCameraTilt",
        "GetCameraTilt",
    ]:
        text = (package_dir / "srv" / f"{service_name}.srv").read_text(encoding="utf-8")
        assert "---" in text
        assert "bool success" in text
        assert "string message" in text
