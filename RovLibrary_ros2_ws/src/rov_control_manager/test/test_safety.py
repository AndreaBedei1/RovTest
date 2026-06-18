from rov_control_manager.safety import (
    SafetySnapshot,
    gripper_requires_armed,
    laser_requires_armed,
    lights_requires_armed,
)


def test_rejects_when_connection_status_missing():
    decision = SafetySnapshot().check("set laser", require_armed=True)

    assert not decision.allowed
    assert "no connection status" in decision.message


def test_rejects_armed_action_when_disarmed():
    decision = SafetySnapshot(
        connection_seen=True,
        connected=True,
        vehicle_seen=True,
        armed=False,
    ).check("set laser", require_armed=True)

    assert not decision.allowed
    assert "not armed" in decision.message


def test_allows_safe_off_action_when_connected_disarmed():
    decision = SafetySnapshot(
        connection_seen=True,
        connected=True,
        vehicle_seen=True,
        armed=False,
    ).check("laser off", require_armed=laser_requires_armed(False))

    assert decision.allowed


def test_connection_only_policy_allows_peripheral_action_when_disarmed():
    decision = SafetySnapshot(
        connection_seen=True,
        connected=True,
        vehicle_seen=True,
        armed=False,
    ).check("set lights", require_armed=False)

    assert decision.allowed


def test_policy_helpers_mark_only_energizing_actions():
    assert lights_requires_armed(10.0)
    assert not lights_requires_armed(0.0)
    assert laser_requires_armed(True)
    assert not laser_requires_armed(False)
    assert gripper_requires_armed("open")
    assert not gripper_requires_armed("stop")
