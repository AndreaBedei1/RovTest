import unittest

from rov_control_manager.safety import (
    SafetySnapshot,
    gripper_requires_armed,
    laser_requires_armed,
    lights_requires_armed,
)


class SafetyPolicyTest(unittest.TestCase):
    def test_rejects_when_connection_status_missing(self):
        decision = SafetySnapshot().check("set laser", require_armed=True)

        self.assertFalse(decision.allowed)
        self.assertIn("no connection status", decision.message)

    def test_rejects_armed_action_when_disarmed(self):
        decision = SafetySnapshot(
            connection_seen=True,
            connected=True,
            vehicle_seen=True,
            armed=False,
        ).check("set laser", require_armed=True)

        self.assertFalse(decision.allowed)
        self.assertIn("not armed", decision.message)

    def test_allows_safe_off_action_when_connected_disarmed(self):
        decision = SafetySnapshot(
            connection_seen=True,
            connected=True,
            vehicle_seen=True,
            armed=False,
        ).check("laser off", require_armed=laser_requires_armed(False))

        self.assertTrue(decision.allowed)

    def test_connection_only_policy_allows_peripheral_action_when_disarmed(self):
        decision = SafetySnapshot(
            connection_seen=True,
            connected=True,
            vehicle_seen=True,
            armed=False,
        ).check("set lights", require_armed=False)

        self.assertTrue(decision.allowed)

    def test_policy_helpers_mark_only_energizing_actions(self):
        self.assertTrue(lights_requires_armed(10.0))
        self.assertFalse(lights_requires_armed(0.0))
        self.assertTrue(laser_requires_armed(True))
        self.assertFalse(laser_requires_armed(False))
        self.assertTrue(gripper_requires_armed("open"))
        self.assertFalse(gripper_requires_armed("stop"))


if __name__ == "__main__":
    unittest.main()
