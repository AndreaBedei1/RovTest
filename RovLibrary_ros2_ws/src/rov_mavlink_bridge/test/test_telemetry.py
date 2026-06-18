import math
import unittest

from rov_mavlink_bridge.backend.telemetry import TelemetryState


class TelemetryStateTest(unittest.TestCase):
    def test_telemetry_metrics_from_existing_mavlink_fields(self):
        state = TelemetryState()
        state.update("SYS_STATUS", {"voltage_battery": 15320, "current_battery": 420})
        state.update("BATTERY_STATUS", {"battery_remaining": 76})
        state.update("ATTITUDE", {"roll": 0.1, "pitch": -0.2, "yaw": 0.3})
        state.update("VFR_HUD", {"alt": -4.5, "heading": 181})

        metrics = state.build_metrics()

        self.assertTrue(math.isclose(metrics["voltage_v"], 15.32))
        self.assertTrue(math.isclose(metrics["current_a"], 4.2))
        self.assertEqual(metrics["battery_remaining"], 76)
        self.assertTrue(math.isclose(metrics["depth_m"], 4.5))
        self.assertTrue(math.isclose(metrics["altitude_m"], -4.5))
        self.assertTrue(math.isclose(metrics["roll_rad"], 0.1))
        self.assertEqual(metrics["heading"], 181)

    def test_battery_cell_voltage_fallback_ignores_unknown_cells(self):
        state = TelemetryState()
        state.update(
            "BATTERY_STATUS",
            {
                "voltages": [3800, 3790, 3810, 65535, 0],
                "current_battery": 123,
                "battery_remaining": -1,
            },
        )

        metrics = state.build_metrics()

        self.assertTrue(math.isclose(metrics["voltage_v"], 11.4))
        self.assertTrue(math.isclose(metrics["current_a"], 1.23))
        self.assertIsNone(metrics["battery_remaining"])


if __name__ == "__main__":
    unittest.main()
