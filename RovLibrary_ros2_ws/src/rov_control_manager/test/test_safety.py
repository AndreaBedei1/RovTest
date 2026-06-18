import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rov_msgs.srv import GetCameraTilt

from rov_control_manager.control_manager_node import ControlManagerNode
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


class ControlManagerRoutingTest(unittest.TestCase):
    def test_public_camera_tilt_get_forwards_to_internal_service(self):
        initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True

        manager = ControlManagerNode()
        internal = Node("routing_test_internal_camera_tilt")
        caller = Node("routing_test_public_camera_tilt")
        executor = MultiThreadedExecutor(num_threads=4)
        spin_thread = None

        def handle_get_camera_tilt(_request, response):
            response.success = True
            response.message = "test tilt returned"
            response.mode = "mount"
            response.tilt_pwm = 1500
            response.tilt_centideg = 0
            return response

        internal_service = internal.create_service(
            GetCameraTilt,
            "/rov/internal/peripherals/camera_tilt/get",
            handle_get_camera_tilt,
        )
        public_client = caller.create_client(GetCameraTilt, "/control/camera_tilt/get")

        try:
            for node in (manager, internal, caller):
                executor.add_node(node)

            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            self.assertTrue(public_client.wait_for_service(timeout_sec=2.0))
            future = public_client.call_async(GetCameraTilt.Request())
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)

            self.assertTrue(future.done(), "public camera tilt get did not complete")
            response = future.result()
            self.assertTrue(response.success)
            self.assertEqual(response.message, "test tilt returned")
            self.assertEqual(response.mode, "mount")
            self.assertEqual(response.tilt_pwm, 1500)
            self.assertEqual(response.tilt_centideg, 0)
        finally:
            executor.shutdown()
            if spin_thread is not None:
                spin_thread.join(timeout=2.0)
            manager.destroy_node()
            internal.destroy_service(internal_service)
            internal.destroy_node()
            caller.destroy_node()
            if initialized_here and rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
