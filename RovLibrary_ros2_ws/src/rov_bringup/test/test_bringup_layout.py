from pathlib import Path
import unittest


class BringupLayoutTest(unittest.TestCase):
    def test_stage1_launch_and_config_are_installed_sources(self):
        package_dir = Path(__file__).resolve().parents[1]

        self.assertTrue((package_dir / "launch" / "rov_stage1.launch.py").exists())
        self.assertTrue((package_dir / "launch" / "rov_stage2.launch.py").exists())
        self.assertTrue((package_dir / "launch" / "rov_peripheral_smoke_test.launch.py").exists())
        self.assertTrue((package_dir / "config" / "stage1.yaml").exists())
        self.assertTrue((package_dir / "config" / "stage2.yaml").exists())

    def test_stage1_config_uses_rov_namespace(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "config" / "stage1.yaml").read_text(encoding="utf-8")

        self.assertIn("/rov/mavlink_bridge:", text)
        self.assertIn("mavlink_endpoint", text)
        self.assertIn("allowed_modes", text)

    def test_stage2_config_includes_manager_and_peripherals(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "config" / "stage2.yaml").read_text(encoding="utf-8")

        self.assertIn("/rov/mavlink_bridge:", text)
        self.assertIn("/rov/peripherals:", text)
        self.assertIn("/rov/control_manager:", text)
        self.assertIn("internal_peripheral_service_prefix", text)
        self.assertIn("require_armed_for_lights_on: false", text)


if __name__ == "__main__":
    unittest.main()
