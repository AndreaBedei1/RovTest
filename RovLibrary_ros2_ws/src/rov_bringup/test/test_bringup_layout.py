from pathlib import Path


def test_stage1_launch_and_config_are_installed_sources():
    package_dir = Path(__file__).resolve().parents[1]

    assert (package_dir / "launch" / "rov_stage1.launch.py").exists()
    assert (package_dir / "config" / "stage1.yaml").exists()


def test_stage1_config_uses_rov_namespace():
    package_dir = Path(__file__).resolve().parents[1]
    text = (package_dir / "config" / "stage1.yaml").read_text(encoding="utf-8")

    assert "/rov/mavlink_bridge:" in text
    assert "mavlink_endpoint" in text
    assert "allowed_modes" in text

