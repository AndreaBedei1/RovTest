# RovLibrary ROS 2 Workspace

This workspace is the first ROS 2 refactor stage for the existing `RovLibrary`
codebase. It preserves the low-level MAVLink connection and telemetry knowledge,
but exposes only a small ROS 2 foundation:

- `rov_msgs`: custom ROV status messages and flight-mode service definition.
- `rov_mavlink_bridge`: Python MAVLink backend plus ROS 2 bridge node.
- `rov_peripherals`: internal services for lights, laser, gripper, and camera tilt.
- `rov_control_manager`: public safe command router for vehicle and peripheral commands.
- `rov_bringup`: launch and configuration for Stage 1 and Stage 2.

No UI, video, gamepad control, or autonomy is implemented in these stages.
Stage 2 adds only bounded peripheral command services and safe routing.

## Package Structure

```text
RovLibrary_ros2_ws/
  requirements.txt
  scripts/
    preflight_ros2_windows.py
    setup_windows_ros2_conda.bat
  src/
    rov_msgs/
      msg/
        ConnectionStatus.msg
        VehicleState.msg
      srv/
        GetCameraTilt.srv
        GripperCommand.srv
        SetCameraTilt.srv
        SetFlightMode.srv
        SetLaser.srv
        SetLights.srv
        SetMountControl.srv
        SetMountMode.srv
        SetRcOverride.srv
        SetRelay.srv
        SetServo.srv
    rov_mavlink_bridge/
      rov_mavlink_bridge/
        backend/
          mavlink_backend.py
          telemetry.py
        mavlink_bridge_node.py
      config/
        mavlink_bridge.yaml
      launch/
        mavlink_bridge.launch.py
      test/
        test_telemetry.py
    rov_peripherals/
      rov_peripherals/
        backend/
          peripheral_backend.py
        hardware_smoke_test.py
        peripherals_node.py
      config/
        hardware_smoke_test.yaml
        peripherals.yaml
      launch/
        hardware_smoke_test.launch.py
        peripherals.launch.py
    rov_control_manager/
      rov_control_manager/
        control_manager_node.py
        safety.py
      config/
        control_manager.yaml
      launch/
        control_manager.launch.py
    rov_bringup/
      config/
        stage1.yaml
        stage2.yaml
      launch/
        rov_peripheral_smoke_test.launch.py
        rov_stage1.launch.py
        rov_stage2.launch.py
      test/
        test_bringup_layout.py
```

## Topics

When launched through `rov_bringup`, the node runs in the `/rov` namespace and
creates these topics:

| Topic | Type | Notes |
| --- | --- | --- |
| `/rov/connection_status` | `rov_msgs/msg/ConnectionStatus` | Link state, heartbeat age, message count. |
| `/rov/vehicle_state` | `rov_msgs/msg/VehicleState` | Armed state, MAVLink mode, target system/component. |
| `/rov/battery` | `sensor_msgs/msg/BatteryState` | Voltage, current, percentage when available. |
| `/rov/attitude` | `sensor_msgs/msg/Imu` | Orientation from MAVLink `ATTITUDE`; angular rates when available. |
| `/rov/depth` | `std_msgs/msg/Float32` | Derived from negative `VFR_HUD.alt` when available. |
| `/rov/altitude` | `std_msgs/msg/Float32` | Raw `VFR_HUD.alt` when available. |

## Services

Stage 1 bridge services are still available if launched directly, but Stage 2
remaps them under `/rov/internal/*`. Use the public `/rov/control/*` services
when running Stage 2.

| Service | Type | Notes |
| --- | --- | --- |
| `/rov/control/arm` | `std_srvs/srv/Trigger` | Manager-checked arm command. |
| `/rov/control/disarm` | `std_srvs/srv/Trigger` | Manager-checked disarm command. |
| `/rov/control/set_flight_mode` | `rov_msgs/srv/SetFlightMode` | Manager-checked mode change. |
| `/rov/control/lights/set` | `rov_msgs/srv/SetLights` | Set lights percent. |
| `/rov/control/laser/set` | `rov_msgs/srv/SetLaser` | Laser on/off with optional bounded hold. |
| `/rov/control/gripper/open` | `std_srvs/srv/Trigger` | One gripper open pulse. |
| `/rov/control/gripper/close` | `std_srvs/srv/Trigger` | One gripper close pulse. |
| `/rov/control/gripper/stop` | `std_srvs/srv/Trigger` | Neutral gripper stop. |
| `/rov/control/camera_tilt/set` | `rov_msgs/srv/SetCameraTilt` | `up`, `down`, `center`, or `set`. |
| `/rov/control/camera_tilt/get` | `rov_msgs/srv/GetCameraTilt` | Current camera tilt state tracked by peripherals. |

Internal Stage 2 services exist under `/rov/internal/*` for routing between
nodes. They are not the operator/HMI API.

No thruster or vehicle-motion command services are created in this step.

Default allowed modes are:

```text
MANUAL, STABILIZE, DEPTH_HOLD, POSHOLD, ALT_HOLD, SURFACE
```

Adjust them in `src/rov_bringup/config/stage1.yaml` or
`src/rov_bringup/config/stage2.yaml` for your vehicle firmware.

## Windows Setup

Use a normal `cmd.exe` or Anaconda Prompt where `conda` is on `PATH`.
The setup script creates or reuses a conda environment named `ros2` with
Python 3.8.3, installs the pinned Python requirements, checks for ROS 2 Humble,
and runs preflight.

```bat
cd /d C:\Users\Andrea\Desktop\RovTest\RovLibrary_ros2_ws
scripts\setup_windows_ros2_conda.bat
```

If ROS 2 Humble is installed somewhere non-standard, set `ROS2_SETUP_BAT` first:

```bat
cd /d C:\Users\Andrea\Desktop\RovTest\RovLibrary_ros2_ws
set ROS2_SETUP_BAT=C:\dev\ros2_humble\local_setup.bat
scripts\setup_windows_ros2_conda.bat
```

Manual preflight after opening a fresh shell:

```bat
cd /d C:\Users\Andrea\Desktop\RovTest\RovLibrary_ros2_ws
conda activate ros2
call C:\dev\ros2_humble\local_setup.bat
python scripts\preflight_ros2_windows.py
```

The Python dependency set is pinned for Windows/Python 3.8.3:

```text
fastcrc==0.3.2
pymavlink==2.4.49
pytest>=7.0,<9.0
```

The `fastcrc` pin avoids the failing `fastcrc 0.3.6` source-build path on
Python 3.8 Windows.

## Build

From a ROS 2 environment:

```bash
cd /path/to/RovLibrary_ros2_ws
source /opt/ros/humble/setup.bash
python3 -m pip install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

On a native Windows ROS 2 shell, use the same workspace path and source the
generated setup script with:

```bat
cd /d C:\Users\Andrea\Desktop\RovTest\RovLibrary_ros2_ws
conda activate ros2
call C:\dev\ros2_humble\local_setup.bat
python scripts\preflight_ros2_windows.py
colcon build --symlink-install
call install\setup.bat
```

## Run

Stage 2, with control manager and peripherals:

```bash
ros2 launch rov_bringup rov_stage2.launch.py
```

Override the endpoint:

```bash
ros2 launch rov_bringup rov_stage2.launch.py mavlink_endpoint:=udpin:0.0.0.0:14550
```

Peripheral-only real-rover smoke test, starting the Stage 2 stack first:

```bash
ros2 launch rov_bringup rov_peripheral_smoke_test.launch.py
```

The smoke test defaults are intentionally conservative:

```bash
ros2 launch rov_bringup rov_peripheral_smoke_test.launch.py \
  lights_percent:=5.0 \
  enable_laser_test:=false \
  enable_gripper_motion:=false
```

Opt in to the more hazardous peripheral checks only when the work area is safe:

```bash
ros2 launch rov_bringup rov_peripheral_smoke_test.launch.py \
  enable_laser_test:=true \
  enable_gripper_motion:=true
```

If Stage 2 is already running, launch only the smoke-test node:

```bash
ros2 launch rov_peripherals hardware_smoke_test.launch.py
```

Stage 1 telemetry-only foundation:

```bash
ros2 launch rov_bringup rov_stage1.launch.py
```

Override the endpoint:

```bash
ros2 launch rov_bringup rov_stage1.launch.py mavlink_endpoint:=udpin:0.0.0.0:14550
```

Run the bridge package launch directly:

```bash
ros2 launch rov_mavlink_bridge mavlink_bridge.launch.py
```

## Test

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Safe local checks:

```bat
python scripts\preflight_ros2_windows.py
colcon list --base-paths src
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Safe read-only runtime checks:

```bash
ros2 topic echo /rov/connection_status
ros2 topic echo /rov/vehicle_state
ros2 topic echo /rov/battery
ros2 service call /rov/control/camera_tilt/get rov_msgs/srv/GetCameraTilt {}
```

Do not call arm, disarm, flight-mode, thruster, or actuator-setting services
during setup validation. The peripheral smoke-test launch is for a later
real-rover validation step after the workspace builds and the test area is safe.

Stage 2 safety defaults:

- Any actuator command is rejected while disconnected.
- Actuator requests are exposed publicly only through `/rov/control/...`.
- Armed-state checks are configurable, but disabled by default for peripheral
  smoke testing because this step does not add thruster or vehicle-motion
  control.

## Assumptions

- The vehicle speaks MAVLink through `pymavlink`.
- The default endpoint remains `udpin:0.0.0.0:14550`, matching the original
  library.
- Depth is derived from `VFR_HUD.alt` when that value is negative, matching the
  existing telemetry logic. A pressure-based depth conversion can be added once
  the exact sensor source is confirmed.
- Flight-mode names must exist in the autopilot mode mapping exposed by
  `pymavlink`.
- Stage 2 assumes the MAVLink bridge owns the only MAVLink connection. Peripheral
  nodes route through internal bridge services instead of opening another UDP
  reader.
