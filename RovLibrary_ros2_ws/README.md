# RovLibrary ROS 2 Workspace

This workspace is the first ROS 2 refactor stage for the existing `RovLibrary`
codebase. It preserves the low-level MAVLink connection and telemetry knowledge,
but exposes only a small ROS 2 foundation:

- `rov_msgs`: custom ROV status messages and flight-mode service definition.
- `rov_mavlink_bridge`: Python MAVLink backend plus ROS 2 bridge node.
- `rov_bringup`: launch and configuration for the first stage.

No UI, video, gamepad control, autonomy, or peripheral actuation is implemented
in this stage.

## Package Structure

```text
RovLibrary_ros2_ws/
  requirements.txt
  src/
    rov_msgs/
      msg/
        ConnectionStatus.msg
        VehicleState.msg
      srv/
        SetFlightMode.srv
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
    rov_bringup/
      config/
        stage1.yaml
      launch/
        rov_stage1.launch.py
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

| Service | Type | Notes |
| --- | --- | --- |
| `/rov/arm` | `std_srvs/srv/Trigger` | Sends `MAV_CMD_COMPONENT_ARM_DISARM` with arm=1. |
| `/rov/disarm` | `std_srvs/srv/Trigger` | Sends `MAV_CMD_COMPONENT_ARM_DISARM` with arm=0. |
| `/rov/set_flight_mode` | `rov_msgs/srv/SetFlightMode` | Changes mode by name after allow-list validation. |

Default allowed modes are:

```text
MANUAL, STABILIZE, DEPTH_HOLD, POSHOLD, ALT_HOLD, SURFACE
```

Adjust them in `src/rov_bringup/config/stage1.yaml` for your vehicle firmware.

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

```powershell
cd C:\Users\Andrea\Desktop\RovTest\RovLibrary_ros2_ws
python -m pip install -r requirements.txt
colcon build --symlink-install
.\install\setup.ps1
```

## Run

Default MAVLink endpoint:

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

Useful manual checks:

```bash
ros2 topic echo /rov/connection_status
ros2 topic echo /rov/vehicle_state
ros2 topic echo /rov/battery
ros2 service call /rov/disarm std_srvs/srv/Trigger {}
ros2 service call /rov/set_flight_mode rov_msgs/srv/SetFlightMode "{mode: MANUAL}"
```

Use `/rov/arm` only when the vehicle is physically safe to arm.

## Assumptions

- The vehicle speaks MAVLink through `pymavlink`.
- The default endpoint remains `udpin:0.0.0.0:14550`, matching the original
  library.
- Depth is derived from `VFR_HUD.alt` when that value is negative, matching the
  existing telemetry logic. A pressure-based depth conversion can be added once
  the exact sensor source is confirmed.
- Flight-mode names must exist in the autopilot mode mapping exposed by
  `pymavlink`.

