# Crazyflie + ArduPilot SITL in Gazebo Garden

This directory contains a custom crazyflie model wired up for ArduPilot
SITL control, plus a world and helper scripts.

See [CHANGES.md](CHANGES.md) for the running log of edits.

## Prereqs

1. Gazebo Garden installed (you have `gz sim 7.9.0`).
2. `ardupilot_gazebo` plugin **built**:
   ```bash
   cd ~/botlab_ws/src/ardupilot_gazebo
   mkdir -p build && cd build
   GZ_VERSION=garden cmake .. && make -j$(nproc)
   ```
   The `GZ_VERSION=garden` is **required** on this box — without it
   CMake defaults to Harmonic and fails with
   `target gz-sim::gz-sim not found`.

   (Skip if `~/botlab_ws/src/ardupilot_gazebo/build/libArduPilotPlugin.so`
   already exists.)
3. ArduPilot SITL toolchain — `sim_vehicle.py` working
   (`./Tools/autotest/sim_vehicle.py --help` should print).

## Run

```bash
cd ~/botlab_ws/src/assignment
source env.sh
gz sim -v4 -r worlds/crazyflie.sdf
```

In a second terminal:
```bash
cd ~/botlab_ws/src/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-crazyflie \
    --model JSON --map --console
```

In the MAVProxy console:
```
mode GUIDED
arm throttle
takeoff 2
# ... hover ...
mode LAND
```

## Gimbal runway world — MAVLink bridge control

### What it does
Spawns a standalone `gimbal_small_1d` (1-DOF pitch gimbal with camera) on a
runway in Gazebo. Pitch is controlled over MAVLink using
`MAV_CMD_DO_MOUNT_CONFIGURE` / `MAV_CMD_DO_MOUNT_CONTROL` — no ArduPilot SITL
required. A lightweight Python bridge translates MAVLink commands to Gazebo
joint position commands in real time.

### Terminal 1 — Launch Gazebo

```bash
cd ~/botlab_ws/src
source assignment/env.sh
gz sim assignment/worlds/gimbal_runway.sdf -r
```

The Gazebo GUI opens with a 3D view and a floating **Gimbal Camera Feed**
window showing the live camera image.

### Terminal 2 — Start the MAVLink bridge

```bash
source assignment/env.sh
python3 assignment/gimbal_mavlink_bridge.py
```

Expected output:
```
[bridge] listening on udpin:0.0.0.0:14551  (sysid=1 compid=154)
[bridge] ready — waiting for commands
```

### Terminal 3 — Send gimbal commands

```bash
# Sweep from -45° to +45° and back (default demo)
python3 assignment/gimbal_control.py --sweep

# Command a single pitch angle  (negative = camera tilts down)
python3 assignment/gimbal_control.py -- -45
python3 assignment/gimbal_control.py 30

# Interactive prompt — type angles until Ctrl-D
python3 assignment/gimbal_control.py --interactive
```

The bridge prints each received command:
```
[bridge] MOUNT_CONFIGURE  mode=2
[bridge] MOUNT_CONTROL  pitch=+45.0°  (+0.7854 rad)  → /gimbal/tilt_cmd
```

### Test the joint directly (no bridge needed)

```bash
gz topic -t /gimbal/tilt_cmd -m gz.msgs.Double -p "data: 0.785"   # +45°
gz topic -t /gimbal/tilt_cmd -m gz.msgs.Double -p "data: -0.785"  # -45°
gz topic -t /gimbal/tilt_cmd -m gz.msgs.Double -p "data: 0.0"     # level
```

---

## Read sensor data back

Open a third terminal:
```bash
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:127.0.0.1:14551')
m.wait_heartbeat()
m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)
while True:
    msg = m.recv_match(blocking=True)
    if msg.get_type() in ('RAW_IMU','ATTITUDE','GLOBAL_POSITION_INT','VFR_HUD'):
        print(msg.get_type(), msg.to_dict())
"
```
