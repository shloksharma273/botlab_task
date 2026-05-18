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
