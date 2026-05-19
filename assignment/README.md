# BotLab Assignment — Crazyflie + Gimbal in Gazebo Garden

This project sets up a **Crazyflie quadrotor** under ArduPilot SITL control and
a **standalone 1-DOF gimbal** with MAVLink pitch control, both simulated in
Gazebo Garden.

---

## Prerequisites

- **Gazebo Garden** (`gz sim --version` should report `7.x`)
- **ArduPilot SITL** toolchain — `sim_vehicle.py` must be on your PATH
- **pymavlink** — `pip3 install pymavlink`
- **ardupilot_gazebo plugin** built against Garden:

  ```bash
  cd ~/botlab_ws/src/ardupilot_gazebo
  mkdir -p build && cd build
  GZ_VERSION=garden cmake .. && make -j$(nproc)
  ```

---

## Clone and setup

```bash
git clone git@github.com:shloksharma273/botlab_task.git
cd botlab_task/assignment

# Set Gazebo resource paths and plugin paths
source env.sh
```

> Run `source env.sh` in **every new terminal** before launching anything.

---

## Run the Crazyflie drone

Open **two terminals**.

**Terminal 1 — Gazebo:**
```bash
source assignment/env.sh
gz sim assignment/worlds/crazyflie.sdf -r
```

**Terminal 2 — ArduPilot SITL:**
```bash
cd ~/botlab_ws/src/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-crazyflie \
    --model JSON --map --console
```

Once MAVProxy shows `APM: EKF3 IMU0 is using GPS`, the drone is ready.

**Basic flight commands in MAVProxy:**
```
mode GUIDED
arm throttle
takeoff 2
mode LAND
```

> First launch ever? Add `-w` to the SITL command to wipe EEPROM and load
> parameters cleanly. Drop `-w` on all subsequent launches.

---

## Run the gimbal world

The gimbal world spawns a standalone `gimbal_small_1d` on a runway.
The **camera feed appears inside the Gazebo window** — no extra viewer needed.

Open **three terminals**.

**Terminal 1 — Gazebo:**
```bash
source assignment/env.sh
gz sim assignment/worlds/gimbal_runway.sdf -r
```

The Gazebo GUI opens with a 3D view and a floating **Gimbal Camera Feed**
panel in the top-left corner showing the live camera image.

**Terminal 2 — MAVLink bridge:**
```bash
source assignment/env.sh
python3 assignment/gimbal_mavlink_bridge.py
```

Wait for:
```
[bridge] listening on udpin:0.0.0.0:14551  (sysid=1 compid=154)
[bridge] ready — waiting for commands
```

**Terminal 3 — Control the gimbal:**
```bash
# Interactive mode — type a pitch angle and press Enter to move the gimbal
python3 assignment/gimbal_control.py --interactive
```

```
Enter pitch in degrees [-90, 90]  (Ctrl-D to quit)
pitch> -45       # tilt camera down
pitch> 0         # level
pitch> 45        # tilt camera up
```

The bridge confirms each command:
```
[bridge] MOUNT_CONTROL  pitch=-45.0°  (-0.7854 rad)  → /gimbal/tilt_cmd
```

And the gimbal arm visibly rotates in the 3D view.

**Other control modes:**
```bash
# Automated sweep from -45° to +45° and back
python3 assignment/gimbal_control.py --sweep

# Single angle command
python3 assignment/gimbal_control.py -- -45
```

---

## Project structure

```
assignment/
├── env.sh                    # Sets GZ_SIM_RESOURCE_PATH and plugin paths
├── crazyflie/                # Crazyflie quad model (ArduPilot-compatible)
├── gimbal_small_1d/          # 1-DOF pitch gimbal with camera
├── runway/                   # Runway model (self-contained)
├── worlds/
│   ├── crazyflie.sdf         # Crazyflie flight world
│   └── gimbal_runway.sdf     # Standalone gimbal world with camera display
├── gimbal_control.py         # MAVLink gimbal controller (GCS side)
├── gimbal_mavlink_bridge.py  # MAVLink ↔ Gazebo joint bridge
├── crazyflie_gimbal.parm     # ArduPilot mount parameters (for drone-mounted gimbal)
├── tuning_params.md          # PID and physics tuning reference
├── CHANGES.md                # Full log of every change made and why
└── commands.md               # Quick command reference
```

---

## See also

- `tuning_params.md` — explanation of every tunable parameter and how it affects flight
- `CHANGES.md` — detailed log of all design decisions
