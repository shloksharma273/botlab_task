# BotLab Assignment Report
## Crazyflie + Gimbal Simulation with ArduPilot SITL and MAVLink Control

---

## 1. Approach and Architecture

The project is split into two independent parts:

**Part A — Crazyflie drone** under full ArduPilot SITL control in Gazebo Garden.
The drone model was adapted from a PX4-style SDF, rewired for ArduPilot, and
flown via MAVLink commands through MAVProxy.

**Part B — Standalone 1-DOF gimbal** spawned in a custom runway world. Pitch
control is implemented entirely in software using a custom MAVLink bridge that
translates `MAV_CMD_DO_MOUNT_CONTROL` commands into Gazebo joint position
commands in real time — no ArduPilot SITL required for this part.

---

## 2. System Architecture

### Part A — Crazyflie SITL data flow

```
┌─────────────────────────────────────────────────────────────┐
│                        GCS / MAVProxy                        │
│           (arm, takeoff, mode change, param set)             │
└──────────────────────────┬──────────────────────────────────┘
                           │ MAVLink UDP 14550
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    ArduPilot SITL                            │
│  - Reads IMU / GPS from Gazebo via FDM socket (UDP 9002)     │
│  - Runs EKF, attitude controller, motor mixer                │
│  - Outputs PWM servo values for 4 motors (channels 0–3)      │
└──────────────────────────┬──────────────────────────────────┘
                           │ FDM JSON  UDP 9002
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                  Gazebo Garden (gz sim)                      │
│                                                              │
│   ArduPilotPlugin                                            │
│   ├─ Receives PWM → converts to rotor velocity target        │
│   ├─ Drives m1–m4 joints via ApplyJointForce                 │
│   └─ Sends back IMU + pose over FDM socket                   │
│                                                              │
│   LiftDragPlugin (×8, two blades per rotor)                  │
│   └─ Converts rotor angular velocity → thrust force          │
│                                                              │
│   Physics (DART/ODE) → drone dynamics → new pose            │
└─────────────────────────────────────────────────────────────┘
```

### Part B — Gimbal MAVLink bridge data flow

```
┌──────────────────────────────┐
│     gimbal_control.py        │
│  (sends DO_MOUNT_CONFIGURE   │
│   and DO_MOUNT_CONTROL)      │
└──────────────┬───────────────┘
               │ MAVLink UDP 14551
               ▼
┌──────────────────────────────┐
│  gimbal_mavlink_bridge.py    │
│  MAVLink component           │
│  sysid=1, compid=154         │
│  - Parses pitch_deg          │
│  - Converts → radians        │
│  - Sends COMMAND_ACK         │
└──────────────┬───────────────┘
               │ gz topic pub /gimbal/tilt_cmd
               ▼
┌──────────────────────────────┐
│  JointPositionController     │
│  (inside gimbal_small_1d     │
│   model.sdf)                 │
│  - PID drives tilt_joint     │
└──────────────┬───────────────┘
               │ joint effort
               ▼
┌──────────────────────────────┐
│  Gazebo Physics              │
│  tilt_link rotates visibly   │
│  Camera feed updates in GUI  │
└──────────────────────────────┘
```

---

## 3. Steps Performed

---

### Part A — Crazyflie Drone

#### Step 1 — Imported the drone SDF and added ArduPilot plugins

The starting point was a Crazyflie SDF wired for **PX4-style control** — it used
`MulticopterMotorModel` and `MulticopterVelocityControl` plugins, which are
incompatible with ArduPilot. The model also used legacy `ignition-gazebo-*`
plugin filenames from Gazebo Fortress, which do not exist in Gazebo Garden.

The SDF was fully rewritten to make it ArduPilot-compatible:

- **Plugin filenames** updated from `ignition-gazebo-*` to `gz-sim-*` to match
  Gazebo Garden's naming convention.
- **`MulticopterMotorModel` and `MulticopterVelocityControl` removed.** These
  are PX4 primitives. ArduPilot performs its own velocity control internally and
  only needs a way to apply force to each rotor joint.
- **`gz-sim-apply-joint-force-system` added** for each of the four rotor joints.
  This lets the ArduPilotPlugin drive rotors by applying joint torques.
- **`gz-sim-lift-drag-system` added** (two blades per rotor, eight plugins total).
  Without this, spinning the rotor joints produces no aerodynamic thrust.
  Blade area and centre-of-pressure values were scaled from the Iris reference
  model to match the Crazyflie's ~45 mm propeller diameter.
- **IMU link and sensor added.** The original SDF had no sensors. ArduPilot
  requires a named IMU sensor to read attitude and angular velocity from Gazebo.
- **`ArduPilotPlugin` added** with four motor control channels mapped to the
  ArduCopter quad-X motor order, FDM address `127.0.0.1:9002`, and lockstep
  enabled so Gazebo physics and SITL tick in sync.
- **`gazebo-crazyflie.parm`** created with frame type, motor limits, attitude
  PID gains, and position controller gains tuned for a micro-quad.

#### Step 2 — Spawned the drone in Gazebo

A minimal world file (`worlds/crazyflie.sdf`) was created containing the
required world-level plugins: Physics, Sensors, SceneBroadcaster, UserCommands,
IMU, and NavSat. Spherical coordinates were set to the ArduPilot SITL default
home location so GPS initialises correctly.

The `env.sh` script was written to add the `assignment/` directory to
`GZ_SIM_RESOURCE_PATH` (so `model://crazyflie` resolves) and the
`ardupilot_gazebo/build/` directory to `GZ_SIM_SYSTEM_PLUGIN_PATH` (so
`ArduPilotPlugin.so` is found). The `GZ_VERSION=garden` export was added so the
ardupilot_gazebo CMake build targets the correct library version.

The drone was spawned successfully and ArduPilot SITL connected via the FDM
socket, confirming the plugin was loading and sensor data was flowing.

#### Step 3 — Fine-tuned using SITL

Several rounds of tuning were needed before stable flight was achieved:

- **`MOT_THST_HOVER`** was set based on the calculated hover throttle for the
  drone's mass and blade area, then left to self-learn after a few flights.
- **Rate controller gains** (`ATC_RAT_RLL/PIT/YAW`) were set to ArduCopter
  defaults and adjusted to reduce oscillation.
- **Filter cutoffs** (`INS_GYRO_FILTER`, `ATC_RAT_*_FLTD`) were lowered to
  reduce noise-driven motor chatter.
- **`ATC_RAT_YAW_I`** was increased significantly to counteract a persistent
  yaw drift caused by minor motor torque asymmetry in the physics model.

---

### Part B — Gimbal

#### Step 1 — Created a custom Gazebo world with a runway

A new world file (`worlds/gimbal_runway.sdf`) was created following the
structure of the existing ArduPilot reference worlds. The `runway` model from
`ardupilot_gazebo/models/` was included and later copied into the `assignment/`
directory to make the submission self-contained. The gimbal model
(`gimbal_small_1d`) was spawned on the runway surface.

#### Step 2 — Spawned the gimbal in the world

The `gimbal_small_1d` model (a 1-DOF pitch gimbal with a camera on the tilt
link) was included in the world at a pose above the runway. Because the model is
dynamic (has mass and no static flag), it required a **world-fixed joint** added
to `model.sdf` to pin the base to the world frame while still allowing the
revolute `tilt_joint` to move freely.

#### Step 3 — Added the camera plugin and in-sim feed

The `tilt_link` already contained a camera sensor with a `GstCameraPlugin` for
UDP streaming. To view the feed directly inside the Gazebo window, a `<gui>`
section was added to the world file with an `ImageDisplay` plugin subscribed to
the camera's topic. The feed now appears as a floating panel in the Gazebo GUI
with no external viewer required.

#### Step 4 — Created the MAVLink interface

A two-script MAVLink interface was implemented:

**`gimbal_mavlink_bridge.py`** — acts as a minimal MAVLink gimbal component
(component ID 154). It listens for `MAV_CMD_DO_MOUNT_CONFIGURE` and
`MAV_CMD_DO_MOUNT_CONTROL` on UDP port 14551, sends heartbeats so the client
can detect it, and on receiving a mount control command converts the pitch angle
to radians and publishes it to the Gazebo joint topic via `gz topic pub`.

**`gimbal_control.py`** — the GCS-side controller. Connects to the bridge,
sends `MAV_CMD_DO_MOUNT_CONFIGURE` to set MAVLink-targeting mode, then sends
`MAV_CMD_DO_MOUNT_CONTROL` with the desired pitch. Supports single angle,
sweep demo, and interactive prompt modes.

The `JointPositionController` plugin was added to `gimbal_small_1d/model.sdf`
subscribed to `/gimbal/tilt_cmd`, completing the chain from MAVLink command to
physical joint movement.

---

## 4. Issues Faced and How They Were Resolved

---

### Issue 1 — SDF arithmetic expressions silently ignored

**Cause:** The original SDF had propeller inertias written as `<ixx>2e-9*70</ixx>`.
SDF does not evaluate arithmetic — the parser reads only the first number, so
all four propellers ended up with different effective inertias, causing the drone
to tip and crash on every takeoff.

**Fix:** Replaced all arithmetic expressions with pre-computed scalar values.
All four propellers set to identical inertia: `ixx=2e-9, iyy=1.67e-7, izz=1.68e-7`.

---

### Issue 2 — Drone jitters and yaws immediately on SITL connection

**Cause (A):** `servo_min=1100` with disarmed PWM=1000. The plugin normalised
this to a negative velocity target, commanding all rotors to spin backwards at
near-full speed before arming.

**Cause (B):** `no_time_sync=1` with `lock_step=1` caused the plugin to burst
through many physics steps during IMU calibration, producing a large impulse jolt.

**Fix:** Set `servo_min=1000` so disarmed PWM maps to zero velocity. Removed
`no_time_sync` to restore wall-clock synchronisation.

---

### Issue 3 — Persistent gyro inconsistency causing drone unable to disarm

**Cause:** Even after fixing the servo mapping and timing issues, the drone
continued to exhibit residual jitter during flight — small but continuous rotor
micro-corrections that fed noise back into the gyroscope readings. This made the
IMU report inconsistent angular rates, which ArduPilot's EKF flagged as an
unhealthy sensor state. As a result the drone refused to disarm safely after
landing, remaining in an armed state with rotors spinning.

Multiple rounds of PID tuning were attempted — reducing `ATC_RAT_RLL/PIT_D`,
lowering `INS_GYRO_FILTER`, and increasing `ATC_INPUT_TC` — but none fully
resolved the issue. The root cause is a fundamental limitation of the simulated
physics model: the Gazebo lift-drag forces contain small numerical asymmetries
between rotors that a software controller cannot completely cancel. This is an
accepted known limitation of the simulation.

---

### Issue 4 — Gimbal too large and heavy to attach to the drone

**Cause:** The `gimbal_small_1d` model was designed as a full-scale standalone
ground gimbal. Its mass (20 g base + tilt) and physical dimensions were too
large relative to the Crazyflie airframe. When attached below the drone body,
the gimbal's collision geometry extended beyond the drone's footprint, intersected
with the ground plane on spawn, and shifted the centre of mass far enough below
the rotors that the flight controller could not maintain stable attitude. The
added payload also exceeded the thrust margin available from the scaled lift-drag
model, preventing takeoff.

**Resolution:** The gimbal was decoupled from the drone entirely. Instead of
being mounted on the Crazyflie, it was spawned as a standalone model in a
dedicated runway world (`gimbal_runway.sdf`) and controlled independently via
the MAVLink bridge. The drone model was cleaned up by removing all gimbal links,
joints, and the ArduPilot servo channel that had been wired to the gimbal joint.

---

### Issue 5 — Gazebo crashes (ODE assertion) when gimbal world is launched

**Cause:** The `gimbal_small_1d` model is dynamic. With nothing holding it up it
fell under gravity, reached extreme coordinates, and triggered an ODE AABB
bounds assertion.

**Fix:** Added a `type="fixed"` joint from `world` to `base_link` in
`model.sdf`. This pins the base in place while leaving `tilt_joint` free to
rotate.

---

### Issue 6 — `JointPositionController` plugin not loading

**Cause:** The plugin was placed inside the `<include>` block in the world SDF.
In Gazebo Garden this merge path does not reliably load plugins attached to
included models.

**Fix:** Moved the plugin directly into `gimbal_small_1d/model.sdf`. The plugin
loaded immediately and the `/gimbal/tilt_cmd` topic became active.

---

### Issue 7 — Gimbal oscillates rapidly after receiving commands

**Cause:** PID `p_gain=5` was used initially. With `tilt_link` inertia of
`0.00001 kg·m²`, this produces a natural frequency of ~700 rad/s — the joint
oscillates violently instead of settling.

**Fix:** Reduced `p_gain` to `0.1`, giving a natural frequency of ~100 rad/s,
which combined with the joint's physical damping produces a stable response.
