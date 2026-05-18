# Crazyflie + ArduPilot SITL — Change Log

This document tracks every change made to bring the custom crazyflie SDF under
ArduPilot SITL control in Gazebo Garden. Each entry includes the *what* and
*why* so future-you (or me) can revisit decisions.

---

## 2026-05-18 — Initial conversion from PX4-style → ArduPilot-style

### Context
The provided `drone_model/` contained a crazyflie SDF wired up for
**PX4-style control**:
- `MulticopterMotorModel` (one per rotor) — a velocity-controlled motor
  primitive used by the rotors_simulator stack.
- `MulticopterVelocityControl` — a PX4-side velocity controller that subscribes
  to `cmd_vel` and emits per-motor speed commands.
- No IMU, no GPS, no air-pressure sensors.
- No `ArduPilotPlugin` (the bridge that exposes a UDP FDM interface to SITL).

Also: the plugins used the legacy `ignition-gazebo-*` filenames (Gazebo
Fortress / `ign-gazebo` 6), but this workstation runs **Gazebo Garden**
(`gz sim 7.9.0`) which uses the `gz-sim-*` names.

### Steps performed

#### 1. Renamed `drone_model/` → `crazyflie/`
- Reason: the SDF references mesh paths as `model://crazyflie/meshes/...`.
  Gazebo's `model://` resolver maps to a folder *named* `crazyflie` on
  `GZ_SIM_RESOURCE_PATH`. The folder name must match the URI.
- The parent directory `~/botlab_ws/src/assignment/` now needs to be on
  `GZ_SIM_RESOURCE_PATH` (see env setup below).

#### 2. Created `CHANGES.md`
- This file. Tracks the *what* and *why* of every edit so we can revisit
  decisions later.

#### 3. Rewrote `crazyflie/model.sdf`
Full rewrite — too many lines changed for an in-place patch. Specifically:

- **SDF version bumped 1.8 → 1.9** to match the iris reference and the
  `gz-sim-*` plugin schema.
- **Removed model yaw** `<pose>0 0 0 0 0 3.14</pose>` → set to
  `<pose>0 0 0.05 0 0 0</pose>`. The 3.14 rad (180°) yaw silently flipped
  every motor's "front/back" interpretation, making the ArduPilot motor
  mapping nearly impossible to reason about. Z=0.05 so the model spawns
  just above the ground at startup.
- **Fixed propeller inertia bugs**: original had arithmetic expressions
  like `<ixx>0.000000002*70</ixx>` ([drone_model/model.sdf:42-44] before
  the rename). SDF does **not** evaluate arithmetic — that string was
  either rejected or silently zeroed depending on parser version. Replaced
  with pre-computed values:
  - m1 (×70 factor): ixx=1.4e-7, iyy=1.169e-5, izz=1.176e-5
  - m2/m3/m4 (×50 factor): ixx=1.0e-7, iyy=8.35e-6, izz=8.4e-6
- **Added `imu_link` + IMU sensor**, joined fixed to `crazyflie/body`.
  ArduPilotPlugin requires an `<imu>` sensor it can name via `<imuName>`.
  The original SDF had no sensors of any kind.
- **Deleted 4× `MulticopterMotorModel` plugins** (PX4-style motor model).
  Replaced with **4× `gz-sim-apply-joint-force-system`** (one per motor
  joint) — this is the primitive the ArduPilotPlugin uses to drive joints
  via its internal PID.
- **Deleted `MulticopterVelocityControl`** (PX4 velocity controller).
  ArduPilot does its own velocity control inside SITL — Gazebo just owns
  the FDM (physics).
- **Deleted `OdometryPublisher`** — not needed; ArduPilot publishes
  position via MAVLink, and we can read ground truth via `gz topic`.
- **Added 8× `gz-sim-lift-drag-system`** (two blades per rotor, mirroring
  iris). Without lift-drag, spinning the rotor joints produces no thrust.
  Scaled `area=8e-5` (≈ iris ÷ 25) and `cp=±0.012` (≈ iris ÷ 7) for the
  ~45 mm crazyflie prop. These will need tuning at first flight.
- **Added `JointStatePublisher` plugin** — convenience, so `gz topic` can
  echo joint states for debugging.
- **Added `ArduPilotPlugin` block** with:
  - `fdm_addr=127.0.0.1`, `fdm_port_in=9002` (matches SITL's defaults).
  - `lock_step=1` so Gazebo physics ticks in lockstep with ArduPilot.
  - `<imuName>crazyflie/imu_link::imu_sensor</imuName>`.
  - Four `<control>` channels with the **ArduCopter quad-X motor order**:
    | Channel | ArduCopter motor | Position    | Spin | SDF joint |
    |---------|------------------|-------------|------|-----------|
    | 0       | MOT_1            | front-right | CCW  | m1_joint  |
    | 1       | MOT_2            | back-left   | CCW  | m3_joint  |
    | 2       | MOT_3            | front-left  | CW   | m4_joint  |
    | 3       | MOT_4            | back-right  | CW   | m2_joint  |
  - `multiplier=±2618` (≈ 25 000 RPM peak, from the crazyflie's original
    `maxRotVelocity`). Sign encodes spin direction (+ = CCW, like iris).
  - `p_gain=0.005`, `cmd_max=0.05`: scaled way down from iris (0.20, 2.5)
    because the crazyflie's rotor inertia is ~1000× smaller — same gain
    would saturate / oscillate. Expect to tune.
- **Plugin filenames** updated from legacy `ignition-gazebo-*`
  (Fortress) to `gz-sim-*` (Garden). Class names updated from
  `ignition::gazebo::systems::*` to `gz::sim::systems::*`.

#### 4. Created `worlds/crazyflie.sdf`
- Minimal world: physics, sensors, IMU/NavSat systems, scene-broadcaster,
  user-commands. All world-level system plugins ArduPilotPlugin and the
  IMU sensor need to function.
- `<spherical_coordinates>` set to the ArduPilot default home
  (-35.363262, 149.165237, 584 m).
- Ground plane added (the original SDF assumed one existed).
- Spawns `model://crazyflie` at `(0, 0, 0.05)`.

#### 5. Created `env.sh`
- Adds `assignment/` and `assignment/worlds/` to `GZ_SIM_RESOURCE_PATH`
  so `gz sim` resolves `model://crazyflie` and the world file.
- Adds `ardupilot_gazebo/build/` to `GZ_SIM_SYSTEM_PLUGIN_PATH` so the
  `ArduPilotPlugin.so` is found.

#### 6. Created `README.md`
- Quickstart: source `env.sh`, run `gz sim worlds/crazyflie.sdf`, run
  `sim_vehicle.py -f gazebo-crazyflie`, MAVProxy commands for
  takeoff/land, and a `pymavlink` snippet for reading IMU/attitude/GPS
  back as MAVLink.

### Files NOT modified
- `crazyflie/model.config` — author info etc., still valid.
- `crazyflie/meshes/*.dae` — unchanged.
- `ardupilot/Tools/autotest/default_params/gazebo-crazyflie.parm` — already
  existed with sensible defaults; `gazebo-crazyflie` frame is already
  registered in `vehicleinfo.json`.

### Outstanding / likely tuning needed on first flight
- **Lift-drag `area` + `cp`** — first guess; may need to be doubled/halved
  to get a reasonable hover.
- **ArduPilotPlugin `p_gain` + `multiplier`** — rotor-velocity PID gains
  are vehicle-scale-dependent and not derivable a priori.
- **Crazyflie prop link inertias** — the iyy/izz values are guesses
  extrapolated from the original `×70` / `×50` expressions; if rotors
  show weird spin-up dynamics, revisit.
- **`MOT_THST_HOVER`** in the parm file (currently 0.55) — ArduCopter
  will re-learn this in flight but a bad initial guess can prevent takeoff.

### Prerequisite the user needs to handle
- Build the `ardupilot_gazebo` plugin (`build/` is currently empty).
  See README.md → Prereqs §2.

---

## 2026-05-18 — Fixed `ardupilot_gazebo` build for Garden

### Problem
Running `cmake ..` in `ardupilot_gazebo/build/` failed with:
```
Target "ArduPilotPlugin" links to target "gz-sim::gz-sim" but the target
was not found.
Target "CameraZoomPlugin" links to target "gz-rendering::gz-rendering" but
the target was not found.
```

### Root cause
`ardupilot_gazebo/CMakeLists.txt` selects the Gazebo version from the
`GZ_VERSION` environment variable. The default branch (when `GZ_VERSION`
is unset) is **Harmonic**, which looks for `gz-sim8` and `gz-rendering8`.
This box has **Garden** installed (`libgz-sim7`, `libgz-rendering7`), so
the harmonic find_package() calls fail and the imported targets never
get created.

The CMake file *does* have a working `garden` branch
([CMakeLists.txt:65-79](../ardupilot_gazebo/CMakeLists.txt#L65-L79)) — it
just isn't selected unless told.

### Fix
Build with `GZ_VERSION=garden`:
```bash
cd ~/botlab_ws/src/ardupilot_gazebo
rm -rf build && mkdir build && cd build
GZ_VERSION=garden cmake .. && make -j$(nproc)
```
This produces `libArduPilotPlugin.so`, `libParachutePlugin.so`,
`libCameraZoomPlugin.so`, `libGstCameraPlugin.so`.

### Docs/script updates
- `env.sh` — added `export GZ_VERSION=garden`, so sourcing it covers
  future rebuilds too.
- `README.md` Prereqs §2 — `cmake` invocation now shows `GZ_VERSION=garden`
  inline.

---

## 2026-05-18 — Pre-arm failure: "Motors: Check frame class and type"

### Symptom
After launching SITL with `-f gazebo-crazyflie`, MAVProxy prints
repeatedly:
```
AP: PreArm: Motors: Check frame class and type
AP: Arm:    Motors: Check frame class and type
```
…and arming is refused (`COMPONENT_ARM_DISARM: FAILED`).

### Diagnosis
This message is emitted by `AP_Arming_Copter::motor_checks()` when
`AP_Motors::initialised_ok()` returns false. The motor matrix builds
itself **once at boot**, from whatever `FRAME_CLASS` / `FRAME_TYPE`
are at that instant.

Confirmed via MAVProxy:
```
param show FRAME_CLASS   → 1
param show FRAME_TYPE    → 1
```
Both correct *now* — meaning the parm file (`gazebo-crazyflie.parm`) did
land in EEPROM, but it must have arrived **after** the motor-init step
on the very first boot, so motors initialised with FRAME_CLASS=0 (no
matrix) and have stayed in that bad state since.

### Fix
Soft-reboot the autopilot inside MAVProxy:
```
reboot
```
The autopilot re-reads parameters from EEPROM in the correct order on
the way up and rebuilds the motor matrix.

If `reboot` doesn't clear it (e.g. some other param was missed on
first load), nuke EEPROM and let the parm files load fresh:
```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-crazyflie \
    --model JSON --map --console -w
```
Drop `-w` on subsequent launches.

### Why this happens at all
Order of operations in SITL startup is:
1. ArduPilot boots → motor library initialises from current params.
2. `sim_vehicle.py` pushes the parm file values into EEPROM via MAVLink
   `PARAM_SET` *after* the connection is up.
3. Many params take effect immediately, but the motor matrix isn't
   re-evaluated on parameter change — it needs a reboot.

The `-w` flag avoids the race by ensuring EEPROM is empty at boot, so
the parm-file values are the only ones that ever exist.

---

## 2026-05-18 — Bugfix: propeller inertias were wrong & asymmetric

### Symptom
- First launch: `takeoff 1` → drone shoots out of the simulation.
- Re-launch: `takeoff 1` → `VFR_HUD throttle=100, climb=0, alt unchanged`,
  drone tips and Gazebo crashes.

### Root cause (a bug I introduced)
The original SDF had propeller inertias written as arithmetic strings:
```
<ixx>0.000000002*70</ixx>
<iyy>0.000000167*70</iyy>   <!-- m1 -->
<ixx>0.000000002*50</ixx>
<iyy>0.000000167*50</iyy>   <!-- m2,m3,m4 -->
```
SDF cannot evaluate arithmetic. The XML parser almost certainly read the
first number and discarded `*70` / `*50` — so the *original* SDF was
running with `ixx=2e-9, iyy≈1.67e-7, izz≈1.68e-7` (sane: a thin-rod
approximation of a 2-blade prop) and the `*N` strings were just leftover
notes from the author.

In the initial rewrite I treated them as real multiplications:
- m1: `ixx=1.4e-7, iyy=1.169e-5, izz=1.176e-5`  (×70)
- m2/m3/m4: `ixx=1e-7, iyy=8.35e-6, izz=8.4e-6` (×50)

Two consequences:
1. **iyy/izz are ~70× too high** — rotors are sluggish, accelerate
   slowly, can't develop thrust fast enough.
2. **m1 ≠ m2/m3/m4** — motor 1 has ~40 % more rotational inertia than
   the other three. During the throttle ramp it spins up slower, the
   front-right corner stays heavy, drone tips that direction, hits the
   ground before lift develops.

That matches the observed `throttle=100, climb=0, then topple`.

### Fix
Set all four propellers to the same physically-sane rod inertia:
```
<ixx>2e-9</ixx>
<iyy>1.67e-7</iyy>
<izz>1.68e-7</izz>
```
(`ixx` ≈ 0 along the long axis of the blade; iyy/izz ≈ (1/12)·m·L²
for a 45 mm 2-bladed prop.)

### Still likely to need tuning after this
The original "flew out fast" complaint hinted thrust is high at hover
throttle. After re-launching:
- If drone climbs **>1 m/s** during `takeoff 1`: reduce `<area>` in the
  8 lift-drag plugins (`8e-5` → `2e-5` first cut), and/or lower
  `MOT_THST_HOVER` from `0.55` toward `0.3`.
- If drone **still won't lift**: increase `<area>`, or raise the
  `<multiplier>` PIDs (`p_gain` from `0.005` toward `0.02`) so rotors
  hit target velocity faster.

---

## 2026-05-18 — Tuning session: arming jitter, pre-arm yaw spin, no-liftoff

Four distinct symptoms were observed and fixed in sequence. Files changed:
`crazyflie/model.sdf`, `ardupilot/Tools/autotest/default_params/gazebo-crazyflie.parm`.

---

### Symptom 1 — Drone jitters on arming

#### Root cause
The ArduPilotPlugin velocity PID had `p_gain=0.005` and `cmd_max=0.05` N·m.
At arm, the target rotor velocity is `multiplier × MOT_SPIN_ARM ≈ 131 rad/s`.
PID error at startup = 131; desired force = `0.005 × 131 = 0.655 N·m`, immediately
clamped to `0.05`. The controller was deeply saturated: it applied maximum force,
the rotor overshot, reversed, repeated — producing bang-bang oscillation that
transmitted as visible body jitter.

`controlVelocitySlowdownSim=1` made it worse by scaling simulation timesteps
proportional to commanded velocity during spin-up, creating non-uniform physics
steps the IMU saw as sudden attitude jolts.

Joint `damping=1e-6` (effectively zero) meant rotor overshoot transferred to the
body undamped.

#### Fix (`crazyflie/model.sdf`)
| Parameter | Before | After |
|---|---|---|
| `p_gain` (×4 channels) | `0.005` | `0.02` |
| `cmd_max` / `cmd_min` (×4) | `±0.05` | `±0.5` |
| joint `damping` (×4 joints) | `1e-6` | `0.001` |
| `controlVelocitySlowdownSim` (×4) | `1` | `0` |

---

### Symptom 2 — Drone jitters and yaws before arming, right after SITL reads IMU

#### Root cause
Two independent issues:

**A) `servo_min=1100` but disarmed PWM = 1000.**
When SITL first connects (after IMU calibration), it outputs the disarmed servo
value of 1000 μs. With `servo_min=1100` the plugin normalises this as:
```
cmd = (1000 − 1100) / (1900 − 1100) = −0.125
```
giving a target velocity of `2618 × (−0.125) = −327 rad/s` — all four rotors
commanded to spin in their *wrong* direction at near-full speed. The resulting
reversed lift-drag forces destabilised the body immediately on connection.

**B) `no_time_sync=1` with `lock_step=1`.**
With `no_time_sync` set, the plugin did not throttle Gazebo to wall-clock time.
During the several seconds ArduPilot spent on IMU calibration (while Gazebo was
paused in lock_step), connecting SITL caused the plugin to burst through many
rapid physics steps, producing a large impulse jolt visible as the initial jitter.

#### Fix (`crazyflie/model.sdf`)
| Parameter | Before | After |
|---|---|---|
| `servo_min` (×4 channels) | `1100` | `1000` |
| `no_time_sync` | `1` | removed |

With `servo_min=1000`, disarmed PWM=1000 maps to `cmd=0` → velocity=0. Motors
stay still until SITL actually arms.

---

### Symptom 3 — Residual low-amplitude jitter after fixes above

#### Root cause
Even with the PID and timing fixes in place, the drone body was very light
(25 g) and the joint damping was still low enough that small numerical
asymmetries in the physics solver at simulation start produced visible motion.

#### Fix (`crazyflie/model.sdf`, `gazebo-crazyflie.parm`)
Body mass increased 4× and joint damping increased 50× to make the body
harder to disturb and absorb rotor spin-up transients.

| Parameter | Before | After |
|---|---|---|
| body `mass` | `0.025 kg` | `0.1 kg` |
| body inertia ixx/iyy/izz | `16.6/16.7/29.3 ×10⁻⁶` | `66.3/66.6/117 ×10⁻⁶` (×4) |
| joint `damping` (×4) | `0.001` | `0.05` |

---

### Symptom 4 — Rotors spin after `takeoff` but drone does not lift

#### Root cause
Two consequences of the mass/damping change above, both preventing liftoff:

**A) Damping=0.05 capped rotor speed at 10 rad/s.**
The ArduPilotPlugin force PID is saturated whenever the velocity error exceeds
`cmd_max / p_gain = 0.5 / 0.02 = 25 rad/s`. While saturated, the net force on
the joint is `cmd_max − damping × ω`. This reaches zero at:
```
ω_limit = cmd_max / damping = 0.5 / 0.05 = 10 rad/s
```
The rotors were visually spinning (10 rad/s is perceptible) but generating
essentially zero thrust against a 1.01 N weight.

**B) `area=8e-5` was sized for the original 25 g drone.**
Even if the rotors could reach full speed (2618 rad/s), the total thrust with
`area=8e-5` is only ~0.48 N against the new 1.01 N weight — insufficient even
at 100% throttle.

#### Fix (`crazyflie/model.sdf`, `gazebo-crazyflie.parm`)
| Parameter | Before | After | Reason |
|---|---|---|---|
| joint `damping` (×4) | `0.05` | `0.0001` | Saturation equilibrium now at 5000 rad/s > 2618 |
| lift-drag `area` (×8) | `8e-5` | `3.2e-4` | ×4 to match ×4 mass; gives T/W ≈ 1.92 at full throttle |
| `MOT_THST_HOVER` | `0.55` | `0.72` | Calculated hover point: ω=1890 rad/s at 0.1 kg with new area |

After this change, run SITL once with `-w` to wipe EEPROM so the new
`MOT_THST_HOVER` loads cleanly:
```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-crazyflie \
    --model JSON --map --console -w
```

#### Remaining tuning notes
- If climb rate on `takeoff 5` is too aggressive: reduce `area` toward `2e-4`.
- If drone oscillates in roll/pitch during hover: reduce `ATC_RAT_PIT_P` /
  `ATC_RAT_RLL_P` from `0.08` toward `0.05`.
- `MOT_THST_HOVER` will self-correct after a few hover cycles; the 0.72 value
  is a calculated starting point to prevent ArduCopter from refusing takeoff.


