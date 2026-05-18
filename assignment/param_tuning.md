# Parameter Tuning Reference

Two files hold all tunable parameters:
- **`crazyflie/model.sdf`** — physics and plugin config (Gazebo side)
- **`ardupilot/Tools/autotest/default_params/gazebo-crazyflie.parm`** — flight controller config (ArduPilot side)

After changing the `.parm` file, wipe EEPROM once so values load cleanly:
```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-crazyflie --model JSON --map --console -w
```
Drop `-w` on subsequent runs. After changing `model.sdf`, just restart Gazebo.

---

## 1. Rotor velocity PID (`model.sdf` → ArduPilotPlugin `<control>` blocks)

These govern how the ArduPilotPlugin drives each joint to reach the target rotor speed.

### `p_gain` (currently `0.02`)
- **What it does**: proportional gain of the rotor velocity PID. Higher = faster spin-up.
- **Too low**: rotors are sluggish, drone can't build thrust fast enough → tips over on takeoff.
- **Too high**: rotors overshoot their target speed → oscillating thrust → body jitter.
- **Range to try**: `0.005` – `0.10`

### `cmd_max` / `cmd_min` (currently `±0.5`)
- **What it does**: caps the maximum torque (N·m) the PID can apply to a joint per step.
- **Too low**: PID saturates early, rotor speed is capped at `cmd_max / damping`. At `cmd_max=0.5` and `damping=0.0001`, cap = 5000 rad/s (safe). If you raise damping, raise this too.
- **Too high**: not usually a problem, but extreme values cause instant spin-up and overshoot.
- **Rule**: always keep `cmd_max / damping > 2618` (your max multiplier).

### `multiplier` (currently `±2618`)
- **What it does**: maps normalised throttle (0–1) to target rotor velocity in rad/s. `|multiplier|` = peak rotor speed. Sign encodes spin direction (+ = CCW, − = CW).
- **Too low**: not enough RPM to generate hover thrust → drone lifts sluggishly or not at all.
- **Too high**: rotors overspeed, lift-drag forces go quadratic → drone shoots upward.
- **2618 rad/s ≈ 25 000 RPM** — correct for a Crazyflie prop. Change only if you rescale the prop model.

### `servo_min` / `servo_max` (currently `1000` / `1900`)
- **What it does**: defines the PWM range ArduPilot uses. The plugin maps `servo_min` → velocity 0, `servo_max` → `multiplier`.
- **Critical**: `servo_min` must equal the PWM ArduPilot sends when disarmed (1000). If `servo_min > 1000`, disarmed output maps to negative velocity → rotors spin backward on connection → pre-arm yaw spin.
- **Do not change** unless you change `RC_MIN` / servo trim on the ArduPilot side to match.

---

## 2. Joint dynamics (`model.sdf` → inside each `<joint>`)

### `damping` (currently `0.0001`)
- **What it does**: viscous friction on the rotor joint (N·m·s/rad). Resists rotation proportional to speed.
- **Too low** (< ~1e-4): rotors oscillate around target speed → rotor-frequency vibration → body jitter.
- **Too high** (> `cmd_max / 2618`): rotor speed is capped below max. At `damping=0.05` and `cmd_max=0.5`, cap = 10 rad/s → drone cannot lift.
- **Safe range**: `0.00005` – `0.0002`. Hard upper limit: `cmd_max / 2618 ≈ 0.000191`.
- **Jitter fix**: increase toward `0.0002`. **No-liftoff fix**: decrease toward `0.00005`.

---

## 3. Lift-drag parameters (`model.sdf` → `<plugin gz-sim-lift-drag-system>`)

These appear 8 times (2 blades × 4 rotors). Change all 8 together.

### `area` (currently `3.2e-4`)
- **What it does**: effective blade area (m²). Thrust scales linearly with area.
- **Too low**: not enough thrust to lift → rotors spin but drone stays on ground.
- **Too high**: excess thrust at low throttle → drone shoots up, controller can't hold altitude.
- **Hover thrust check**: at hover throttle (72% of 2618 rad/s), total thrust should ≈ weight (0.1 kg × 9.81 ≈ 1.01 N).
- **Range to try**: `1.5e-4` – `5e-4`

### `a0` (currently `0.3` rad ≈ 17°)
- **What it does**: blade pitch angle (angle of attack at zero airspeed). All lift in hover comes from this term since kinematic alpha is zero for a horizontal spinning prop.
- **Too low**: less lift per RPM → need higher throttle to hover.
- **Too high**: more lift per RPM → drone overshoots altitude aggressively.
- **Interact with `area`**: reducing `area` and increasing `a0` by the same factor produces the same hover throttle but changes the shape of the thrust curve.

### `cla` (currently `4.25`)
- **What it does**: lift curve slope (per radian). Scales how much lift increases with `a0`.
- Lift per blade ≈ `cla × a0 × 0.5 × rho × v² × area`. Changing `cla` has the same effect as changing `area × a0` together.
- **Leave alone** unless you want to model a different airfoil.

### `cda` (currently `0.10`)
- **What it does**: drag coefficient. Drag force opposes blade motion, which creates a **yaw torque** on the body (reaction torque). CCW rotors create CW yaw torque, CW rotors create CCW torque — these should cancel in balanced hover.
- **If drone yaws slowly**: check that all 4 rotors have the same `cda`. If CW and CCW pairs have different effective drag (e.g. due to wrong `forward` vectors), there's a net yaw.
- **Increasing `cda`**: more drag torque → tighter yaw response but more power consumed.

### `cp` (currently `0.012 0 0` / `-0.012 0 0`)
- **What it does**: center of pressure location — the distance from the hub at which the lift-drag force is applied. For a 45 mm prop, ≈ blade midpoint (22.5 mm → 0.0225 m). Currently set to 12 mm (a bit conservative).
- **Increasing `|cp|`**: same area generates more thrust moment → more yaw authority.
- **Leave alone** unless you change prop diameter.

---

## 4. Body inertia (`model.sdf` → `crazyflie/body` `<inertial>`)

### `mass` (currently `0.1 kg`)
- **What it does**: total body mass. Heavier = more inertia = harder to disturb.
- **Too light** (< 0.05 kg in sim): small physics solver noise causes visible jitter.
- **Too heavy** (> 0.15 kg): thrust/weight drops; may need to re-tune `area` and `MOT_THST_HOVER`.
- **If you change mass**: scale `ixx/iyy/izz` by the same ratio, and recalculate `MOT_THST_HOVER`.

### `ixx`, `iyy`, `izz`
- **What they do**: rotational inertia around X (roll), Y (pitch), Z (yaw) axes.
- **Too low**: attitude controller sees very fast angular response → oscillation.
- **Too high**: drone is sluggish to attitude commands.
- **Keep proportional to mass** unless you're deliberately changing the drone's shape.

---

## 5. ArduPilot flight controller params (`gazebo-crazyflie.parm`)

### `MOT_THST_HOVER` (currently `0.72`)
- **What it does**: ArduCopter's initial estimate of the throttle fraction needed to hover. Used to feed-forward the right throttle when switching to altitude-hold or guided modes.
- **Too low**: drone climbs aggressively on every mode switch (overshoots altitude).
- **Too high**: drone sinks on mode switch before the controller catches it.
- **Self-correcting**: ArduCopter learns the real value after a few stable hover cycles. Set it close and let it converge.
- **How to recalculate**: `hover_throttle = hover_omega / multiplier`, where `hover_omega = sqrt(weight / (8 × cla × a0 × 0.5 × rho × area)) / cp`.

### `MOT_SPIN_ARM` (currently `0.05`)
- **What it does**: fraction of max throttle the motors spin at when armed but not flying. Keeps motors "warm."
- **Too high**: drone lifts off as soon as it arms.
- **Too low**: rotors stop and start unevenly → small lurch at takeoff.

### `MOT_SPIN_MIN` (currently `0.10`)
- **What it does**: minimum throttle while in flight. Below this, ArduCopter treats the motor as off.
- **If drone tips on landing**: increase slightly so motors stay active through touchdown.

### `MOT_THST_EXPO` (currently `0.10`)
- **What it does**: thrust curve shape. 0 = perfectly linear, 1 = very exponential (squared).
- Crazyflie props are nearly linear → keep low (0.0 – 0.15).

---

## 6. Attitude rate PID (`gazebo-crazyflie.parm`)

These are the inner-loop PIDs. Symptoms appear as oscillation, sluggishness, or drift.

### `ATC_RAT_PIT_P` / `ATC_RAT_RLL_P` (currently `0.08`)
- **What it does**: proportional gain on pitch/roll rate error.
- **Too high**: fast oscillation in pitch/roll (drone rocks back and forth at ~5–20 Hz).
- **Too low**: drone is slow to correct attitude — drifts and can't hold position.
- **Range**: `0.03` – `0.15`

### `ATC_RAT_PIT_I` / `ATC_RAT_RLL_I` (currently `0.08`)
- **What it does**: integrator — corrects steady-state lean errors.
- **Too high**: slow low-frequency oscillation / wobble (integrator windup).
- **Too low**: drone leans persistently to one side in hover.

### `ATC_RAT_PIT_D` / `ATC_RAT_RLL_D` (currently `0.0015`)
- **What it does**: derivative — damps oscillation.
- **Too high**: high-frequency buzzing / jitter (D amplifies noise).
- **Too low**: underdamped oscillation after disturbance.

### `ATC_RAT_YAW_P` (currently `0.18`)
- **What it does**: yaw rate P gain.
- **If drone spins on its axis**: first check motor mapping and lift-drag `cda` balance. If those are correct, this gain may be too high — try `0.10`.
- **Range**: `0.05` – `0.30`

### `ATC_ANG_PIT_P` / `ATC_ANG_RLL_P` (currently `6.0`)
- **What it does**: outer-loop angle P gain. Higher = snappier attitude tracking.
- **Too high**: feeds too much rate demand into the inner loop → oscillation.
- **Range**: `3.0` – `8.0`

---

## 7. Sensor filters (`gazebo-crazyflie.parm`)

### `INS_GYRO_FILTER` (currently `50` Hz)
- **What it does**: low-pass filter cutoff on gyro data.
- **Too high**: high-frequency vibration (from rotor imbalance or physics noise) passes to the rate controller → D-term amplifies it → jitter.
- **Too low**: controller reacts slowly → sluggish attitude response.
- **If jitter persists after all other fixes**: lower to `20` Hz first.

### `ATC_RAT_PIT_FLTD` / `ATC_RAT_RLL_FLTD` (currently `40` Hz)
- **What it does**: D-term notch filter cutoff.
- **Decrease** if D-term is amplifying rotor-frequency noise. Try `20` Hz.

---

## Quick symptom → parameter map

| Symptom | First parameter to try |
|---|---|
| Jitters on ground / before arm | `damping` ↑ (but keep < `cmd_max/2618`) |
| Yaws before arming | Check `servo_min=1000`; check `cda` is equal for all 8 lift-drag plugins |
| Rotors spin but won't lift | `area` ↑ or `damping` ↓ |
| Shoots up immediately on takeoff | `area` ↓ or `MOT_THST_HOVER` ↓ |
| Oscillates in roll/pitch during hover | `ATC_RAT_PIT_P` / `ATC_RAT_RLL_P` ↓ |
| Drifts / can't hold position | `ATC_RAT_PIT_I` / `ATC_RAT_RLL_I` ↑ |
| High-frequency buzzing in flight | `INS_GYRO_FILTER` ↓ or `ATC_RAT_PIT_FLTD` ↓ |
| Sluggish attitude response | `ATC_ANG_PIT_P` ↑ or `ATC_RAT_PIT_P` ↑ |
| Yaws slowly during hover | `ATC_RAT_YAW_P` ↑ or check `cda` balance |
| Won't arm — "Motors: Check frame class and type" | Run `reboot` in MAVProxy, or add `-w` once to wipe EEPROM |
