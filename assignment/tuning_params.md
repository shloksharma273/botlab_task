# Crazyflie Tuning Parameters

Parameters are split into two layers: **Gazebo physics** (in `crazyflie/model.sdf`)
and **ArduPilot flight controller** (in `gazebo-crazyflie.parm`).  
Tune the Gazebo layer first — if the physics model is wrong, no amount of FC
tuning will fix the flight.

---

## Layer 1 — Gazebo physics (`crazyflie/model.sdf`)

These live inside the `ArduPilotPlugin` `<control>` blocks and the
`gz-sim-lift-drag-system` plugins.

### Rotor velocity PID

| Parameter | Current | Effect |
|-----------|---------|--------|
| `p_gain` | `0.02` | How fast a rotor reaches its target speed. Too high → rotor oscillates / body jitters on arm. Too low → slow spin-up, drone can't develop thrust fast enough. |
| `cmd_max` | `0.5` | Max torque (N·m) the plugin can apply to a rotor joint. Sets the saturation limit. Raise together with `p_gain` if rotors are slow. |
| `multiplier` | `2618` | Peak rotor speed (rad/s) at PWM 2000. ~25 000 RPM for the Crazyflie. Reduce if the drone launches violently; increase if it can't lift. |

### Lift-drag (thrust model)

| Parameter | Current | Effect |
|-----------|---------|--------|
| `area` | `3.2e-4` | Effective blade area. **Primary thrust knob.** Double it → roughly double thrust. If drone overshoots altitude or climbs too fast, halve it. |
| `cp` | `±0.012` | Centre-of-pressure offset. Controls the torque/yaw authority per rotor. Rarely needs changing. |

### Rotor joint

| Parameter | Current | Effect |
|-----------|---------|--------|
| `damping` | `0.0001` | Mechanical drag on the rotor joint. Too high → rotor hits a speed ceiling below target, no thrust. Keep well below `cmd_max / (p_gain × max_speed)`. |

---

## Layer 2 — ArduPilot (`gazebo-crazyflie.parm`)

### Motor / thrust

| Parameter | Current | Effect |
|-----------|---------|--------|
| `MOT_THST_HOVER` | `0.72` | Throttle fraction at hover (0–1). ArduCopter uses this to centre its throttle stick. If the drone climbs at mid-stick, lower it; if it sinks, raise it. ArduPilot self-learns this after a few hovers. |
| `MOT_SPIN_ARM` | `0.05` | Motor speed when armed, zero throttle. Low enough to be silent but high enough to confirm motors are live. |
| `MOT_SPIN_MIN` | `0.10` | Minimum speed during flight. Prevents a motor from stopping mid-flight on a hard manoeuvre. |
| `MOT_THST_EXPO` | `0.10` | Thrust curve nonlinearity. 0 = linear, 1 = highly curved. Micro motors are nearly linear so this stays low. |

### Rate controller — inner loop

Controls **how fast** the drone reaches a commanded angular rate.
Symptoms: oscillation / wobble at a specific frequency → tune these.

| Parameter | Current | Effect |
|-----------|---------|--------|
| `ATC_RAT_RLL_P` | `0.08` | Roll rate proportional gain. Too high → fast roll oscillation. Too low → sluggish roll response. |
| `ATC_RAT_RLL_I` | `0.08` | Roll rate integral. Corrects steady-state error (e.g. drone drifts sideways at hover). Too high → slow wobble that builds over time. |
| `ATC_RAT_RLL_D` | `0.0015` | Roll rate derivative (damping). Reduces overshoot. Too high → high-frequency buzz. |
| `ATC_RAT_PIT_P/I/D` | same as roll | Same meaning for pitch axis. Usually kept equal to roll on a symmetric frame. |
| `ATC_RAT_YAW_P` | `0.18` | Yaw rate P. Higher than roll/pitch because yaw has less authority. |
| `ATC_RAT_YAW_I` | `0.018` | Yaw rate I. Corrects slow yaw drift. |

### Angle controller — outer loop

Controls **what angle** the drone holds. Feeds into the rate controller above.

| Parameter | Current | Effect |
|-----------|---------|--------|
| `ATC_ANG_RLL_P` | `6.0` | Roll angle P. Higher → snappier response to stick input. Too high → feeds too much rate demand, causes oscillation. |
| `ATC_ANG_PIT_P` | `6.0` | Pitch angle P. Same as roll. |
| `ATC_ANG_YAW_P` | `4.5` | Yaw angle P. Lower than roll/pitch because yaw is slower to respond. |

### Position / velocity controller

Controls GPS-hold, auto modes, and GUIDED flight.

| Parameter | Current | Effect |
|-----------|---------|--------|
| `PSC_POSXY_P` | `1.0` | Horizontal position P. How aggressively the drone corrects horizontal displacement. Too high → oscillates around the target point. |
| `PSC_VELXY_P` | `1.5` | Horizontal velocity P. Inner loop for position. Raise if position hold is sluggish; lower if it overshoots. |
| `PSC_VELXY_I` | `0.5` | Horizontal velocity I. Corrects slow drift in wind or with CG offset. |
| `PSC_POSZ_P` | `1.0` | Vertical position P. Too high → altitude bounces. |
| `PSC_VELZ_P` | `5.0` | Vertical velocity P. Controls climb/descent damping. |
| `PSC_ACCZ_P/I` | `0.5 / 1.0` | Vertical acceleration PID. Fine-tuned last; rarely needs changing on a sim model. |

### Filters

| Parameter | Current | Effect |
|-----------|---------|--------|
| `INS_GYRO_FILTER` | `50` Hz | Low-pass cutoff on gyro data. Lower → smoother but laggier. Higher → more responsive but noisier. In sim noise is low so 50 Hz is fine. |
| `ATC_RAT_PIT_FLTD` | `40` Hz | D-term filter for pitch rate. Prevents D-term from amplifying noise. Lower if the drone buzzes with D enabled. |
| `ATC_RAT_RLL_FLTD` | `40` Hz | Same for roll rate. |

### Pilot limits

| Parameter | Current | Effect |
|-----------|---------|--------|
| `ANGLE_MAX` | `2000` (20°) | Max lean angle. Increase for faster/more aggressive flight; decrease for safer indoor flight. |
| `WPNAV_SPEED` | `200` cm/s | Max horizontal speed in AUTO/GUIDED. |
| `WPNAV_SPEED_UP` | `100` cm/s | Max climb rate. |
| `WPNAV_SPEED_DN` | `100` cm/s | Max descent rate. |

---

## Quick tuning sequence

1. **Can't lift off** → increase `area` in lift-drag plugins, or raise `MOT_THST_HOVER`.
2. **Climbs too fast / overshoots altitude** → decrease `area`, lower `MOT_THST_HOVER`.
3. **Fast oscillation on roll/pitch** → reduce `ATC_RAT_RLL_P` / `ATC_RAT_PIT_P` by 20%.
4. **Slow wobble that builds over time** → reduce `ATC_RAT_RLL_I` / `ATC_RAT_PIT_I`.
5. **High-frequency buzz** → reduce `ATC_RAT_RLL_D` or lower `ATC_RAT_RLL_FLTD`.
6. **Drifts in position hold** → raise `PSC_VELXY_I`.
7. **Jitters on arm (before takeoff)** → `p_gain` or `cmd_max` too high in model.sdf.
