#!/usr/bin/env python3
"""
MAVLink interface for a 1-DOF (pitch-only) servo gimbal on Crazyflie.

MAVLink commands used
---------------------
MAV_CMD_DO_MOUNT_CONFIGURE (205)
    Sets the mount into MAVLink-targeting mode so subsequent
    DO_MOUNT_CONTROL commands are accepted.

MAV_CMD_DO_MOUNT_CONTROL (205→206)
    Commands gimbal pitch (param1) and yaw (param3) in degrees.
    This script only exercises pitch; yaw is always 0 for a 1-DOF mount.

Hardware path
-------------
  Python script
    → MAVLink UDP 14550
      → ArduPilot SITL (mount subsystem → SERVO5 PWM output)
        → ArduPilotPlugin channel 4 (POSITION type)
          → gimbal_tilt_joint in Gazebo

Usage
-----
  python3 gimbal_control.py               # sweep demo
  python3 gimbal_control.py -- -45        # tilt to -45° (camera down)
  python3 gimbal_control.py 30            # tilt to +30°
  python3 gimbal_control.py --interactive # interactive prompt
  python3 gimbal_control.py --conn tcp:127.0.0.1:5760  # custom connection
"""

import argparse
import math
import sys
import time
from pymavlink import mavutil

# Bridge listens on 14551; connect outbound so the bridge knows our address.
DEFAULT_CONN = "udpout:127.0.0.1:14551"

# Must match MNT1_PITCH_MIN / MNT1_PITCH_MAX in crazyflie_gimbal.parm
PITCH_MIN_DEG = -90.0
PITCH_MAX_DEG =  90.0


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

def connect(conn_str: str) -> mavutil.mavfile:
    print(f"[gimbal] connecting to {conn_str} …")
    mav = mavutil.mavlink_connection(conn_str)
    mav.wait_heartbeat(timeout=30)
    print(f"[gimbal] heartbeat  sys={mav.target_system}  comp={mav.target_component}")
    return mav


# ---------------------------------------------------------------------------
# Parameter helpers
# ---------------------------------------------------------------------------

def _param_set(mav: mavutil.mavfile, name: str, value: float) -> None:
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def push_mount_params(mav: mavutil.mavfile) -> None:
    """
    Upload gimbal parameters at runtime (alternative to loading a .parm file).
    Useful when testing without restarting SITL.
    """
    params = {
        "MNT1_TYPE":        1,
        "MNT1_PITCH_MIN":   PITCH_MIN_DEG,
        "MNT1_PITCH_MAX":   PITCH_MAX_DEG,
        "SERVO5_FUNCTION":  7,    # Mount1Pitch
        "SERVO5_MIN":    1000,
        "SERVO5_MAX":    2000,
        "SERVO5_TRIM":   1500,
        "SERVO5_REVERSED":  0,
    }
    for name, value in params.items():
        _param_set(mav, name, value)
        time.sleep(0.05)
    print(f"[gimbal] {len(params)} mount params pushed")


# ---------------------------------------------------------------------------
# MAV_CMD_DO_MOUNT_CONFIGURE
# ---------------------------------------------------------------------------

def mount_configure(mav: mavutil.mavfile) -> None:
    """
    Switch Mount 1 to MAVLink-targeting mode.

    param1 = MAV_MOUNT_MODE_MAVLINK_TARGETING (4)
      The autopilot accepts pitch/yaw targets from MAV_CMD_DO_MOUNT_CONTROL
      rather than from an RC input channel or GPS point.

    param3 = 1  →  stabilise pitch (the single DOF we have).
    """
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
        0,                                                   # confirmation
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,   # param1: mode
        0,                                                   # param2: stabilise roll
        1,                                                   # param3: stabilise pitch
        0,                                                   # param4: stabilise yaw
        0, 0, 0,
    )
    print("[gimbal] MAV_CMD_DO_MOUNT_CONFIGURE  →  MAVLink-targeting mode")


# ---------------------------------------------------------------------------
# MAV_CMD_DO_MOUNT_CONTROL
# ---------------------------------------------------------------------------

def mount_control(mav: mavutil.mavfile,
                  pitch_deg: float,
                  yaw_deg: float = 0.0) -> None:
    """
    Command gimbal pitch (and yaw) via MAV_CMD_DO_MOUNT_CONTROL.

    param1  pitch in degrees  (negative = camera tilts down toward nadir)
    param2  roll  in degrees  (always 0 — 1-DOF gimbal has no roll axis)
    param3  yaw   in degrees  (0 = forward)
    param7  MAV_MOUNT_MODE   (must repeat mode; some FW versions require it)

    ArduPilot clamps to [MNT1_PITCH_MIN, MNT1_PITCH_MAX]; we clamp here too
    so the caller gets predictable behaviour regardless of param upload order.
    """
    pitch_deg = float(max(PITCH_MIN_DEG, min(PITCH_MAX_DEG, pitch_deg)))
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,
        pitch_deg,                                           # param1: pitch (deg)
        0.0,                                                 # param2: roll  (deg)
        yaw_deg,                                             # param3: yaw   (deg)
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,   # param7: mode
    )


# ---------------------------------------------------------------------------
# Demo routines
# ---------------------------------------------------------------------------

def sweep(mav: mavutil.mavfile,
          low: float = -45.0,
          high: float = 45.0,
          step: float = 5.0,
          delay: float = 0.15) -> None:
    """Sweep pitch from `low` to `high` and back."""
    fwd = [low + i * step for i in range(int((high - low) / step) + 1)]
    for angle in fwd + list(reversed(fwd[:-1])):
        mount_control(mav, angle)
        print(f"  pitch = {angle:+.1f}°")
        time.sleep(delay)


def interactive(mav: mavutil.mavfile) -> None:
    """Read pitch values from stdin until EOF."""
    print(f"Enter pitch in degrees [{PITCH_MIN_DEG}, {PITCH_MAX_DEG}]  (Ctrl-D to quit)")
    while True:
        try:
            raw = input("pitch> ").strip()
        except EOFError:
            break
        if not raw:
            continue
        try:
            pitch = float(raw)
        except ValueError:
            print("  ← not a number")
            continue
        mount_control(mav, pitch)
        print(f"  sent pitch = {pitch:+.1f}°")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="MAVLink 1-DOF gimbal controller (pitch via DO_MOUNT_CONTROL)"
    )
    parser.add_argument(
        "pitch", nargs="?", type=float, default=None,
        help="Target pitch in degrees. Omit to run sweep demo.",
    )
    parser.add_argument(
        "--sweep", action="store_true",
        help="Sweep from -45° to +45° and back.",
    )
    parser.add_argument(
        "--interactive", "-i", action="store_true",
        help="Prompt for pitch values interactively.",
    )
    parser.add_argument(
        "--conn", default=DEFAULT_CONN, metavar="URI",
        help=f"MAVLink connection string (default: {DEFAULT_CONN})",
    )
    parser.add_argument(
        "--push-params", action="store_true",
        help="Upload mount parameters before commanding (skips .parm file requirement).",
    )
    args = parser.parse_args()

    mav = connect(args.conn)

    if args.push_params:
        print("[gimbal] --push-params is only needed with ArduPilot SITL; skipping for bridge mode")

    mount_configure(mav)
    time.sleep(0.3)

    if args.interactive:
        interactive(mav)
    elif args.sweep or args.pitch is None:
        print("[gimbal] sweep demo  -45° → +45° → -45°")
        sweep(mav)
    else:
        print(f"[gimbal] commanding pitch = {args.pitch:+.1f}°")
        mount_control(mav, args.pitch)
        time.sleep(1.0)

    print("[gimbal] done")


if __name__ == "__main__":
    main()
