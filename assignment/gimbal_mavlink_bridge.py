#!/usr/bin/env python3
"""
MAVLink ↔ Gazebo bridge for the standalone gimbal_small_1d model.

Runs as a minimal MAVLink gimbal component (sysid=1, compid=154).
Receives MAV_CMD_DO_MOUNT_CONFIGURE and MAV_CMD_DO_MOUNT_CONTROL from
gimbal_control.py and translates pitch angle → gz topic pub on
/gimbal/tilt_cmd (gz.msgs.Double, radians).

Usage
-----
  python3 gimbal_mavlink_bridge.py

Then in a second terminal:
  python3 gimbal_control.py --sweep
  python3 gimbal_control.py -45
  python3 gimbal_control.py --interactive
"""

import math
import subprocess
import threading
import time

from pymavlink import mavutil

LISTEN_ADDR  = "udpin:0.0.0.0:14551"
GZ_TOPIC     = "/gimbal/tilt_cmd"
HEARTBEAT_HZ = 1.0

# MAVLink component identity
SYS_ID  = 1
COMP_ID = mavutil.mavlink.MAV_COMP_ID_GIMBAL   # 154


def _gz_pub(angle_rad: float) -> None:
    """Publish a Double message to the Gazebo joint topic."""
    msg = f"data: {angle_rad:.6f}"
    subprocess.Popen(
        ["gz", "topic", "-t", GZ_TOPIC, "-m", "gz.msgs.Double", "-p", msg],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _heartbeat_loop(mav: mavutil.mavfile) -> None:
    while True:
        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GIMBAL,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        time.sleep(1.0 / HEARTBEAT_HZ)


def _handle_mount_configure(mav: mavutil.mavfile, msg) -> None:
    mode = int(msg.param1)
    print(f"[bridge] MOUNT_CONFIGURE  mode={mode}")
    mav.mav.command_ack_send(
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
    )


def _handle_mount_control(mav: mavutil.mavfile, msg) -> None:
    pitch_deg = float(msg.param1)
    pitch_deg = max(-90.0, min(90.0, pitch_deg))
    pitch_rad = math.radians(pitch_deg)
    print(f"[bridge] MOUNT_CONTROL  pitch={pitch_deg:+.1f}°  ({pitch_rad:+.4f} rad)  → {GZ_TOPIC}")
    _gz_pub(pitch_rad)
    mav.mav.command_ack_send(
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
    )


def run() -> None:
    print(f"[bridge] listening on {LISTEN_ADDR}  (sysid={SYS_ID} compid={COMP_ID})")
    mav = mavutil.mavlink_connection(
        LISTEN_ADDR,
        source_system=SYS_ID,
        source_component=COMP_ID,
    )

    hb_thread = threading.Thread(target=_heartbeat_loop, args=(mav,), daemon=True)
    hb_thread.start()

    print("[bridge] ready — waiting for commands")
    while True:
        msg = mav.recv_match(
            type=["COMMAND_LONG"],
            blocking=True,
            timeout=5.0,
        )
        if msg is None:
            continue

        cmd = msg.command
        if cmd == mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE:
            _handle_mount_configure(mav, msg)
        elif cmd == mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL:
            _handle_mount_control(mav, msg)


if __name__ == "__main__":
    run()
