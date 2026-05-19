# Commands Reference

---

## Standalone gimbal_small_1d — MAVLink bridge control

### 1. Launch the Gazebo world

```bash
source assignment/env.sh
gz sim assignment/worlds/gimbal_runway.sdf -r
```

`-r` starts the simulation running immediately (no need to press Play).

### 2. Verify simulation is live

```bash
# Clock should show incrementing sim time
gz topic -e -t /clock --duration 2

# Camera topics confirm the model loaded
gz topic -l | grep tilt
# Expected:
# /world/gimbal_runway/model/gimbal_small_1d/link/tilt_link/sensor/camera/camera_info
# /world/gimbal_runway/model/gimbal_small_1d/link/tilt_link/sensor/camera/image
```

### 3. Start the MAVLink bridge

```bash
source assignment/env.sh
python3 assignment/gimbal_mavlink_bridge.py
# Output: [bridge] listening on udpin:0.0.0.0:14551  (sysid=1 compid=154)
#         [bridge] ready — waiting for commands
```

### 4. Send gimbal commands

```bash
# Sweep demo — moves from -45° to +45° and back
python3 assignment/gimbal_control.py --sweep

# Single angle (negative = camera tilts down)
python3 assignment/gimbal_control.py -- -45
python3 assignment/gimbal_control.py 30

# Interactive prompt
python3 assignment/gimbal_control.py --interactive
# Enter pitch in degrees [-90, 90]  (Ctrl-D to quit)
# pitch> -30
# pitch> 0
```

### 5. Test joint control directly (bypass bridge)

```bash
# Publish a joint angle in radians straight to Gazebo
gz topic -t /gimbal/tilt_cmd -m gz.msgs.Double -p "data: 0.785"   # +45°
gz topic -t /gimbal/tilt_cmd -m gz.msgs.Double -p "data: -0.785"  # -45°
gz topic -t /gimbal/tilt_cmd -m gz.msgs.Double -p "data: 0.0"     # level
```

> Note: `/gimbal/tilt_cmd` will not appear in `gz topic -l` because
> `JointPositionController` is a subscriber — it only shows once published to.

---

## RViz Live Camera Feed

### 1. Bridge Gazebo camera topic to ROS2

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /gimbal_camera@sensor_msgs/msg/Image[gz.msgs.Image
```

### 2. Launch RViz2

```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

In the RViz GUI: **Add** → **By topic** → `/gimbal_camera` → **Image** → OK

### Alternative — rqt_image_view (simpler)

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /gimbal_camera
```

### Verify topic is live

```bash
ros2 topic list | grep gimbal
ros2 topic hz /gimbal_camera
```

> Camera runs at 10 Hz. Ensure Gazebo is running and simulation is unpaused.
