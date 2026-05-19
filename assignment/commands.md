# Commands Reference

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
