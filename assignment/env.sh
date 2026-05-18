# Source this file before launching Gazebo: `source env.sh`
# Sets the resource and plugin search paths so `gz sim` can find the
# crazyflie model and the ArduPilotPlugin.

# Tells ardupilot_gazebo's CMake which Gazebo branch to compile against.
# This box has Garden (libgz-sim7). Without this, CMake defaults to
# Harmonic (gz-sim8) and fails with "target gz-sim::gz-sim not found".
export GZ_VERSION=garden

# Folder containing models (must contain a directory named exactly `crazyflie`
# because the SDF references mesh URIs like model://crazyflie/meshes/...).
export GZ_SIM_RESOURCE_PATH=$HOME/botlab_ws/src/assignment:\
$HOME/botlab_ws/src/assignment/worlds:\
$HOME/botlab_ws/src/ardupilot_gazebo/models:\
$HOME/botlab_ws/src/ardupilot_gazebo/worlds:\
${GZ_SIM_RESOURCE_PATH}

# ArduPilotPlugin (.so) lives in the ardupilot_gazebo build directory.
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/botlab_ws/src/ardupilot_gazebo/build:\
${GZ_SIM_SYSTEM_PLUGIN_PATH}
