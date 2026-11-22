#!/usr/bin/env bash
# Comandos ilustrativos, no ejecuta nada automaticamente.
set -euo pipefail
printf '%s\n' "1) Copiar tb4_autonomous_patrol a ~/ros2_ws/src/"
printf '%s\n' "2) cd ~/ros2_ws && colcon build --packages-select tb4_autonomous_patrol --symlink-install"
printf '%s\n' "3) source ~/ros2_ws/install/setup.bash"
printf '%s\n' "4) ros2 run tb4_autonomous_patrol patrol_runner --ros-args --params-file ~/ros2_ws/src/tb4_autonomous_patrol/config/mission_params.yaml"
