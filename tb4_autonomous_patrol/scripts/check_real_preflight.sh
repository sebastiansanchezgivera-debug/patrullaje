#!/usr/bin/env bash
# Diagnostico en modo lectura: no publica velocidades ni lanza navegación.
set -u
printf '\n=== ROS / RED ===\n'
printf 'ROS_DISTRO=%s ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s\n' "${ROS_DISTRO:-NO_CONFIGURADO}" "${ROS_DOMAIN_ID:-NO_CONFIGURADO}" "${RMW_IMPLEMENTATION:-NO_CONFIGURADO}"
if [ "${ROS_DISTRO:-}" != humble ]; then
  echo 'ATENCION: comprobar version ROS; preparado para Humble.'
fi
printf '\n=== TOPICOS / ACCIONES ===\n'
ros2 topic list -t --no-daemon | grep -E '^/(scan|odom|battery_state|dock_status|cmd_vel|cmd_vel_raw|hazard_detection)' || true
ros2 action list -t | grep -E '^/(dock|undock|spin|navigate_to_pose)' || true
printf '\n=== DETALLE ===\n'
for t in /scan /odom /battery_state /dock_status; do
  echo "--- $t ---"
  ros2 topic info "$t" --no-daemon || true
done
printf '\n=== VELOCIDADES ===\n'
ros2 topic info /cmd_vel_raw --verbose --no-daemon || true
ros2 topic info /cmd_vel --verbose --no-daemon || true
printf '\n=== MONITOR ===\n'
ros2 node list --no-daemon 2>/dev/null | grep collision_monitor || true
echo 'No se enviaron comandos de movimiento.'
echo 'Verificar TF, huella, LiDAR, bateria, estado dock, boton e-stop y un solo filtro de velocidad antes de arrancar.'
