# Stage TB4 V2 — fase 1
Modelo experimental 2D de TB4 Standard. Conserva bitmap y pose inicial del laboratorio.
El mundo nuevo no necesita robots.inc. Dimensiones de envolvente aproximadas,
huella eliptica poligonal, LiDAR 640 muestras y 360 grados, desplazamiento x=-0.04 m,
yaw 90 grados. Altura TF y origen angular real deben contrastarse en el equipo.
El reloj Stage sigue con interval_sim 100 ms; no se intenta reproducir 62 Hz.
La odometria inicial es ideal (gps de Stage), no ruido real. No hay dock funcional aun.

En terminal limpia, con Stage y Humble ya instalados:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
ros2 run stage_ros2 stage_ros2 --ros-args -p world_file:="$HOME/patrullaje_stage_v2/stage/laboratorio/laboratorio_tb4.world" -p enforce_prefixes:=false -p one_tf_tree:=true -r /base_scan:=/scan
```
No ejecutar junto con Gazebo ni con otros Stage. No iniciar el supervisor
original: sector frontal y geometria de Collision Monitor necesitan ajuste.
Validar topic list, scan, odom, TF y teleop primero. URDF Standard se integrara
sin duplicar TF; evitar joint_state_publisher o robot_state_publisher hasta
conocer los frames y publishers de Stage.
Dock virtual requiere pose y geometria validadas, deteccion IR simplificada,
exclusion mutua de comandos, Collision Monitor, cancelacion y tiempos de espera.
Compilacion/ejecucion local pendientes; ninguna prueba en hardware.
