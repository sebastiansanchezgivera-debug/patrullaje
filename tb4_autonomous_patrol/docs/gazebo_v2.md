# Gazebo V2 — fase 1 (experimental, solo simulacion)

No usar en el robot fisico. El supervisor inicia automaticamente: NO lanzar
patrol_runner hasta medir las tres estaciones y HOME en el mapa, verificar
Collision Monitor y cadena cmd_vel y disponer de una forma de detener Gazebo.

En cada terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb
unset ROS_DISCOVERY_SERVER
```

Iniciar simulador:
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite world:=maze slam:=true nav2:=true rviz:=true
```

Pulsar Play. No iniciar otra instancia Nav2. Desde el directorio del paquete:
```bash
python3 scripts/check_gazebo_preflight.py
ros2 topic info /cmd_vel -v
ros2 action list -t
```

Preflight observa mensajes 30 s reales; no envia metas. Un OK confirma
disponibilidad basica, no certifica seguridad ni frescura TF. Si falta mapa o
map->odom, comprobar SLAM. Perfil independiente con /scan, /odom, tiempo
simulado, coordenadas pendientes, bateria y docking por temporizador y
recuperaciones desactivadas. La opcion front_scan_in_base_frame corrige el
sector angular por yaw de TF del LiDAR fijo, sin compensar traslacion ni
sustituir Collision Monitor. Por defecto desactivada en otros perfiles.
El watchdog de LiDAR sigue en tiempo monotono real (puede bloquear la mision
si el RTF es bajo); medir primero. Armado manual, compilacion y pruebas pendientes.
