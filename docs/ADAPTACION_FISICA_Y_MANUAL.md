# Adaptación Stage → TurtleBot4 físico (ROS 2 Humble)

**Estado:** fuente actualizado; despliegue físico aún sin compilar/probar en el TurtleBot4 real. **NO** ejecutar patrullaje con valores de ejemplo sin validar el hardware.

## Qué se conserva

El árbol `mission.xml`, el supervisor de las 3 estaciones, emergencias, batería, integración Nav2 y Groot2 y las recuperaciones de Stage.

## Cambios hechos

- `src/main.cpp`: `require_operator_arm`; rechaza sensores/docking simulados en modo físico, así como una compilación sin `irobot_create_msgs`.
- `dock_controller.cpp/.hpp`: lectura del estado de carga inicial. El robot no debe moverse hasta recibir `/dock_status` reciente.
- `mission_supervisor.cpp/.hpp`: en físico, `WAITING_DOCK_STATUS` y `AWAITING_ARM`; solo el comando `RESUME` habilita marcha tras batería, LiDAR y cuatro poses configuradas. Si empieza acoplado, ejecuta `/undock` y espera confirmación.
- `config/mission_params_real.yaml`: perfil separado de Stage, incluye `stop_distance: 0.45` y `front_half_angle_deg: 15.0`. Sus estaciones/HOME están deliberadamente a -9999, **NO** habilitadas.
- `config/collision_monitor_real.example.yaml`: **plantilla no validada**, no se debe lanzar directamente.
- `scripts/check_real_preflight.sh`: comprobación sin ordenar movimientos.

## Instalación en otra PC

Ubuntu 22.04, ROS 2 Humble, Nav2, SLAM Toolbox, RViz2, BehaviorTree.CPP compatible y paquetes de TurtleBot4. Para Create3 real también se requieren las interfaces `irobot_create_msgs` instaladas **antes** de compilar.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/sebastiansanchezgivera-debug/patrullaje.git ~/patrullaje
ln -s ~/patrullaje/tb4_autonomous_patrol ~/ros2_ws/src/tb4_autonomous_patrol
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --packages-select tb4_autonomous_patrol --symlink-install
source install/setup.bash
```

**No crear el enlace** si ese paquete ya existe en `~/ros2_ws/src`. Stage + `stage_ros2` se instalan por separado.

## Mapeo real (SLAM)

Comprobar red, ROS_DOMAIN_ID, `/scan`, `/odom`, TF y batería. Detener AMCL. En terminales separadas:

```bash
ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=false
ros2 launch turtlebot4_viz view_robot.launch.py
```

Teleoperar lentamente bajo vigilancia. Guardar:

```bash
mkdir -p ~/ros2_ws/maps/laboratorio_real
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/laboratorio_real/mapa_laboratorio
```

## Localización y navegación sobre un mapa guardado

**Detener SLAM**. Verificar launches compatibles con instalación Humble:

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=$HOME/ros2_ws/maps/laboratorio_real/mapa_laboratorio.yaml use_sim_time:=false
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=false
ros2 launch turtlebot4_viz view_robot.launch.py
```

En RViz usar 2D Pose Estimate y comprobar `map → odom → base_link`; probar meta simple con operador presente. Antes de patrullar validar cadena de velocidades `/cmd_vel_raw → Collision Monitor → /cmd_vel` y sus publicadores; no duplicar el monitor oficial.

## Comprobación física sin movimiento

```bash
bash ~/ros2_ws/src/tb4_autonomous_patrol/scripts/check_real_preflight.sh
ros2 action info /dock
ros2 action info /undock
ros2 topic echo /dock_status --once
ros2 run tf2_ros tf2_echo odom base_link
```

Editar `mission_params_real.yaml` con las 3 estaciones, HOME, geometría LiDAR y robot reales. `enable_spin_recovery` está desactivado hasta comprobar giro seguro.

Solo después de validar mapa, Nav2, Collision Monitor, sensores y dispositivos físicos de parada:

```bash
ros2 run tb4_autonomous_patrol patrol_runner --ros-args --params-file ~/ros2_ws/src/tb4_autonomous_patrol/config/mission_params_real.yaml
ros2 topic echo /mission/status
# Solo con autorización y área segura:
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "RESUME"}'
```

**`STOP` y `PAUSE` del programa solicitan regreso a HOME, no son e-stop físico.** Ante peligro usar el control de seguridad del robot. El supervisor cancela Nav2 cuando el LiDAR ve <=0.45 m a ±15°, pero la latencia real de detención **no está medida**, y el `ApproachFootprint` por sí solo no es una zona STOP fija. El árbol del supervisor tampoco sustituye literalmente al BT personalizado interno de `bt_navigator` de la guía.

## Actualizar repositorio

```bash
cd ~/patrullaje && git status && git pull --ff-only origin main
cd ~/ros2_ws && colcon build --packages-select tb4_autonomous_patrol --symlink-install
```

Manual extenso, tabla de pruebas y los dos PDFs del curso: entregados como archivos descargables en la conversación (el PDF no está publicado aquí).