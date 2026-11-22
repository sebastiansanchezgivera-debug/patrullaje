# TB4 Autonomous Patrol — paquete ROS 2 independiente

**Paquete nuevo**: `tb4_autonomous_patrol`. No busca ni importa `tb4_station_mission`.
Conserva Node2 Nav2 y TurtleBot4 Stage como infraestructura ROS2 instalada en Ubuntu.
El código de patrullaje y el XML de Groot2 viven completamente dentro de este paquete.

## Requisitos

- Ubuntu 22.04 + ROS 2 Humble.
- `behaviortree_cpp` 4.10.0 (ya comprobado en el equipo), rclcpp, rclcpp_action, nav2_msgs, sensor_msgs, geometry_msgs, std_msgs, ament_index_cpp.
- Nav2 activo (`/navigate_to_pose`) y AMCL/localización activa; Stage simulado con `/base_scan`.
- Collision Monitor funcionando entre `/cmd_vel_raw` y `/cmd_vel`. No se elimina ni desactiva.
- Para usar Groot2 en tiempo real se requiere una edición de Groot2 con monitorización.

## Instalación (sin scripts que busquen el paquete viejo)

En WSL:

```bash
# Detén el ejecutable viejo antes de iniciar el nuevo.
mkdir -p ~/ros2_ws/src
cp -a /RUTA_DONDE_DESCOMPRIMISTE/tb4_autonomous_patrol ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_autonomous_patrol --symlink-install --event-handlers console_direct+
source ~/ros2_ws/install/setup.bash
ros2 pkg executables tb4_autonomous_patrol
```

Debe aparecer `tb4_autonomous_patrol patrol_runner`.

**Si la carpeta de este nuevo paquete está ya dentro de src**, no la copies sobre sí misma. Compila directamente.

## Arranque de misión

IMPORTANTE: configura primero `home_x` y `home_y` con un punto libre del mapa, usando RViz Publish Point. El valor `-9999` significa «sin configurar»; STOP y batería baja no pueden ir a un objetivo inválido.

Con Stage, Nav2, AMCL y Collision Monitor activos y **solo Collision Monitor publicando /cmd_vel**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run tb4_autonomous_patrol patrol_runner --ros-args \
  --params-file ~/ros2_ws/src/tb4_autonomous_patrol/config/mission_params.yaml
```

Groot2 Monitor: `localhost:1667`. Ejecuta solo un `patrol_runner` a la vez. En el árbol la rama `Estado_En_Tiempo_Real` señala el estado activo (solo visualización); el supervisor conserva el estado y coordina Nav2.

### Comandos del operador

```bash
# Estado, patrullaje continuo (1→2→3→1...)
ros2 topic echo /mission/status --no-daemon
# Parar patrullaje, ir a home (simula estación de carga), permanecer en espera
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "STOP"}'
# Retomar estación pendiente después del STOP o al recuperarse batería
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "RESUME"}'
# Terminar definitivamente (cancela objetivo activo; no va a home)
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "CANCEL"}'
# Solicitar replanteamiento de ruta SOLO cuando bloqueado por un obstáculo
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "AVOID"}'
# Dejar de intentar una evasión y permanecer detenido
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "WAIT"}'
# Cancelar emergencia y retomar patrulla pendiente
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "EMERGENCY_CANCEL"}'
```

AVOID **no es una maniobra de retroceso**. Nav2 puede volver a calcular ruta, pero Collision Monitor bloquea las velocidades si sigue habiendo algo dentro de 0,45m. No hay retroceso automático a ciegas.

### Emergencia mediante RViz

```bash
python3 ~/ros2_ws/src/tb4_autonomous_patrol/scripts/emergency_from_rviz.py
```

Haz clic con «Publish Point» en RViz para enviar el punto de emergencia. El robot acudirá y esperará allí hasta `EMERGENCY_CANCEL`. Si se cancela antes de llegar, interrumpe el objetivo y vuelve a la estación de patrulla pendiente. El bridge debe estar apagado mientras selecciones estaciones o la base de carga.

Alternativa sin puente:

```bash
ros2 topic pub --once /mission/emergency_goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

La coordenada `10,0` de ejemplo **no está verificada** para tu mapa.

### Simular batería en Stage y ajustar valores

```bash
ros2 param set /tb4_autonomous_patrol sim_battery_pct 19.0
ros2 param set /tb4_autonomous_patrol sim_battery_pct 30.0
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "RESUME"}'
```

La batería real utiliza `sensor_msgs/msg/BatteryState` de acuerdo con `battery_topic`; `percentage` es fracción [0,1]. Ajusta `simulate_battery: false` y `use_sim_time: false` para hardware real. Sin batería fresca o LiDAR fresco no envía objetivos nuevos y solicita cancelar el activo.

### Estaciones, cargador, espera y umbrales dinámicos

```bash
ros2 param set /tb4_autonomous_patrol station_2_x 19.9
ros2 param set /tb4_autonomous_patrol station_2_y -0.04
ros2 param set /tb4_autonomous_patrol station_2_wait_sec 5.0
ros2 param set /tb4_autonomous_patrol home_x 0.0
ros2 param set /tb4_autonomous_patrol home_y 0.0
ros2 param set /tb4_autonomous_patrol battery_threshold_pct 20.0
ros2 param set /tb4_autonomous_patrol battery_resume_pct 25.0
```

La posición `home=0,0` es un ejemplo, **no lo uses sin comprobar su ubicación**. Los cambios de objetivos activos provocan cancelación y nuevo envío; los tiempos de espera se aplican al entrar en la estación.

### Integración con Nav2 y Collision Monitor sin depender del paquete anterior

Incluimos nuestro propio `config/collision_monitor_stage.yaml`, con los vértices numéricos admitidos por Humble, ±15° y 0,45 m desde el LiDAR ubicado a x=0,15m respecto a base_link. Verifica TF antes de hardware real.

```bash
# Sólo si todavía no tienes el filtro activo:
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py \
  params_file:=$HOME/ros2_ws/src/tb4_autonomous_patrol/config/collision_monitor_stage.yaml \
  use_sim_time:=True use_composition:=False
```

Nav2 oficial usa un archivo `navigation_launch.py` instalado en /opt/ros/humble. El script incluido crea un lanzamiento seguro propio:

```bash
python3 ~/ros2_ws/src/tb4_autonomous_patrol/scripts/prepare_nav2_safety.py
ros2 launch ~/ros2_ws/src/tb4_autonomous_patrol/launch/navigation_safety.launch.py \
  params_file:=$HOME/ros2_ws/config/nav2_stage.yaml \
  use_sim_time:=True use_composition:=False
```

**Requiere que exista** `~/ros2_ws/config/nav2_stage.yaml`, creado en el trabajo anterior. Es un archivo de Nav2, no del paquete anterior. Si ya tienes Nav2 funcionando con remapeos seguros, no hace falta reiniciarlo. No lances dos Nav2 a la vez.

### Limitaciones explícitas antes de hardware real

1. En Stage, HOME significa llegar a una pose del mapa; **no** acoplarse realmente al cargador. En TurtleBot4 físico hay que añadir docking/undocking y verificar confirmación de carga.
2. En modo AVOID se reintenta navegación con Nav2 sin desactivar Collision Monitor. No garantiza liberar un obstáculo dentro de la zona de parada ni hace marcha atrás a ciegas.
3. `Ctrl+C` mata el proceso; usa `CANCEL` para una cancelación normal supervisada. Ante emergencia física utiliza una parada segura independiente.
4. Tiempos de respuesta, sensores caducados, latencia de cancelación, tolerancias y acoplamiento requieren ensayo y validación en Stage y robot real. Collision Monitor no sustituye una función certificada de seguridad.
5. Groot2 muestra la rama `Estado_En_Tiempo_Real` con indicadores activos para espera, evasión, retorno, emergencia y patrullaje. Son **indicadores**, no acciones de seguridad; las decisiones se realizan en `MissionSupervisor` y en Collision Monitor.
