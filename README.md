# Patrullaje autónomo con ROS 2 Humble, Nav2, Stage y Groot2

Repositorio exportado desde el código que funciona en la PC de origen.

## Qué contiene
- `tb4_autonomous_patrol/`: fuente completo del paquete (CPP, cabeceras, BT XML, YAML, launch y scripts).
- `config/nav2_stage.yaml`: parámetros de Nav2, si estaban en la máquina de origen.
- `maps/`: YAML e imagen del mapa, si se localizaron o se indicó `MAP_YAML`.
- `stage/`: mundo Stage propio, si se indicó `STAGE_WORLD`.

## Requisitos en otro equipo
- Ubuntu 22.04 / ROS 2 Humble; colcon, rosdep, Nav2, slam_toolbox y RViz2.
- BehaviorTree.CPP 4 compatible (en la PC de origen se usaba 4.10).
- Stage y `stage_ros2`: dependencia externa que hay que instalar por separado.
- Si se usa un TurtleBot4 físico: drivers, firmware, red y configuraciones de hardware apropiadas.

## Instalar el paquete en otra PC
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
# Tras clonar este repo en ~/tb4-patrullaje-portable:
ln -s ~/tb4-patrullaje-portable/tb4_autonomous_patrol ~/ros2_ws/src/tb4_autonomous_patrol
rosdep install --from-paths ~/ros2_ws/src/tb4_autonomous_patrol --ignore-src -r -y --rosdistro humble
cd ~/ros2_ws
colcon build --packages-select tb4_autonomous_patrol --symlink-install
source ~/ros2_ws/install/setup.bash
```

Si ya existe `~/ros2_ws/src/tb4_autonomous_patrol`, no crees el symlink sin comprobarlo.

El paquete **no es una instalación completa de ROS, Nav2 ni Stage**. Los comandos de arranque de Stage
son específicos del paquete stage_ros2 y del mundo utilizado; documentarlos o añadir un launch propio.

## Condiciones de seguridad
- El supervisor detecta obstáculo frontal con `stop_distance: 0.45` y
  `front_half_angle_deg: 15.0`, y solicita cancelar Nav2.
- El monitor `ApproachFootprint` de Stage no garantiza una zona STOP frontal fija a 0.45 m.
- Antes de operar, comprobar que `/cmd_vel` tenga exclusivamente a Collision Monitor como publicador,
  y que Stage sea suscriptor; verifique tambien TF, sensores y distancias.
- `allow_partial_scan_sim_only` y `simulate_docking` no son parámetros seguros para hardware real.

El mapa de Stage solo es reutilizable en el robot físico si corresponde al entorno real y permite
localización válida. Redefinir estaciones para cada mapa.

## Adaptación física (noviembre de 2026)

La versión del código ya incorpora armado manual, inicialización desde `/dock_status` y un perfil físico separado.

- [Manual de adaptación, instalación, SLAM y seguridad](docs/ADAPTACION_FISICA_Y_MANUAL.md)
- [Parámetros TurtleBot4 real](tb4_autonomous_patrol/config/mission_params_real.yaml)
- [Plantilla de Collision Monitor (NO ejecutar directamente)](tb4_autonomous_patrol/config/collision_monitor_real.example.yaml)
- [Diagnóstico de hardware sin movimiento](tb4_autonomous_patrol/scripts/check_real_preflight.sh)

**Importante:** todavía no se ha compilado ni probado esta modificación en un TurtleBot4 físico. No utilizar valores de ejemplo para habilitar navegación. El comando `STOP` del supervisor solicita regresar a HOME y **no** equivale al e-stop físico.
