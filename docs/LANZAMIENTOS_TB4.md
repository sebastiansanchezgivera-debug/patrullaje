# Lanzamientos Stage y robot real
## Stage
Guardar RViz como ~/config_ros2/stage_tb4.rviz (opcional).
Cerrar los nodos anteriores. source ~/entorno_stage.sh y el overlay
~/ros2_stage_tb4_ws/install/setup.bash. Lanzar simulation_stage.launch.py.
El launch usa los archivos locales existentes, no pisa coordenadas ni geometria.
Map Server/AMCL/Collision Monitor se activan automaticamente. Nav2 queda
sin activar hasta seleccionar 2D Pose Estimate y ejecutar:
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
Verificar pose, nodos activos, dock_status desacoplado, datos recientes y que no
hay metas de Nav2 ni teleop. ARM inicia supervisor:
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: 'ARM'}"
Groot2 monitor puerto 1667. CANCEL espera fin de acciones gestionadas; antes de
Ctrl+C cancelar y verificar que el robot esta detenido.
## Hardware
robot_real.launch.py no inicia driver, Stage ni Create3 virtual. Requiere
driver fisico funcionando, entorno de red real, mapa y archivos validados:
~/config_ros2/amcl_real.yaml, nav2_tb4_real.yaml, collision_monitor_real.yaml,
mission_params_real.yaml (coordenadas validadas y acciones autorizadas).
No copiar ajustes reducidos de Stage. No duplicar Nav2/localizacion ya existente.
validacion explicita validated_real:=true; ARM sigue siendo necesario.
No es una certificacion de seguridad: ARM comprueba datos, TF, estados y
publicador final en el momento de autorizar, no verifica fisicamente el entorno
ni garantiza ausencia de otros clientes de accion. Probar con operador, e-stop,
sin personas cerca y con recuperacion/auto-resume deshabilitadas inicialmente.
No conectar simulacion y robot fisico al mismo dominio. No usar entorno_stage.sh.
Compilacion y pruebas pendientes en el equipo del usuario.
