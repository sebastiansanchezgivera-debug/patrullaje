"""Launch compartido: no inicia metas hasta ARM; Nav2 se activa tras localizar."""
import math
import os
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        value = arg(name).lower()
        if value not in ('true', 'false'):
            raise RuntimeError(name + ' debe ser true o false')
        return value == 'true'

    home = Path.home()
    sim = arg('mode') == 'stage'
    if not sim and arg('mode') != 'real':
        raise RuntimeError('mode debe ser stage o real')
    if not sim:
        if not flag('validated_real'):
            raise RuntimeError('Hardware: validar configuraciones, red, interfaces y seguridad antes de validated_real:=true')
        if os.environ.get('ROS_LOCALHOST_ONLY') == '1' or 'config_ros2/cyclonedds.xml' in os.environ.get('CYCLONEDDS_URI', ''):
            raise RuntimeError('No utilizar entorno_stage.sh para robot fisico')
    else:
        if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
            raise RuntimeError('Stage: cargar ~/entorno_stage.sh antes del launch')

    def file_arg(name, default):
        path = Path(arg(name) or default).expanduser()
        if not path.is_file():
            raise RuntimeError(f'Falta archivo {name}: {path}')
        return str(path.resolve())

    config = home / 'config_ros2'
    model = home / 'modelos_tb4'
    map_file = file_arg('map', home / 'mapas_robot_real/laboratorio/mapa_real.yaml')
    nav = file_arg('nav_params', config / ('nav2_tb4_stage.yaml' if sim else 'nav2_tb4_real.yaml'))
    amcl = file_arg('amcl_params', config / ('amcl_stage.yaml' if sim else 'amcl_real.yaml'))
    collision = file_arg('collision_params', config / ('collision_monitor_tb4.yaml' if sim else 'collision_monitor_real.yaml'))
    if not sim and 'example' in Path(collision).name:
        raise RuntimeError('No usar plantilla Collision Monitor sin validar')
    cp = yaml.safe_load(Path(collision).read_text())['collision_monitor']['ros__parameters']
    if cp.get('cmd_vel_in_topic') != '/cmd_vel_raw' or cp.get('cmd_vel_out_topic') != '/cmd_vel':
        raise RuntimeError('Cadena de velocidad requerida: /cmd_vel_raw -> /cmd_vel')
    if not cp.get('polygons') or not cp.get('observation_sources'):
        raise RuntimeError('Collision Monitor requiere zonas y fuentes')
    patrol = flag('start_patrol')
    if patrol:
        mission = file_arg('mission_params', config / ('mission_params_stage_tb4.yaml' if sim else 'mission_params_real.yaml'))
        mp = yaml.safe_load(Path(mission).read_text())['tb4_autonomous_patrol']['ros__parameters']
        if mp.get('simulate_battery') is not False or mp.get('simulate_docking') is not False:
            raise RuntimeError('Configurar bateria y docking mediante interfaces ROS, no temporizadores')
        if mp.get('enable_stage_docking_actions', False) != sim or mp.get('enable_real_docking', False) == sim:
            raise RuntimeError('Autorizacion docking no coincide con modo Stage/real')
        for prefix in ('station_1', 'station_2', 'station_3', 'home'):
            for axis in ('x', 'y', 'yaw'):
                value = mp.get(prefix+'_'+axis)
                if not isinstance(value, (int, float)) or not math.isfinite(value) or value == -9999.0:
                    raise RuntimeError('Coordenadas no configuradas: '+prefix+'_'+axis)
    clock = {'use_sim_time': sim}
    actions = []
    if sim:
        world = file_arg('world', home/'patrullaje_stage_v2/stage/laboratorio/laboratorio_dock_tb4.world')
        robot_desc = file_arg('robot_description_params', model/'robot_description.yaml')
        robot_urdf = file_arg('robot_urdf', model/'turtlebot4_stage.urdf')
        dock_desc = file_arg('dock_description_params', model/'dock_description.yaml')
        dock_params = file_arg('dock_params', config/'dock_stage.yaml')
        actions += [
            Node(package='stage_ros2', executable='stage_ros2', output='screen',
                 parameters=[{'world_file': world, 'enforce_prefixes': False}],
                 remappings=[('/base_scan', '/scan')]),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 output='screen', parameters=[robot_desc, clock]),
            Node(package='joint_state_publisher', executable='joint_state_publisher',
                 arguments=[robot_urdf], parameters=[clock], output='screen'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='dock_state_publisher', output='screen',
                 parameters=[dock_desc, clock],
                 remappings=[('robot_description','/dock_description'),
                             ('joint_states','/dock_joint_states')]),
            Node(package='tb4_stage_simulator', executable='create3_sim', output='screen',
                 parameters=[dock_params, clock]),
        ]
    actions += [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[clock, {'yaml_filename': map_file}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[amcl, clock], output='screen'),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', parameters=[collision, clock], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', parameters=[clock, {
                 'autostart': True, 'node_names': ['map_server','amcl']}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_collision', parameters=[clock, {
                 'autostart': True, 'node_names': ['collision_monitor']}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('tb4_autonomous_patrol'))/'launch/navigation_safety.launch.py')),
            launch_arguments={'use_sim_time': str(sim).lower(), 'autostart': 'false',
                              'use_composition': 'False', 'params_file': nav}.items()),
    ]
    if flag('rviz'):
        rviz_config = Path(arg('rviz_config') or config/('stage_tb4.rviz' if sim else 'real_tb4.rviz')).expanduser()
        args = ['-d', str(rviz_config)] if rviz_config.is_file() else []
        actions.append(Node(package='rviz2', executable='rviz2', arguments=args,
                            parameters=[clock], output='screen'))
    if patrol:
        actions.append(Node(package='tb4_autonomous_patrol', executable='patrol_runner',
                            output='screen', parameters=[mission, clock,
                                {'require_operator_arm': True}]))
    actions.append(LogInfo(msg='Primero 2D Pose Estimate en RViz; despues STARTUP de lifecycle_manager_navigation y ARM. Sin metas simultaneas.'))
    return actions


def generate_launch_description():
    defaults = {'mode':'stage', 'validated_real':'false', 'start_patrol':'true',
                'rviz':'true', 'rviz_config':'', 'world':'', 'map':'',
                'nav_params':'', 'amcl_params':'', 'collision_params':'',
                'mission_params':'', 'dock_params':'', 'robot_description_params':'',
                'robot_urdf':'', 'dock_description_params':''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v)
        for k,v in defaults.items()] + [OpaqueFunction(function=build)])
