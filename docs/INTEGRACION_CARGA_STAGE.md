# Integracion patrullaje y Create 3 en Stage
Solo entorno Stage aislado, con /clock, AMCL, Nav2, Collision Monitor y create3_sim.
Actualizar parametros locales sin reemplazar coordenadas de las estaciones.
use_sim_time=true, simulate_battery=false, simulate_docking=false,
enable_real_docking=false, enable_stage_docking_actions=true,
auto_resume_after_charge=true, battery_threshold_pct=20.0, battery_resume_pct=80.0.
Para primera prueba se puede usar 25% de reanudacion. STOP mantiene espera hasta RESUME.
Comenzar con robot desacoplado, /dock_status reciente e is_docked=false.
El supervisor no emite navegacion si DockStatus no es reciente o indica acoplamiento
no gestionado; para iniciar desde dock se necesita desacoplar manualmente antes.
La accion Dock solo comienza despues de resultado Nav2 de HOME; el servidor simulado
usa /cmd_vel_raw y confirma pose, no contacto fisico.
No ejecutar teleop ni metas/acciones manuales mientras el supervisor controla el robot.
No usar en hardware. Compilacion y prueba de integracion pendientes.
