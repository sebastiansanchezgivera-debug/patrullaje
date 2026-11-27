# Stage Create 3: fase 1
Ejecutar solo en Stage aislado, con use_sim_time true, dock_stage.yaml,
AMCL y /scan. No publica velocidades ni ofrece /dock o /undock todavia.
Mantener simulate_battery true en el supervisor hasta integrar acciones.
Bateria 0..1, descarga por minuto simulado aun en reposo.
battery_override_pct (0..100) mantiene un valor de prueba; -1 reanuda descarga.
IR aproximado con TF map/base_link, cara del dock, distancia y oclusion
estimada por un rayo del scan; no reproduce haces reales ni firmware.
Publica BUOY_BOTH como simplificacion. is_docked siempre false en esta fase.
Recarga y acciones pendientes. No usar en hardware.
