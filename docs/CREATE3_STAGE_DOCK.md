# Stage Create 3: fase 2
Acciones /dock y /undock con mensajes oficiales, movimiento lento por /cmd_vel_raw,
sin puente de Collision Monitor. Solo Stage aislado. Activacion explicita con
dock_motion_enabled=true. Probar manualmente con supervisor cancelado y Nav2 sin
metas, teleop cerrado, Collision Monitor activo y unico publicador /cmd_vel.
No es acoplamiento de contacto: standoff de 0.33 m entre centro robot y cara dock.
Pose AMCL y comprobacion de eco LiDAR aproximada, no sensores fisicos ni ground truth.
Timeout de pared y vigilancia de /clock, cancelacion y fallo si pierden scan/TF/visibilidad.
No se garantiza evitar interferencia con otros publicadores /cmd_vel_raw: no ejecutar
acciones de navegacion ni teleop simultaneamente. Recarga virtual durante is_docked.
battery_override_pct=-1.0 para permitir carga y descarga; 0..100 fuerza porcentaje.
Undock retrocede al punto de salida, sin giro de 180 grados. Primera prueba manual,
integracion con supervisor y validacion pendientes. No usar en hardware.
