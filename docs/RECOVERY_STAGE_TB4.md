# Recuperacion Stage TB4
El supervisor obtiene TF base_link <- laser y la comparte con SafeRecovery.
Se convierten los angulos de sector y coordenadas de impacto al frame de la base.
TF ausente o parametros invalidos invalidan el despeje. El giro requiere scan
reciente y huella libre; Nav2 /spin y Collision Monitor conservan sus comprobaciones.
Primera prueba: enable_spin_recovery=true, enable_backup_recovery=false.
No se garantiza evasion: el robot puede volver a esperar si no hay salida segura.
No publicar directamente a /cmd_vel. No deshabilitar Collision Monitor.
El archivo local con estaciones debe conservarse: actualizar solo los parametros indicados.
Compilacion y prueba pendientes en ROS 2 Humble; no habilitar en hardware sin validar.
