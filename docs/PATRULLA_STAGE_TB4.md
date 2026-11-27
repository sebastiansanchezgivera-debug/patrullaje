# Patrullaje Stage TB4 — primera prueba
Usar Stage, AMCL, Map Server, Nav2 y Collision Monitor ya validados. El nuevo perfil
usa /scan y /odom sin prefijos y sector frontal angular transformado a base_link.
El cálculo angular supone montaje planar; no corrige la traslación del origen láser.
La clasificación de paredes y las recuperaciones propias Spin/BackUp del supervisor
están desactivadas provisionalmente (no elimina las recuperaciones internas de Nav2).
Conservar Collision Monitor; /cmd_vel debe tener un único publicador autorizado.
Batería a 100% y docking por temporizador provisional; no equivale a las acciones Create 3.
Configurar 3 estaciones y HOME en frame map, nunca usar coordenadas del mapa anterior.
Probar cada destino en Nav2 antes de iniciar. El supervisor comienza automáticamente.
STOP significa regreso a HOME, no parada de emergencia; CANCEL cancela la misión.
La rama requiere compilación y pruebas en ROS 2 Humble; no usar en hardware.
