#!/usr/bin/env python3
"""Crea un launch seguro a partir del navigation_launch.py instalado por ROS 2 Humble.
No depende de ningun paquete personalizado anterior.
Uso: python3 scripts/prepare_nav2_safety.py
"""
from pathlib import Path
import os
import re
import sys

ros_distro = os.environ.get("ROS_DISTRO", "humble")
source = Path(f"/opt/ros/{ros_distro}/share/nav2_bringup/launch/navigation_launch.py")
out = Path(__file__).resolve().parents[1] / "launch/navigation_safety.launch.py"
if not source.is_file():
    sys.exit(f"ERROR: no existe {source}; instale nav2_bringup o cargue el entorno ROS")
text = source.read_text()

# En Humble existen dos configuraciones: normal y composicion.
old = "('cmd_vel_smoothed', 'cmd_vel')"
if text.count(old) != 2:
    sys.exit("ERROR: Nav2 posee un lanzamiento distinto. No se modificó ningún archivo.")
text = text.replace(old, "('cmd_vel_smoothed', 'cmd_vel_raw')")

# Behavior Server publica velocidades durante recuperaciones. Pasarlas tambien por el filtro.
pattern = r"(package='nav2_behaviors',.*?remappings=)remappings(\))"
text, hits = re.subn(
    pattern,
    r"\1remappings + [('cmd_vel', 'cmd_vel_raw')]\2",
    text,
    flags=re.DOTALL,
)
if hits != 2:
    sys.exit(f"ERROR: se esperaban 2 remapeos de behavior_server, encontrados: {hits}.")

# Comprobar sintaxis antes de sobrescribir el generado
compile(text, str(out), 'exec')
out.write_text(text)
print(f"OK: {out}")
print("Verificar que /cmd_vel tenga un solo publicador: collision_monitor")
