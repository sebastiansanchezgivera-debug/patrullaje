#!/usr/bin/env python3
"""Diagnostico de Gazebo; no envia metas ni velocidades."""
import os
import sys
import time
import math
if (os.environ.get("ROS_DISTRO") != "humble"
        or os.environ.get("ROS_DOMAIN_ID") != "42"
        or os.environ.get("ROS_LOCALHOST_ONLY") != "1"):
    sys.exit("ERROR: cargar Humble, ROS_DOMAIN_ID=42 y ROS_LOCALHOST_ONLY=1.")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException

def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec

def main():
    rclpy.init()
    node = Node("gazebo_patrol_preflight",
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    counts = dict.fromkeys(("clock", "scan", "odom", "map"), 0)
    first, latest = {}, {}
    scan_frame = ""
    def record(key, ns):
        counts[key] += 1
        first.setdefault(key, ns)
        latest[key] = ns
    def scan_cb(msg):
        nonlocal scan_frame
        scan_frame = msg.header.frame_id
        if msg.ranges and msg.angle_increment > 0 and any(not math.isnan(r) and r >= 0 for r in msg.ranges):
            record("scan", stamp_ns(msg.header.stamp))
    node.create_subscription(Clock, "/clock",
                             lambda m: record("clock", stamp_ns(m.clock)), qos_profile_sensor_data)
    node.create_subscription(LaserScan, "/scan", scan_cb, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom",
                             lambda m: record("odom", stamp_ns(m.header.stamp)), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, "/map",
                             lambda m: record("map", stamp_ns(m.header.stamp)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    buf = Buffer(node=node)
    listener = TransformListener(buf, node)
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    started = time.monotonic()
    try:
        print("Observando sensores 30 segundos reales; sin enviar metas.", flush=True)
        while rclpy.ok() and time.monotonic() - started < 30:
            rclpy.spin_once(node, timeout_sec=0.2)
        checks = {key: counts[key] >= 2 and latest.get(key, 0) > first.get(key, 0)
                  for key in ("clock", "scan", "odom")}
        checks["map"] = counts["map"] > 0
        for key, ok in checks.items():
            print(f"{'OK' if ok else 'FALLO'} /{key}: {counts[key]} mensajes")
        tf_ok = bool(scan_frame)
        pairs = [("odom", "base_link"), ("map", "odom")]
        if scan_frame:
            pairs.append(("base_link", scan_frame))
        for target, source in pairs:
            try:
                tf = buf.lookup_transform(target, source, Time())
                print(f"OK TF {target} -> {source}")
                if target == "base_link":
                    q = tf.transform.rotation
                    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    print(f"  Orientacion LiDAR: {math.degrees(yaw):.2f} grados")
            except TransformException as exc:
                tf_ok = False
                print(f"FALLO TF {target} -> {source}: {exc}")
        nav_ok = client.server_is_ready()
        print(f"{'OK' if nav_ok else 'FALLO'} /navigate_to_pose")
        print("No se ha validado cadena cmd_vel, coordenadas ni seguridad de movimiento.")
        return 0 if all(checks.values()) and tf_ok and nav_ok else 1
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(main())
