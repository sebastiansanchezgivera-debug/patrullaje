#!/usr/bin/env python3
"""Convertir RViz Publish Point (/clicked_point) en un destino de emergencia.
Uso: source ~/ros2_ws/install/setup.bash && python3 scripts/emergency_from_rviz.py
Con este programa activo, cada clic de Publish Point activa una emergencia.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

class EmergencyFromRViz(Node):
    def __init__(self):
        super().__init__('emergency_from_rviz')
        self.pub = self.create_publisher(PoseStamped, '/mission/emergency_goal', 10)
        self.sub = self.create_subscription(
            PointStamped, '/clicked_point', self.handle_point, 10)
        self.get_logger().info(
            'Listo. Selecciona Publish Point en RViz y haz clic en el mapa.')

    def handle_point(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().error('Se requiere Fixed Frame: map')
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)
        self.get_logger().info(
            f'Emergencia enviada: x={msg.point.x:.3f}, y={msg.point.y:.3f}')

def main():
    rclpy.init()
    node = EmergencyFromRViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
