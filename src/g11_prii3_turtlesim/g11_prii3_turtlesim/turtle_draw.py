#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class Turtle11(Node):
    def __init__(self):
        super().__init__('turtle_11')
        self.get_logger().info("Iniciando nodo para dibujar el número 11...")

        # Publicador para mover la tortuga
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Clientes de servicio
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.setpen_cli = self.create_client(SetPen, '/turtle1/set_pen')

        # Esperar a que los servicios estén disponibles
        self.get_logger().info("Esperando servicios de turtlesim...")
        self.teleport_cli.wait_for_service()
        self.setpen_cli.wait_for_service()

        # Dibujar el número 11
        self.draw_11()

    def teleport(self, x, y, theta=1.57):
        """Teletransporta tortuga a (x, y) mirando hacia arriba."""
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        future = self.teleport_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)

    def set_pen(self, r, g, b, width=5, off=False):
        """Cambia el color o activa/desactiva el lápiz."""
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = 1 if off else 0
        future = self.setpen_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.3)

    def move_up(self, distance=5.0, speed=1.0):
        """Mueve la tortuga hacia arriba dibujando."""
        msg = Twist()
        msg.linear.x = speed
        steps = int(distance / (speed * 0.1))
        for _ in range(steps):
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        msg.linear.x = 0.0
        self.cmd_pub.publish(msg)
        time.sleep(0.5)

    def draw_11(self):
        self.get_logger().info("Iniciando dibujo del número 11...")

        # --- Primer palo ---
        self.get_logger().info("Primer palo...")
        self.set_pen(255, 255, 255, 5, off=True)
        self.teleport(2.5, 2.0)  # Más a la izquierda
        self.set_pen(255, 255, 255, 5, off=False)
        self.move_up(6.0)  # Más largo

        # --- Segundo palo ---
        self.get_logger().info("Segundo palo...")
        self.set_pen(255, 255, 255, 5, off=True)
        self.teleport(6.5, 2.0)  # Más separado del primero
        self.set_pen(255, 255, 255, 5, off=False)
        self.move_up(6.0)

        self.get_logger().info("Número 11 dibujado correctamente (líneas blancas y gruesas).")

def main(args=None):
    rclpy.init(args=args)
    node = Turtle11()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

