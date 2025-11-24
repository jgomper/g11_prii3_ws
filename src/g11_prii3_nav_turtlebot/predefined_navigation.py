#!/usr/bin/env python3
"""
predefined_navigation.py
Nodo ROS para navegacion predefinida EN SIMULACIÓN - SIN NAV2
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class PredefinedNavigation(Node):
    def __init__(self):
        super().__init__('predefined_navigation')
        
        self.get_logger().info("Inicializando Navegacion Predefinida F1L3 - SIMULACION")
        
        # Publisher para controlar el robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Waypoints predefinidos (coordenadas x, y en el mundo Gazebo)
        self.waypoints = [
            (2.0, 0.0, "Pasillo central"),      # Waypoint 1
            (4.0, 1.0, "Puerta laboratorio"),   # Waypoint 2  
            (5.0, 3.0, "Interior laboratorio")  # Waypoint 3
        ]
        
        # Ejecutar la secuencia de navegacion
        self.execute_navigation_sequence()
        
    def move_forward(self, distance):
        """Mover el robot hacia adelante una distancia especifica"""
        self.get_logger().info(f"Moviendo hacia adelante {distance} metros")
        
        # Crear mensaje de velocidad
        twist = Twist()
        twist.linear.x = 0.5  # Velocidad lineal moderada
        
        # Calcular tiempo necesario para la distancia
        move_time = distance / twist.linear.x
        
        # Publicar comando por el tiempo calculado
        start_time = time.time()
        while (time.time() - start_time) < move_time:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        
        # Detener el robot
        self.stop_robot()
        time.sleep(1)
    
    def turn(self, angle_degrees):
        """Girar el robot un angulo especifico"""
        self.get_logger().info(f"Girando {angle_degrees} grados")
        
        twist = Twist()
        twist.angular.z = 0.5 if angle_degrees > 0 else -0.5  # Velocidad angular
        
        # Calcular tiempo necesario para el angulo (aproximado)
        turn_time = abs(angle_degrees) * math.pi / (180 * twist.angular.z)
        
        # Publicar comando por el tiempo calculado
        start_time = time.time()
        while (time.time() - start_time) < turn_time:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        
        # Detener el robot
        self.stop_robot()
        time.sleep(1)
    
    def stop_robot(self):
        """Detener completamente el robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
    
    def execute_navigation_sequence(self):
        """Ejecutar secuencia de navegacion predefinida"""
        self.get_logger().info("Iniciando secuencia de navegacion predefinida")
        
        try:
            # Secuencia de navegacion
            self.get_logger().info("Navegando a: Pasillo central")
            self.move_forward(25.5)
            
            self.get_logger().info("Navegando a: Puerta laboratorio") 
            self.turn(125)  # Girar hacia la puerta
            time.sleep(1)
            self.move_forward(6.0)
            
            self.get_logger().info("Navegando a: Interior laboratorio")
            #self.move_forward(7.5)
            
            self.get_logger().info("SECUENCIA COMPLETADA - Robot en posicion final")
            
        except Exception as e:
            self.get_logger().error(f"Error en la navegacion: {e}")
        finally:
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    nav_node = PredefinedNavigation()
    
    # Mantener el nodo activo hasta que termine la secuencia
    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.stop_robot()
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()