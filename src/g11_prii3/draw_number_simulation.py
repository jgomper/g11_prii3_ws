#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import math
import time

class DrawNumberSimulation(Node):
    def __init__(self):
        super().__init__('draw_number_simulation')
        
        # Publisher para comandos de movimiento
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service para controlar el dibujo
        self.service = self.create_service(SetBool, 'control_drawing', self.control_callback)
        
        # Variables de control - INICIA PAUSADO para no interferir con collision_avoidance
        self.is_drawing = False  # Cambiado a False
        self.is_paused = True    # Cambiado a True
        self.current_step = 0
        
        # Parámetros de movimiento
        self.linear_speed = 0.15
        self.angular_speed = 0.8
        self.side_length = 1.0
        
        self.get_logger().info('Draw Number Simulation node started - Grupo 11')
        self.get_logger().info('Esperando señal para iniciar dibujo...')
        
        # Timer para ejecutar el dibujo
        self.timer = self.create_timer(0.1, self.execute_drawing)
        
    def control_callback(self, request, response):
        """Service callback para controlar el dibujo"""
        if request.data:
            # Reanudar o reiniciar
            if self.is_paused:
                self.is_paused = False
                self.is_drawing = True
                response.success = True
                response.message = "Dibujo reanudado"
                self.get_logger().info("Iniciando dibujo del numero 11...")
            else:
                # Reiniciar
                self.current_step = 0
                self.is_paused = False
                self.is_drawing = True
                response.success = True
                response.message = "Dibujo reiniciado"
                self.get_logger().info("Dibujo reiniciado")
        else:
            # Pausar
            self.is_paused = True
            self.is_drawing = False
            response.success = True
            response.message = "Dibujo pausado"
            self.get_logger().info("Dibujo pausado")
            
        return response
        
    def move_forward(self, distance):
        """Mueve el robot hacia adelante una distancia específica"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        
        start_time = time.time()
        move_time = distance / self.linear_speed
        
        while time.time() - start_time < move_time and not self.is_paused and rclpy.ok():
            self.publisher_.publish(twist)
            time.sleep(0.1)
            
        self.stop_robot()
        
    def rotate(self, angle):
        """Rota el robot un ángulo específico - GIRO DERECHA (-90º)"""
        twist = Twist()
        twist.angular.z = -self.angular_speed  # SIEMPRE GIRO DERECHA (negativo)
        
        start_time = time.time()
        rotate_time = abs(angle) / self.angular_speed
        
        while time.time() - start_time < rotate_time and not self.is_paused and rclpy.ok():
            self.publisher_.publish(twist)
            time.sleep(0.1)
            
        self.stop_robot()
        
    def stop_robot(self):
        """Detiene el robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(0.2)
        
    def execute_drawing(self):
        """Ejecuta la secuencia para dibujar el número 11"""
        if not self.is_drawing or self.is_paused:
            return
            
        # Secuencia optimizada para dibujar el número 11 con GIROS DERECHA
        steps = [
            # Primer 1 - Línea vertical izquierda
            ('forward', self.side_length),
            ('rotate', math.pi/2),  # GIRO DERECHA 90º
            ('forward', self.side_length * 0.3),  # Pequeño desplazamiento
            
            # Segundo 1 - Línea vertical derecha
            ('rotate', math.pi/2),  # GIRO DERECHA 90º (volver a vertical)
            ('forward', self.side_length),
            ('rotate', math.pi/2),  # GIRO DERECHA 90º
            
            # Base horizontal
            ('forward', self.side_length * 0.6),
            ('rotate', math.pi/2),  # GIRO DERECHA 90º
            ('forward', self.side_length * 0.6),
            ('rotate', math.pi/2),  # GIRO DERECHA 90º
            
            # Volver a posición inicial
            ('forward', self.side_length * 0.3),
            ('rotate', math.pi/2),  # GIRO DERECHA 90º
        ]
        
        if self.current_step < len(steps):
            action, value = steps[self.current_step]
            
            if action == 'forward':
                self.get_logger().info(f'Paso {self.current_step + 1}/{len(steps)}: Adelante {value:.1f}m')
                self.move_forward(value)
            elif action == 'rotate':
                degrees = math.degrees(value)
                self.get_logger().info(f'Paso {self.current_step + 1}/{len(steps)}: Giro DERECHA {degrees:.0f}º')
                self.rotate(value)
                
            self.current_step += 1
            
            if self.current_step >= len(steps):
                self.is_drawing = False
                self.get_logger().info('Dibujo del numero 11 completado')
                
    def start_drawing(self):
        """Inicia el dibujo"""
        self.is_drawing = True
        self.is_paused = False
        self.current_step = 0
        self.get_logger().info("Iniciando dibujo del numero 11...")
                
    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DrawNumberSimulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()