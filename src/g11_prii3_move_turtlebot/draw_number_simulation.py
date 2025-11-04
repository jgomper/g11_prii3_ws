#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class DrawNumberSimulation(Node):
    def __init__(self):
        super().__init__('draw_number_simulation')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.is_drawing = True
        self.current_step = 0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.girando = False
        
        # Variables para controlar el estado
        self.movimiento_completado = False
        self.giro_completado = False
        
        self.get_logger().info('Iniciando dibujo del numero 11 (sin obstáculos)')
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        # Obtener orientacion actual
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        # Calcular yaw
        self.current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
    def mover_adelante(self, segundos):
        twist = Twist()
        twist.linear.x = 0.15
        
        start = time.time()
        while time.time() - start < segundos and rclpy.ok():
            self.publisher_.publish(twist)
            time.sleep(0.1)
        
        self.parar()
        self.movimiento_completado = True
        
    def empezar_giro_90(self, direccion=-1):
        # direccion: -1 para giro izquierda, 1 para giro derecha
        self.target_yaw = self.current_yaw + direccion * math.pi / 2.0  # 90 grados
        self.girando = True
        self.giro_completado = False
        self.get_logger().info(f'Empezando giro de {self.current_yaw:.2f} a {self.target_yaw:.2f}')
        
    def parar(self):
        twist = Twist()
        self.publisher_.publish(twist)
        time.sleep(0.5)
        
    def control_loop(self):
        if not self.is_drawing:
            return
            
        # Control de giro
        if self.girando:
            error = self.target_yaw - self.current_yaw
            # Normalizar error
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi
                
            if abs(error) < 0.05:  # ~3 grados
                self.girando = False
                self.giro_completado = True
                self.parar()
                self.get_logger().info('Giro completado')
                self.ejecutar_siguiente_paso()
            else:
                twist = Twist()
                # Velocidad angular más rápida (0.8 rad/s)
                twist.angular.z = -0.8 if error < 0 else 0.8
                self.publisher_.publish(twist)
            return
            
        # Ejecutar secuencia del número 11
        if self.current_step == 0:
            if not self.movimiento_completado:
                self.get_logger().info('Paso 1: Primer palo (12s)')
                self.mover_adelante(12.0)
            else:
                self.movimiento_completado = False
                self.current_step = 1
            
        elif self.current_step == 1:
            if not self.giro_completado:
                self.get_logger().info('Paso 2: Giro 90 izquierda')
                self.empezar_giro_90(direccion=-1)  # Giro izquierda
            else:
                self.giro_completado = False
                self.current_step = 2
            
        elif self.current_step == 2:
            if not self.movimiento_completado:
                self.get_logger().info('Paso 3: Espacio (5s)')
                self.mover_adelante(3.0)
            else:
                self.movimiento_completado = False
                self.current_step = 3
            
        elif self.current_step == 3:
            if not self.giro_completado:
                self.get_logger().info('Paso 4: Giro 90 izquierda')
                self.empezar_giro_90(direccion=-1)  # Giro izquierda
            else:
                self.giro_completado = False
                self.current_step = 4
            
        elif self.current_step == 4:
            if not self.movimiento_completado:
                self.get_logger().info('Paso 5: Segundo palo (12s)')
                self.mover_adelante(12.0)
            else:
                self.movimiento_completado = False
                self.current_step = 5
            
        elif self.current_step == 5:
            self.is_drawing = False
            self.parar()
            self.get_logger().info('DIBUJO DEL NUMERO 11 TERMINADO')
            
    def ejecutar_siguiente_paso(self):
        self.get_logger().info(f'Avanzando al paso {self.current_step + 1}')

def main(args=None):
    rclpy.init(args=args)
    node = DrawNumberSimulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupcion por teclado - Deteniendo robot')
        node.parar()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()