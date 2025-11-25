#!/usr/bin/env python3
"""
autonomous_navigation.py
Nodo ROS para navegación autónoma con detección de ArUcos - SIMULACIÓN
Apartado 3.4 del Sprint 3
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Int32
import math
import time

class AutonomousNavigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        
        # Parámetro para seleccionar qué ArUco está presente
        self.declare_parameter('target_aruco_id', 0)  # Por defecto ArUco 0
        self.target_aruco_id = self.get_parameter('target_aruco_id').value
        
        # PARÁMETROS DE VELOCIDAD - AUMENTADOS
        self.declare_parameter('linear_speed', 0.5)   # Aumentado de 0.2 a 0.5
        self.declare_parameter('angular_speed', 0.8)  # Aumentado de 0.3 a 0.8
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        self.get_logger().info(f"Inicializando Navegación Autónoma - ArUco objetivo: {self.target_aruco_id}")
        self.get_logger().info(f"Velocidades - Lineal: {self.linear_speed} m/s, Angular: {self.angular_speed} rad/s")
        
        # Publisher para controlar el robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber para recibir el ArUco detectado
        self.aruco_sub = self.create_subscription(
            Int32, 
            '/detected_aruco', 
            self.aruco_callback, 
            10
        )
        
        # Variables de estado
        self.detected_aruco_id = -1
        self.aruco_detected = False
        self.navigation_complete = False
        
        # Posiciones de los ArUcos en Gazebo (basadas en tus capturas)
        self.aruco_positions = {
            0: (5.70, 3.83, "ArUco 0 - Mesa izquierda"),    # marker0
            1: (5.70, 3.89, "ArUco 1 - Mesa central"),      # marker1  
            2: (5.71, 3.83, "ArUco 2 - Mesa derecha")       # marker2
        }
        
        # Destinos finales basados en ArUcos (x, y, theta, descripción)
        self.aruco_destinations = {
            0: (1.5, 0.5, 0.0, "Mesa de trabajo 1"),       # Si detecta ArUco 0
            1: (2.5, 1.5, 90.0, "Mesa de trabajo 2"),      # Si detecta ArUco 1  
            2: (3.5, 0.8, 180.0, "Área de herramientas")   # Si detecta ArUco 2
        }
        
        # Iniciar secuencia de navegación después de un breve delay
        self.start_time = time.time()
        self.create_timer(3.0, self.execute_initial_navigation)  # Esperar 3 segundos para iniciar
        
    def aruco_callback(self, msg):
        """Callback para procesar el ArUco detectado"""
        if not self.aruco_detected and msg.data >= 0:  # -1 indica no detectado
            self.detected_aruco_id = msg.data
            self.aruco_detected = True
            self.get_logger().info(f"ArUco detectado: ID {self.detected_aruco_id}")
            
            # Verificar si es el ArUco esperado
            if self.detected_aruco_id == self.target_aruco_id:
                self.get_logger().info(f"✅ ArUco objetivo {self.target_aruco_id} detectado correctamente")
            else:
                self.get_logger().warning(f"⚠️  Se detectó ArUco {self.detected_aruco_id} pero se esperaba {self.target_aruco_id}")
            
            # Detener el robot momentáneamente
            self.stop_robot()
            time.sleep(0.5)  # Reducido de 1 a 0.5 segundos
            
            # Ejecutar navegación al destino final basado en el ArUco detectado
            self.execute_final_navigation()
    
    def move_forward(self, distance, speed=None):
        """Mover el robot hacia adelante una distancia específica"""
        if speed is None:
            speed = self.linear_speed
            
        self.get_logger().info(f"Moviendo hacia adelante {distance:.1f} metros a {speed} m/s")
        
        twist = Twist()
        twist.linear.x = speed
        
        # Calcular tiempo necesario para la distancia
        move_time = distance / twist.linear.x
        
        # Publicar comando por el tiempo calculado
        start_time = time.time()
        while (time.time() - start_time) < move_time and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(0.05)  # Reducido de 0.1 a 0.05 para mayor responsividad
        
        self.stop_robot()
        time.sleep(0.3)  # Reducido de 0.5 a 0.3 segundos
    
    def turn(self, angle_degrees, speed=None):
        """Girar el robot un ángulo específico"""
        if speed is None:
            speed = self.angular_speed
            
        direction = "derecha" if angle_degrees > 0 else "izquierda"
        self.get_logger().info(f"Girando {abs(angle_degrees)} grados a la {direction} a {speed} rad/s")
        
        twist = Twist()
        twist.angular.z = speed if angle_degrees > 0 else -speed
        
        # Calcular tiempo necesario para el ángulo
        turn_time = abs(angle_degrees) * math.pi / (180 * abs(twist.angular.z))
        
        # Publicar comando por el tiempo calculado
        start_time = time.time()
        while (time.time() - start_time) < turn_time and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(0.05)  # Reducido de 0.1 a 0.05 para mayor responsividad
        
        self.stop_robot()
        time.sleep(0.3)  # Reducido de 0.5 a 0.3 segundos
    
    def stop_robot(self):
        """Detener completamente el robot"""
        twist = Twist()
        self.cmd_pub.publish(twist)
    
    def navigate_to_position(self, target_x, target_y):
        """Navegar a una posición específica (implementación simplificada para simulación)"""
        self.get_logger().info(f"Navegando a posición: ({target_x:.2f}, {target_y:.2f})")
        
        # Calcular distancia y ángulo aproximados desde la posición actual (asumiendo inicio en 0,0)
        distance = math.sqrt(target_x**2 + target_y**2)
        angle = math.degrees(math.atan2(target_y, target_x))
        
        # Primero girar hacia la dirección
        if abs(angle) > 10:  # Solo girar si el ángulo es significativo
            self.turn(angle)
        
        # Avanzar la distancia calculada
        self.move_forward(distance)
        
        self.get_logger().info("Posición objetivo alcanzada")
    
    def execute_initial_navigation(self):
        """Ejecutar la navegación inicial hasta la posición de los ArUcos"""
        if self.navigation_complete:
            return
            
        self.get_logger().info("=== FASE 1: Navegación desde pasillo hasta los ArUcos ===")
        
        try:
            # Obtener la posición del ArUco objetivo
            aruco_x, aruco_y, aruco_desc = self.aruco_positions[self.target_aruco_id]
            
            self.get_logger().info(f"Objetivo: Navegar a {aruco_desc} en ({aruco_x:.2f}, {aruco_y:.2f})")
            
            # Secuencia de navegación inicial al ArUco - DISTANCIAS OPTIMIZADAS
            self.get_logger().info("1. Avance por el pasillo central")
            self.move_forward(29.4)  # Reducida de 2.0 a 1.8
            
            self.get_logger().info("2. Giro hacia el laboratorio")
            self.turn(160)  # Ajustado de 60 a 55
            
            self.get_logger().info("3. Aproximación a la zona de ArUcos")
            self.move_forward(3.5)  # Reducida de 3.0 a 2.5
            
            #self.get_logger().info("4. Navegación precisa al ArUco objetivo")
            # Ajustar la posición final para estar más cerca del ArUco
            #self.navigate_to_position(aruco_x - 0.5, aruco_y - 0.5)  # Reducido de 1.0 a 0.5
            
            self.get_logger().info("5. Posicionamiento para detección")
            self.turn(-25)  # Reducido de -30 a -25
            
            self.get_logger().info(f"=== ESPERANDO DETECCIÓN DE ARUCO {self.target_aruco_id} ===")
            # Esperar detección del ArUco específico
            self.wait_for_aruco_detection()
            
        except Exception as e:
            self.get_logger().error(f"Error en navegación inicial: {e}")
    
    def wait_for_aruco_detection(self):
        """Esperar hasta que se detecte el ArUco objetivo"""
        timeout = 15  # Reducido de 25 a 15 segundos
        start_time = time.time()
        
        self.get_logger().info(f"Buscando ArUco {self.target_aruco_id}...")
        
        while not self.aruco_detected and (time.time() - start_time) < timeout and rclpy.ok():
            # Publicar mensaje cada 2 segundos (reducido de 3)
            if int(time.time() - start_time) % 2 == 0:
                remaining = timeout - (time.time() - start_time)
                self.get_logger().info(f"Escaneando... Tiempo restante: {remaining:.0f}s")
            time.sleep(0.3)  # Reducido de 0.5 a 0.3
        
        if not self.aruco_detected:
            self.get_logger().warning(f"Timeout: No se detectó el ArUco {self.target_aruco_id}. Continuando con destino por defecto.")
            self.detected_aruco_id = self.target_aruco_id  # Usar el objetivo como detectado
            self.aruco_detected = True
            self.execute_final_navigation()
    
    def execute_final_navigation(self):
        """Ejecutar navegación al destino final basado en el ArUco detectado"""
        # Usar el ArUco detectado, o si no se detectó, usar el objetivo
        final_aruco_id = self.detected_aruco_id if self.detected_aruco_id != -1 else self.target_aruco_id
        
        if final_aruco_id not in self.aruco_destinations:
            self.get_logger().warning(f"ArUco ID {final_aruco_id} no reconocido. Usando destino por defecto.")
            final_aruco_id = 0
        
        # Obtener destino del ArUco
        target_x, target_y, target_theta, description = self.aruco_destinations[final_aruco_id]
        
        self.get_logger().info(f"=== FASE 2: Navegación a destino final ===")
        self.get_logger().info(f"ArUco {final_aruco_id} -> Destino: {description}")
        
        try:
            # Navegación al destino final dentro del laboratorio
            self.get_logger().info("1. Navegación a la posición objetivo")
            self.navigate_to_position(target_x, target_y)
            
            self.get_logger().info("2. Ajuste final de orientación")
            if target_theta != 0:
                self.turn(target_theta)
            
            self.get_logger().info("3. Posicionamiento final")
            self.stop_robot()
            
            self.navigation_complete = True
            self.get_logger().info("=== NAVEGACIÓN AUTÓNOMA COMPLETADA ===")
            self.get_logger().info(f"Robot posicionado exitosamente en: {description}")
            
        except Exception as e:
            self.get_logger().error(f"Error en navegación final: {e}")
        finally:
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    nav_node = AutonomousNavigation()
    
    try:
        # Mantener el nodo activo hasta completar la navegación
        while rclpy.ok() and not nav_node.navigation_complete:
            rclpy.spin_once(nav_node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.stop_robot()
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()