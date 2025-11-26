#!/usr/bin/env python3
"""
autonomous_navigation.py
Nodo ROS unificado para navegación autónoma con detección de ArUcos - VERSIÓN MEJORADA
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class AutonomousNavigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        
        # Parámetros de configuración
        self.declare_parameter('target_aruco_id', 5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.4)
        
        self.target_aruco_id = self.get_parameter('target_aruco_id').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Publisher para controlar el robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Configuración de ArUco
        self.setup_aruco_detector()
        
        # Suscriptor a la imagen de la cámara
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Variables de estado
        self.detected_aruco_id = -1
        self.aruco_detected = False
        self.navigation_complete = False
        self.current_pose = [0.0, 0.0, 0.0]
        self.window_initialized = False
        self.last_image_time = 0
        self.image_display_interval = 0.033
        
        # Waypoints para navegación por ejes (coordenadas absolutas)
        self.waypoints_phase1 = [
            (25, 0.0, 0.0, "Avance por pasillo"),
            (25, 0.0, math.radians(42), "Giro hacia laboratorio"),
            (25, 2.5, 0.0, "Aproximación")
        ]
        
        # Destinos finales por ArUco - AHORA PUEDES PONER HASTA 100 WAYPOINTS
        self.aruco_destinations = {
            5: [
                (25, 0.0, math.radians(-10), "Giro pevio para entrar a clase"),
                (25.03, 0.0, math.radians(10), "Posicion para entrar a clase"),
                (25.03, 0.0, math.radians(10), "Entrar a la puerta"),
                (25.03, 5.0, math.radians(0), "Dentro de clase"),
                #(24, 7.5, math.radians(5), "Posición universal"),
            ],
            
            6: [
                (2.5, 3.0, math.radians(135), "Mesa 2"),
                (3.0, 3.5, math.radians(135), "Posición final Aruco 6")
            ],
            
            17: [
                (3.5, 1.5, math.radians(45), "Área herramientas"),
                (4.0, 1.0, math.radians(45), "Posición final Aruco 17")
            ]
        }
        
        self.get_logger().info("🚀 Navegación Autónoma Inicializada")
        self.get_logger().info(f"Buscando ArUcos 5x5: 5, 6, 17")
        
        # Timer para verificar periodicamente si hay que cerrar ventanas
        self.create_timer(0.1, self.check_window_status)
        
        # Iniciar secuencia después de delay
        self.create_timer(5.0, self.start_navigation)
        
    def setup_aruco_detector(self):
        """Configurar el detector de ArUcos"""
        try:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
            self.parameters = aruco.DetectorParameters_create()
            
            self.parameters.adaptiveThreshWinSizeMin = 3
            self.parameters.adaptiveThreshWinSizeMax = 23
            self.parameters.adaptiveThreshConstant = 10
            self.parameters.minMarkerPerimeterRate = 0.02
            self.parameters.maxMarkerPerimeterRate = 4.0
            self.parameters.polygonalApproxAccuracyRate = 0.05
            self.parameters.minCornerDistanceRate = 0.05
            self.parameters.minDistanceToBorder = 3
            self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            
            self.valid_aruco_ids = [5, 6, 17]
            self.last_detected_id = -1
            self.consecutive_detections = 0
            self.detection_threshold = 2
            
            self.get_logger().info("✅ Detector ArUco configurado con DICT_5X5_50")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error configurando detector ArUco: {e}")
    
    def check_window_status(self):
        """Verificar periódicamente el estado de la ventana"""
        try:
            if not self.aruco_detected and not self.navigation_complete:
                if self.window_initialized:
                    cv2.waitKey(1)
        except:
            self.window_initialized = False
        
    def image_callback(self, msg):
        """Procesar imagen y detectar ArUcos"""
        if self.aruco_detected or self.navigation_complete:
            return
            
        current_time = time.time()
        if current_time - self.last_image_time < self.image_display_interval:
            return
            
        self.last_image_time = current_time
        
        try:
            if msg.encoding != 'rgb8':
                return
            
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
            gray = cv2.medianBlur(gray, 5)
            
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            current_detection = -1
            display_image = cv_image.copy()
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in self.valid_aruco_ids:
                        current_detection = marker_id
                        
                        aruco.drawDetectedMarkers(display_image, corners, ids)
                        
                        corner_array = corners[i][0]
                        center = np.mean(corner_array, axis=0).astype(int)
                        
                        cv2.polylines(display_image, [corner_array.astype(int)], True, (0, 255, 0), 2)
                        
                        cv2.putText(
                            display_image, 
                            f"ID: {marker_id} (5x5)", 
                            (center[0] - 50, center[1] - 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            0.7,
                            (0, 255, 0), 
                            2
                        )
                        
                        area = cv2.contourArea(corner_array)
                        cv2.putText(
                            display_image, 
                            f"Area: {area:.0f}", 
                            (center[0] - 50, center[1] + 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5,
                            (255, 255, 0), 
                            1
                        )
                        
                        self.get_logger().info(f'🔍 ArUco 5x5 ID {marker_id} detectado', 
                                              throttle_duration_sec=2.0)
                        break
            
            cv2.putText(
                display_image, 
                "Camara Robot - Buscando ArUcos 5x5 (5,6,17)", 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.7,
                (255, 255, 255), 
                2
            )
            
            cv2.putText(
                display_image, 
                f"Robot en: ({self.current_pose[0]:.1f}, {self.current_pose[1]:.1f})", 
                (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.6,
                (255, 255, 0), 
                1
            )
            
            if ids is None:
                cv2.putText(
                    display_image, 
                    "No se detectan ArUcos - Moviendo camara...", 
                    (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.6,
                    (0, 0, 255), 
                    2
                )
            
            try:
                if not self.window_initialized:
                    cv2.namedWindow('Camara Robot - Deteccion ArUcos 5x5', cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('Camara Robot - Deteccion ArUcos 5x5', 800, 600)
                    self.window_initialized = True
                    self.get_logger().info("📷 Ventana de camara inicializada")
                
                cv2.imshow('Camara Robot - Deteccion ArUcos 5x5', display_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    self.get_logger().info("👋 Ventana cerrada por usuario")
                    cv2.destroyAllWindows()
                    self.window_initialized = False
                    
            except Exception as e:
                self.get_logger().warn(f"⚠️  Error mostrando ventana: {e}")
                self.window_initialized = False
            
            if current_detection != -1:
                if current_detection == self.last_detected_id:
                    self.consecutive_detections += 1
                else:
                    self.consecutive_detections = 1
                    self.last_detected_id = current_detection
                
                if self.consecutive_detections >= self.detection_threshold:
                    self.detected_aruco_id = current_detection
                    self.aruco_detected = True
                    self.get_logger().info(f"🎯 ARUCO 5x5 ID {self.detected_aruco_id} CONFIRMADO!")
                    
                    self.stop_robot()
                    time.sleep(1)
                    
                    try:
                        cv2.destroyWindow('Camara Robot - Deteccion ArUcos 5x5')
                        self.window_initialized = False
                    except:
                        pass
                    
                    self.execute_final_navigation()
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error en procesamiento de imagen: {str(e)}')
    
    def start_navigation(self):
        """Iniciar la secuencia de navegación"""
        if self.navigation_complete:
            return
            
        self.get_logger().info("=== INICIANDO NAVEGACIÓN AUTÓNOMA ===")
        self.execute_phase1_navigation()
    
    def execute_phase1_navigation(self):
        """Ejecutar navegación inicial hasta zona de ArUcos"""
        self.get_logger().info("--- FASE 1: Navegación al laboratorio ---")
        
        try:
            # ✅ CORREGIDO: Maneja cualquier número de waypoints
            for waypoint in self.waypoints_phase1:
                # Extraer los valores del waypoint de forma segura
                if len(waypoint) >= 4:
                    target_x, target_y, target_theta, description = waypoint[0], waypoint[1], waypoint[2], waypoint[3]
                else:
                    self.get_logger().error(f"Waypoint inválido: {waypoint}")
                    continue
                
                self.get_logger().info(f"Waypoint: {description}")
                
                success = self.navigate_to_waypoint(target_x, target_y, target_theta)
                
                if not success:
                    self.get_logger().error(f"Error en waypoint: {description}")
                    return
                
                self.current_pose = [target_x, target_y, target_theta]
                time.sleep(0.5)
            
            self.get_logger().info("--- FASE 1 COMPLETADA - Buscando ArUcos ---")
            self.start_aruco_scanning()
            
        except Exception as e:
            self.get_logger().error(f"Error en Fase 1: {e}")
    
    def navigate_to_waypoint(self, target_x, target_y, target_theta):
        """Navegar a un waypoint específico"""
        current_x, current_y, current_theta = self.current_pose
        
        dx = target_x - current_x
        dy = target_y - current_y
        dtheta = target_theta - current_theta
        
        self.get_logger().info(f"Navegando: ΔX={dx:.2f}, ΔY={dy:.2f}, Δθ={math.degrees(dtheta):.1f}°")
        
        try:
            if abs(dtheta) > 0.1:
                self.rotate_to_angle(target_theta)
            
            if abs(dx) > 0.1:
                self.move_along_axis('x', dx)
            
            if abs(dy) > 0.1:
                if dy > 0:
                    self.rotate_to_angle(current_theta + math.radians(90))
                else:
                    self.rotate_to_angle(current_theta - math.radians(90))
                
                self.move_along_axis('x', abs(dy))
                self.rotate_to_angle(target_theta)
            
            if abs(dtheta) > 0.1:
                self.rotate_to_angle(target_theta)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error en navegación: {e}")
            return False
    
    def move_along_axis(self, axis, distance):
        """Mover a lo largo de un eje específico"""
        if abs(distance) < 0.1:
            return
            
        direction = 1 if distance > 0 else -1
        abs_distance = abs(distance)
        
        self.get_logger().info(f"Movimiento en {axis.upper()}: {abs_distance:.2f}m")
        
        twist = Twist()
        twist.linear.x = self.linear_speed * direction
        
        move_time = abs_distance / self.linear_speed
        
        start_time = time.time()
        while (time.time() - start_time) < move_time and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.3)
    
    def rotate_to_angle(self, target_angle):
        """Rotar a un ángulo específico"""
        current_angle = self.current_pose[2]
        angle_diff = target_angle - current_angle
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) < 0.05:
            return
            
        degrees_diff = math.degrees(angle_diff)
        self.get_logger().info(f"Rotación: {degrees_diff:.1f}°")
        
        twist = Twist()
        twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        
        rotate_time = abs(angle_diff) / self.angular_speed
        
        start_time = time.time()
        while (time.time() - start_time) < rotate_time and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.3)
        self.current_pose[2] = target_angle
    
    def start_aruco_scanning(self):
        """Iniciar escaneo de ArUcos"""
        self.get_logger().info("🔍 ESCANEANDO ARUCOS 5x5...")
        self.get_logger().info("Robot girando - Coloca un ArUco 5x5 (ID 5,6,17) frente a la cámara")
        
        twist = Twist()
        twist.angular.z = 0.2
        
        start_time = time.time()
        scan_duration = 12
        
        while (time.time() - start_time) < scan_duration and not self.aruco_detected and rclpy.ok():
            self.cmd_pub.publish(twist)
            
            if int(time.time() - start_time) % 5 == 0:
                remaining = scan_duration - (time.time() - start_time)
                self.get_logger().info(f"Escaneando... {remaining:.0f}s restantes")
            
            time.sleep(0.1)
        
        self.stop_robot()
        
        if not self.aruco_detected:
            self.get_logger().warning("⏰ Timeout: No se detectó ArUco. Usando ID 5 por defecto")
            self.detected_aruco_id = 5
            self.aruco_detected = True
            self.execute_final_navigation()
    
    def execute_final_navigation(self):
        """✅ CORREGIDO: Ejecutar navegación final con cualquier número de waypoints"""
        if self.detected_aruco_id not in self.aruco_destinations:
            self.detected_aruco_id = 5
            
        waypoints = self.aruco_destinations[self.detected_aruco_id]
        
        self.get_logger().info(f"--- FASE 2: Navegación con ArUco ID {self.detected_aruco_id} ---")
        self.get_logger().info(f"Ejecutando {len(waypoints)} waypoints para este ArUco")
        
        try:
            # ✅ CORREGIDO: Maneja cualquier número de waypoints de forma segura
            for i, waypoint in enumerate(waypoints):
                # Verificar que el waypoint tenga al menos 4 elementos
                if len(waypoint) < 4:
                    self.get_logger().error(f"Waypoint {i} inválido: {waypoint}")
                    continue
                
                # Extraer valores de forma segura
                target_x, target_y, target_theta, description = waypoint[0], waypoint[1], waypoint[2], waypoint[3]
                
                self.get_logger().info(f"Destino {i+1}/{len(waypoints)}: {description}")
                self.get_logger().info(f"Coordenadas: ({target_x:.1f}, {target_y:.1f}, {math.degrees(target_theta):.1f}°)")
                
                success = self.navigate_to_waypoint(target_x, target_y, target_theta)
                
                if not success:
                    self.get_logger().error(f"Error en destino {i+1}")
                    break
                
                self.current_pose = [target_x, target_y, target_theta]
                time.sleep(0.5)
            
            self.navigation_complete = True
            self.get_logger().info("🎉 NAVEGACIÓN COMPLETADA!")
            self.get_logger().info(f"Se ejecutaron {len(waypoints)} waypoints para ArUco ID {self.detected_aruco_id}")
            
        except Exception as e:
            self.get_logger().error(f"Error en Fase 2: {e}")
        finally:
            self.stop_robot()
            try:
                cv2.destroyAllWindows()
                self.window_initialized = False
            except:
                pass
    
    def stop_robot(self):
        """Detener el robot"""
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigation()
    
    try:
        while rclpy.ok() and not node.navigation_complete:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()