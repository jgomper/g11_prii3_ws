#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import math
import time
import yaml
import os

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Cargar waypoints desde archivo YAML
        self.waypoints = self.load_waypoints_from_yaml()
        
        self.current_waypoint = 0
        self.navigation_client = None
        self.initial_pose_set = False
        
        # Crear publisher para la pose inicial
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Crear cliente de acción para navegación
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("Iniciando navegacion a x=4 metros")
        self.get_logger().info("Esperando servidor de navegacion...")
        
        # Timer para verificar conexión
        self.connection_timer = self.create_timer(1.0, self.check_navigation_server)
    
    def check_navigation_server(self):
        """Verificar si el servidor de navegación está disponible"""
        if self.navigation_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Servicio de navegacion disponible")
            self.connection_timer.cancel()
            # Configurar pose inicial
            self.create_timer(2.0, self.set_initial_pose)
    
    def load_waypoints_from_yaml(self):
        """Cargar waypoints desde archivo YAML"""
        waypoints_file = os.path.join(
            os.path.dirname(__file__),
            'waypoints.yaml'
        )
        
        try:
            with open(waypoints_file, 'r') as file:
                waypoints_data = yaml.safe_load(file)
                waypoints = []
                
                for wp_name, wp_data in waypoints_data['waypoints'].items():
                    position = wp_data['position']
                    waypoints.append({
                        'name': wp_name,
                        'position': position,
                        'wait_time': wp_data.get('wait_time', 2.0)
                    })
                
                self.get_logger().info(f"Cargados {len(waypoints)} waypoints")
                for wp in waypoints:
                    self.get_logger().info(f" - {wp['name']}: ({wp['position'][0]}, {wp['position'][1]}, {wp['position'][2]})")
                return waypoints
                
        except FileNotFoundError:
            self.get_logger().error("Archivo waypoints.yaml no encontrado")
            # Waypoints por defecto - avance lineal a x=4
            return [
                {'name': 'start', 'position': [0.0, 0.0, 0.0], 'wait_time': 3.0},
                {'name': 'advance_4m', 'position': [4.0, 0.0, 0.0], 'wait_time': 10.0},
            ]
        except Exception as e:
            self.get_logger().error(f"Error cargando waypoints: {e}")
            return [
                {'name': 'start', 'position': [0.0, 0.0, 0.0], 'wait_time': 3.0},
                {'name': 'advance_4m', 'position': [4.0, 0.0, 0.0], 'wait_time': 10.0},
            ]
    
    def set_initial_pose(self):
        """Configurar la pose inicial del robot en el mapa"""
        if self.initial_pose_set:
            return
            
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Pose inicial en el centro del mapa
        initial_pose.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        initial_pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Covarianza para localización inicial
        covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        initial_pose.pose.covariance = covariance
        
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info("Pose inicial publicada en (0.0, 0.0, 0.0)")
        self.initial_pose_set = True
        
        # Iniciar navegación después de configurar la pose inicial
        self.create_timer(3.0, self.start_navigation)
    
    def start_navigation(self):
        """Iniciar la secuencia de waypoints"""
        if self.current_waypoint < len(self.waypoints):
            self.navigate_to_waypoint(self.waypoints[self.current_waypoint])
        else:
            self.get_logger().info("No hay waypoints para navegar")
    
    def navigate_to_waypoint(self, waypoint):
        """Navegar a un waypoint específico"""
        position = waypoint['position']
        x, y, theta = position
        
        # Crear mensaje de goal
        goal_msg = NavigateToPose.Goal()
        
        # Configurar el frame_id
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Configurar la posición
        goal_msg.pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        
        # Convertir ángulo a quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, theta)
        goal_msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        self.get_logger().info(f'Navegando al waypoint {waypoint["name"]}: ({x:.2f}, {y:.2f}, {theta:.2f})')
        
        # Enviar goal
        future = self.navigation_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback cuando se recibe respuesta del goal"""
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().warn(f'Goal rechazado para waypoint {self.waypoints[self.current_waypoint]["name"]}')
                self.create_timer(2.0, self.move_to_next_waypoint)
                return
            
            self.get_logger().info(f'Goal aceptado para {self.waypoints[self.current_waypoint]["name"]}, navegando...')
            
            # Obtener resultado
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error en goal response: {e}')
            self.create_timer(2.0, self.move_to_next_waypoint)
    
    def get_result_callback(self, future):
        """Callback cuando se completa la navegación"""
        try:
            result = future.result()
            status = result.status
            
            if self.current_waypoint < len(self.waypoints):
                current_waypoint_name = self.waypoints[self.current_waypoint]["name"]
                
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f'Waypoint {current_waypoint_name} alcanzado con exito')
                else:
                    self.get_logger().warn(f'Fallo la navegacion al waypoint {current_waypoint_name} (estado: {status})')
                
                # Esperar antes del siguiente waypoint
                wait_time = self.waypoints[self.current_waypoint]["wait_time"]
                self.create_timer(wait_time, self.move_to_next_waypoint)
                
        except Exception as e:
            self.get_logger().error(f'Error en callback de resultado: {e}')
            self.create_timer(2.0, self.move_to_next_waypoint)
    
    def move_to_next_waypoint(self):
        """Mover al siguiente waypoint en la lista"""
        self.current_waypoint += 1
        
        if self.current_waypoint < len(self.waypoints):
            self.navigate_to_waypoint(self.waypoints[self.current_waypoint])
        else:
            self.get_logger().info('Todos los waypoints completados')
            # Opcional: reiniciar la secuencia después de 10 segundos
            self.create_timer(10.0, self.restart_navigation)
    
    def restart_navigation(self):
        """Reiniciar la secuencia de waypoints"""
        self.current_waypoint = 0
        self.get_logger().info('Reiniciando secuencia de waypoints...')
        self.start_navigation()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convertir ángulos de Euler a quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    
    waypoint_navigator = WaypointNavigator()
    
    try:
        rclpy.spin(waypoint_navigator)
    except KeyboardInterrupt:
        waypoint_navigator.get_logger().info('Navegacion por waypoints interrumpida por el usuario')
    finally:
        waypoint_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()