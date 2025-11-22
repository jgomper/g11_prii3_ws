#!/usr/bin/env python3
"""
predefined_navigation.py
Nodo ROS para navegacion predefinida - INCLUYE Nav2
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
import os
import time

class PredefinedNavigation(Node):
    def __init__(self):
        super().__init__('predefined_navigation')
        
        self.get_logger().info("Inicializando Navegacion Predefinida F1L3...")
        
        # Inicializar navegador CON Nav2
        self.navigator = BasicNavigator()
        
        # Cargar waypoints
        self.waypoints = self.load_waypoints()
        
        # Esperar a que Nav2 este activo
        self.get_logger().info("Esperando a que Nav2 este activo...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 activo - Iniciando navegacion predefinida")
        
    def load_waypoints(self):
        """Cargar waypoints desde archivo YAML"""
        waypoints_file = os.path.join(
            os.path.dirname(__file__),
            'predefined_waypoints.yaml'
        )
        
        try:
            with open(waypoints_file, 'r') as file:
                data = yaml.safe_load(file)
                waypoints = data['predefined_waypoints']
                self.get_logger().info(f"Waypoints cargados: {list(waypoints.keys())}")
                return waypoints
        except Exception as e:
            self.get_logger().error(f"Error cargando waypoints: {e}")
            return {}
    
    def navigate_to_waypoint(self, waypoint_name):
        """Navegar a un waypoint especifico"""
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f"Waypoint '{waypoint_name}' no encontrado")
            return False
        
        pose_data = self.waypoints[waypoint_name]
        
        # Crear goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose_data['x']
        goal_pose.pose.position.y = pose_data['y']
        goal_pose.pose.orientation.z = pose_data['z']
        goal_pose.pose.orientation.w = pose_data['w']
        
        self.get_logger().info(f"Navegando a: {waypoint_name}")
        self.navigator.goToPose(goal_pose)
        
        # Esperar hasta completar
        while not self.navigator.isTaskComplete():
            pass
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Llegado a {waypoint_name}!")
            return True
        else:
            self.get_logger().error(f"Fallo navegando a {waypoint_name}")
            return False
    
    def run_demo(self):
        """Ejecutar secuencia de navegacion predefinida"""
        self.get_logger().info("Iniciando secuencia de navegacion predefinida...")
        
        waypoint_sequence = ['pasillo_central', 'puerta_laboratorio', 'interior_laboratorio']
        
        for waypoint in waypoint_sequence:
            if not self.navigate_to_waypoint(waypoint):
                break
            time.sleep(2)
        
        self.get_logger().info("Secuencia de navegacion completada")

def main(args=None):
    rclpy.init(args=args)
    nav_node = PredefinedNavigation()
    nav_node.run_demo()
    rclpy.shutdown()

if __name__ == '__main__':
    main()