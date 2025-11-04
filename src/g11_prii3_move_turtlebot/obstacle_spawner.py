#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_srvs.srv import SetBool
import time
import threading

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        
        # Clientes para servicios
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.drawing_client = self.create_client(SetBool, 'control_drawing')
        
        # Variables de control
        self.obstacle_spawned = False
        self.demo_started = False
        
        # Esperar a que los servicios estén disponibles
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /spawn_entity...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /delete_entity...')
        while not self.drawing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio control_drawing...')
            
        self.get_logger().info('Obstacle Spawner node started - Grupo 11')
        
        # Iniciar demo automáticamente después de 15 segundos
        self.timer = self.create_timer(15.0, self.run_demo)
        
    def create_cube_sdf(self):
        """Crea el contenido SDF para un cubo rojo"""
        cube_sdf = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="obstacle_cube">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient>
          <diffuse>1.0 0.0 0.0 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        return cube_sdf
        
    def spawn_obstacle(self):
        """Coloca el obstáculo en las coordenadas especificadas"""
        try:
            # Coordenadas del obstáculo
            obstacle_x = 2.0
            obstacle_y = -0.5
            obstacle_z = 0.2
            
            request = SpawnEntity.Request()
            request.name = 'obstacle_cube'
            request.xml = self.create_cube_sdf()
            request.initial_pose.position.x = obstacle_x
            request.initial_pose.position.y = obstacle_y
            request.initial_pose.position.z = obstacle_z
            request.reference_frame = 'world'
            
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info(f'BLOQUE COLOCADO en ({obstacle_x}, {obstacle_y}, {obstacle_z})')
                self.obstacle_spawned = True
                return True
            else:
                self.get_logger().error('Error al colocar bloque')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False
            
    def delete_obstacle(self):
        """Elimina el obstáculo"""
        try:
            request = DeleteEntity.Request()
            request.name = 'obstacle_cube'
            
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info('BLOQUE ELIMINADO')
                self.obstacle_spawned = False
                return True
            else:
                self.get_logger().error('Error al eliminar bloque')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False
    
    def start_drawing(self):
        """Inicia el dibujo del número 11"""
        request = SetBool.Request()
        request.data = True
        
        future = self.drawing_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Dibujo: {future.result().message}')
            return True
        else:
            self.get_logger().error('Error al iniciar dibujo')
            return False
            
    def run_demo(self):
        """Ejecuta la secuencia completa automáticamente"""
        if self.demo_started:
            return
            
        self.demo_started = True
        self.timer.cancel()
        
        self.get_logger().info('=== INICIANDO DEMO AUTOMÁTICA ===')
        
        # Paso 1: Colocar bloque
        self.get_logger().info('Colocando bloque de obstáculo...')
        if self.spawn_obstacle():
            self.get_logger().info('Robot avanzando en línea recta hasta encontrar el obstáculo...')
        
        # Paso 2: Esperar 10 segundos y luego iniciar dibujo
        threading.Timer(10.0, self.start_drawing_sequence).start()
    
    def start_drawing_sequence(self):
        """Inicia la secuencia de dibujo después de la detección"""
        self.get_logger().info('Iniciando dibujo del número 11...')
        self.start_drawing()
        
        # Paso 3: Eliminar bloque después de 25 segundos
        threading.Timer(25.0, self.cleanup).start()
    
    def cleanup(self):
        """Limpieza al final de la demo"""
        if self.obstacle_spawned:
            self.get_logger().info('Eliminando bloque...')
            self.delete_obstacle()
        
        self.get_logger().info('Demo completada')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.obstacle_spawned:
            node.delete_obstacle()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()