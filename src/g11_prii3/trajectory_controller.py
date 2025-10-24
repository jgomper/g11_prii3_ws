#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import time
import math

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # Publicador para comandos de movimiento
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptor al LIDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Clientes para servicios de Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Variables de control
        self.phase = 0  # 0: Avanzar hasta objeto, 1: Esperar 5s, 2: Avanzar hasta x=5, 3: Girar -90°, 4: Avanzar hasta y=-1.5, 5: Girar -90°, 6: Avanzar hasta x=0
        self.obstacle_detected = False
        self.obstacle_stop_time = None
        self.obstacle_spawned = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_time = time.time()
        
        # Parámetros
        self.linear_speed = 0.15
        self.angular_speed = 0.8
        
        # Timer principal
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Esperar servicios
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /spawn_entity...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /delete_entity...')
            
        self.get_logger().info('Trajectory Controller iniciado - Grupo 11')
        
        # Iniciar secuencia después de 3 segundos
        self.create_timer(3.0, self.start_sequence)
        
    def start_sequence(self):
        """Inicia la secuencia completa"""
        self.get_logger().info('INICIANDO SECUENCIA COMPLETA')
        self.spawn_obstacle()
        
    def spawn_obstacle(self):
        """Coloca el obstáculo en la posición ORIGINAL"""
        try:
            # POSICIÓN ORIGINAL - (2.0, -0.5)
            obstacle_x = 2.0
            obstacle_y = -0.5
            obstacle_z = 0.2
            
            cube_sdf = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="obstacle_cube">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.8 0.8 0.8</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.8 0.8 0.8</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
            
            request = SpawnEntity.Request()
            request.name = 'obstacle_cube'
            request.xml = cube_sdf
            request.initial_pose.position.x = obstacle_x
            request.initial_pose.position.y = obstacle_y
            request.initial_pose.position.z = obstacle_z
            request.reference_frame = 'world'
            
            future = self.spawn_client.call_async(request)
            future.add_done_callback(self.obstacle_spawned_callback)
                
        except Exception as e:
            self.get_logger().error(f'Error al colocar obstaculo: {e}')
            
    def obstacle_spawned_callback(self, future):
        """Callback cuando el obstáculo es colocado"""
        try:
            result = future.result()
            self.obstacle_spawned = True
            self.get_logger().info('Obstaculo colocado en (2.0, -0.5)')
            self.phase = 0  # Comenzar a avanzar
        except:
            self.get_logger().error('Error al colocar obstaculo')
            
    def delete_obstacle(self):
        """Elimina el obstáculo"""
        try:
            request = DeleteEntity.Request()
            request.name = 'obstacle_cube'
            
            future = self.delete_client.call_async(request)
            future.add_done_callback(self.obstacle_deleted_callback)
                
        except Exception as e:
            self.get_logger().error(f'Error al eliminar obstaculo: {e}')
            
    def obstacle_deleted_callback(self, future):
        """Callback cuando el obstáculo es eliminado"""
        try:
            result = future.result()
            if result.success:
                self.obstacle_spawned = False
                self.get_logger().info('Obstaculo eliminado')
            else:
                self.get_logger().error('Error al eliminar obstaculo')
        except:
            self.get_logger().error('Error en callback de eliminacion')
            
    def lidar_callback(self, msg):
        """Detección de obstáculos - REVISIÓN COMPLETA 360°"""
        if self.phase != 0:  # Solo en fase 0
            return
            
        # Revisar TODOS los ángulos del LIDAR
        min_distance = float('inf')
        for i in range(len(msg.ranges)):
            distance = msg.ranges[i]
            # Cualquier lectura válida
            if 0.1 < distance < 10.0:
                if distance < min_distance:
                    min_distance = distance
        
        # DEBUG: Mostrar distancia mínima de TODO el LIDAR
        current_time = time.time()
        if hasattr(self, 'last_debug_time'):
            if current_time - self.last_debug_time > 1.0:
                self.get_logger().info(f'Distancia MINIMA (360°): {min_distance:.2f}m')
                self.last_debug_time = current_time
        else:
            self.last_debug_time = current_time
        
        # Detectar obstáculo a 1.5 metros (para el objeto en 2.0, -0.5)
        if min_distance < 1.5:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.obstacle_stop_time = time.time()
                self.get_logger().warn(f'OBSTACULO DETECTADO a {min_distance:.2f}m - PARANDO')
                self.phase = 1  # Pasar a fase de espera
            
    def update_pose_estimation(self, linear_vel, angular_vel):
        """Estimación simple de posición"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if linear_vel > 0 and angular_vel == 0:  # Movimiento recto
            self.current_x += linear_vel * dt
            
    def control_loop(self):
        """Bucle principal de control"""
        twist = Twist()
        
        if self.phase == 0:
            # FASE 0: Avanzar hasta detectar objeto
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.get_logger().info('Avanzando hasta objeto... Posicion X: %.1f' % self.current_x, throttle_duration_sec=2)
            
        elif self.phase == 1:
            # FASE 1: Esperar 5 segundos
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            wait_time = time.time() - self.obstacle_stop_time
            self.get_logger().info('Esperando 5 segundos... %.1f/5.0' % wait_time, throttle_duration_sec=1)
            
            if wait_time >= 5.0:
                self.get_logger().info('Pasados 5 segundos - Eliminando obstaculo')
                self.delete_obstacle()
                self.phase = 2
                
        elif self.phase == 2:
            # FASE 2: Avanzar hasta x=5
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
            self.get_logger().info('Avanzando hasta x=5.0... Actual: %.1f' % self.current_x, throttle_duration_sec=2)
            
            if self.current_x >= 5.0:
                self.get_logger().info('Alcanzado x=5.0 - Girando -90°')
                self.phase = 3
                self.turn_start_time = time.time()
                
        elif self.phase == 3:
            # FASE 3: Girar -90°
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed  # Giro derecha
            
            turn_progress = time.time() - self.turn_start_time
            self.get_logger().info('Girando -90°... %.1f/2.0s' % turn_progress, throttle_duration_sec=1)
            
            if turn_progress >= 2.0:  # 90° aprox
                self.get_logger().info('Giro -90° completado - Avanzando hasta y=-1.5')
                self.phase = 4
                self.current_y = 0.0  # Resetear Y para nueva dirección
                
        elif self.phase == 4:
            # FASE 4: Avanzar hasta y=-1.5
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
            # Estimación en Y (simplificada)
            current_time = time.time()
            dt = current_time - self.last_time
            self.current_y -= self.linear_speed * dt  # Negativo porque vamos hacia Y negativo
            self.last_time = current_time
            
            self.get_logger().info('Avanzando hasta y=-1.5... Actual: %.1f' % self.current_y, throttle_duration_sec=2)
            
            if self.current_y <= -1.5:
                self.get_logger().info('Alcanzado y=-1.5 - Girando -90°')
                self.phase = 5
                self.turn_start_time = time.time()
                
        elif self.phase == 5:
            # FASE 5: Girar -90°
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed  # Giro derecha
            
            turn_progress = time.time() - self.turn_start_time
            self.get_logger().info('Girando -90°... %.1f/2.0s' % turn_progress, throttle_duration_sec=1)
            
            if turn_progress >= 2.0:  # 90° aprox
                self.get_logger().info('Giro -90° completado - Avanzando hasta x=0')
                self.phase = 6
                self.current_x = 5.0  # Empezar desde x=5
                
        elif self.phase == 6:
            # FASE 6: Avanzar hasta x=0
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
            self.get_logger().info('Avanzando hasta x=0... Actual: %.1f' % self.current_x, throttle_duration_sec=2)
            
            if self.current_x <= 0.0:
                self.get_logger().info('SECUENCIA COMPLETADA - Robot en (0, -1.5)')
                twist.linear.x = 0.0
                self.phase = 7
                
        elif self.phase == 7:
            # FASE 7: Completado
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        # Publicar comando y actualizar posición
        self.cmd_pub.publish(twist)
        self.update_pose_estimation(twist.linear.x, twist.angular.z)
    
    def destroy_node(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
        if self.obstacle_spawned:
            self.delete_obstacle()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()