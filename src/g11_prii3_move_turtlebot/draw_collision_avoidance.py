#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import math
import time
import os

class DrawCollisionAvoidance(Node):
    def __init__(self):
        super().__init__('draw_collision_avoidance')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Clients para spawn y delete de entidades
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        self.is_drawing = True
        self.current_step = 0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.girando = False
        self.objeto_detectado = False
        self.objeto_procesado = False
        self.objeto_nombre = "objeto_bloque"
        
        # Variables para controlar el estado
        self.movimiento_completado = False
        self.giro_completado = False
        self.giro_iniciado = False
        self.esperando_objeto = False
        
        # Timer para spawnear objeto en el momento correcto
        self.objeto_spawneado = False
        self.timer_spawn = None
        
        self.get_logger().info('Iniciando dibujo del numero 11 en mundo vacio')
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def spawn_objeto(self):
        """Spawnear un objeto bloque delante del robot"""
        if self.objeto_spawneado or self.objeto_procesado:
            return
            
        self.get_logger().info('Spawneando objeto...')
        
        # Esperar maximo 10 segundos por el servicio
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Servicio /spawn_entity no disponible. Continuando sin objeto.')
            return
            
        # Crear request para spawn - objeto en mundo vacio
        request = SpawnEntity.Request()
        request.name = self.objeto_nombre
        request.xml = self.get_bloque_sdf()
        request.initial_pose.position.x = 1.0  # 1 metro delante del robot
        request.initial_pose.position.y = 0.0  # Centrado
        request.initial_pose.position.z = 0.15  # Altura adecuada
        request.initial_pose.orientation.w = 1.0
        request.reference_frame = 'world'
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        self.objeto_spawneado = True
        
    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('OBJETO SPAWNEADO a 1 metro delante')
                self.objeto_detectado = True
            else:
                self.get_logger().error('Error al spawnear objeto')
        except Exception as e:
            self.get_logger().error(f'Excepcion en spawn: {str(e)}')
            
    def delete_objeto(self):
        """Eliminar el objeto spawneado"""
        if not self.delete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio /delete_entity no disponible')
            return
            
        request = DeleteEntity.Request()
        request.name = self.objeto_nombre
        
        future = self.delete_client.call_async(request)
        future.add_done_callback(self.delete_callback)
        
    def delete_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('OBJETO ELIMINADO')
            else:
                self.get_logger().warn('No se pudo eliminar el objeto')
        except Exception as e:
            self.get_logger().error(f'Excepcion al eliminar: {str(e)}')
            
    def get_bloque_sdf(self):
        """Retorna el SDF de un bloque simple para mundo vacio"""
        return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="objeto_bloque">
    <static>false</static>
    <link name="link">
      <pose>0 0 0.15 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.4 0.4 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.4 0.4 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
    def odom_callback(self, msg):
        # Obtener orientacion actual
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        # Calcular yaw
        self.current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
    def mover_adelante(self, segundos):
        """Mover hacia adelante por tiempo determinado"""
        twist = Twist()
        twist.linear.x = 0.2
        
        start = time.time()
        while time.time() - start < segundos and rclpy.ok():
            self.publisher_.publish(twist)
            time.sleep(0.1)
        
        self.parar()
        self.movimiento_completado = True
        
    def empezar_giro_90(self, direccion=-1):
        """Iniciar giro de 90 grados"""
        if not self.giro_iniciado:
            self.target_yaw = self.current_yaw + direccion * math.pi / 2.0  # 90 grados
            self.girando = True
            self.giro_iniciado = True
            self.giro_completado = False
            self.get_logger().info(f'Empezando giro de {self.current_yaw:.2f} a {self.target_yaw:.2f}')
        
    def parar(self):
        """Detener el robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(0.5)
        
    def procesar_objeto(self):
        """Detener robot, esperar 3 segundos y eliminar objeto"""
        if not self.objeto_procesado:
            self.get_logger().info('OBJETO DETECTADO - Robot detenido')
            self.parar()
            
            # Esperar 3 segundos
            self.get_logger().info('Esperando 3 segundos...')
            time.sleep(3)
            
            # Eliminar objeto
            self.delete_objeto()
            self.get_logger().info('OBJETO ELIMINADO - Continuando movimiento')
            self.objeto_detectado = False
            self.objeto_procesado = True
            self.esperando_objeto = False
        
    def control_loop(self):
        if not self.is_drawing:
            return
            
        # Si hay objeto detectado, procesarlo primero
        if self.objeto_detectado and not self.objeto_procesado:
            self.procesar_objeto()
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
                self.giro_iniciado = False
                self.giro_completado = True
                self.parar()
                self.get_logger().info('Giro completado')
                self.ejecutar_siguiente_paso()
            else:
                twist = Twist()
                twist.angular.z = -0.8 if error < 0 else 0.8
                self.publisher_.publish(twist)
            return
            
        # Ejecutar secuencia del numero 11
        if self.current_step == 0:
            if not self.movimiento_completado:
                self.get_logger().info('Paso 1: Primer palo (6s)')
                self.mover_adelante(6.0)
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
            # Spawnear objeto justo antes de empezar el movimiento
            if not self.objeto_spawneado:
                self.spawn_objeto()
                time.sleep(1)  # Esperar un poco para que el objeto aparezca
                
            if not self.movimiento_completado:
                self.get_logger().info('Paso 3: Espacio (1s) - REDUCIDO')
                self.mover_adelante(1.0)  # REDUCIDO de 2s a 1s
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
                self.get_logger().info('Paso 5: Segundo palo (6s)')
                self.mover_adelante(6.0)
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
    node = DrawCollisionAvoidance()
    
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