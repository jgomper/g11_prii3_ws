#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import math
import time

class DrawObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('draw_obstacle_avoidance')
        
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
        self.obstaculo_spawneado = False
        self.obstaculo_procesado = False
        self.obstaculo_nombre = "obstaculo_pequeno"
        self.esquivando = False
        self.esquiva_completada = False
        
        # Variables para controlar el estado
        self.movimiento_completado = False
        self.giro_completado = False
        
        self.get_logger().info('Iniciando dibujo del numero 11 con evasion de obstaculos')
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def spawn_obstaculo(self):
        """Spawnear un obstaculo pequeño delante del robot"""
        if self.obstaculo_spawneado:
            return
            
        self.get_logger().info('Spawneando obstaculo pequeño...')
        
        # Esperar maximo 10 segundos por el servicio
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Servicio /spawn_entity no disponible. Continuando sin obstaculo.')
            return
            
        # Crear request para spawn - obstaculo más cerca y en mejor posición
        request = SpawnEntity.Request()
        request.name = self.obstaculo_nombre
        request.xml = self.get_obstaculo_sdf()
        request.initial_pose.position.x = -1.2
        request.initial_pose.position.y = -0.65
        request.initial_pose.position.z = 0.1
        request.initial_pose.orientation.w = 1.0
        request.reference_frame = 'world'
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        self.obstaculo_spawneado = True
        
    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('OBSTACULO PEQUEÑO SPAWNEADO - Robot debe esquivarlo')
            else:
                self.get_logger().error('Error al spawnear obstaculo')
        except Exception as e:
            self.get_logger().error(f'Excepcion en spawn: {str(e)}')
            
    def delete_obstaculo(self):
        """Eliminar el obstaculo spawneado"""
        if not self.delete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio /delete_entity no disponible')
            return
            
        request = DeleteEntity.Request()
        request.name = self.obstaculo_nombre
        
        future = self.delete_client.call_async(request)
        future.add_done_callback(self.delete_callback)
        
    def delete_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('OBSTACULO ELIMINADO')
            else:
                self.get_logger().warn('No se pudo eliminar el obstaculo')
        except Exception as e:
            self.get_logger().error(f'Excepcion al eliminar: {str(e)}')
            
    def get_obstaculo_sdf(self):
        """Retorna el SDF de un obstaculo muy pequeño"""
        return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="obstaculo_pequeno">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.05 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
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
        
    def mover_adelante(self, segundos, velocidad=0.15):
        """Mover hacia adelante por tiempo determinado"""
        twist = Twist()
        twist.linear.x = velocidad
        
        start = time.time()
        while time.time() - start < segundos and rclpy.ok():
            self.publisher_.publish(twist)
            time.sleep(0.1)
        
        self.parar()
        self.movimiento_completado = True
        
    def empezar_giro_90(self, direccion=-1):
        """Iniciar giro de 90 grados"""
        self.target_yaw = self.current_yaw + direccion * math.pi / 2.0  # 90 grados
        self.girando = True
        self.giro_completado = False
        self.get_logger().info(f'Empezando giro de {self.current_yaw:.2f} a {self.target_yaw:.2f}')
        
    def parar(self):
        """Detener el robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(0.5)
        
    def ejecutar_esquiva(self):
        """Ejecutar maniobra de esquiva del obstaculo"""
        if not self.esquivando and not self.esquiva_completada:
            self.get_logger().info('INICIANDO MANIOBRA DE ESQUIVA')
            self.esquivando = True
            
            # Paso 1: Girar 45 grados a la derecha
            self.target_yaw = self.current_yaw + math.pi / 4.0  # 45 grados derecha
            self.girando = True
            self.esquiva_paso = 1
            
    def control_loop(self):
        if not self.is_drawing:
            return
            
        # Control de giro para esquiva
        if self.girando and self.esquivando:
            error = self.target_yaw - self.current_yaw
            # Normalizar error
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi
                
            if abs(error) < 0.05:  # ~3 grados
                self.girando = False
                self.parar()
                time.sleep(0.5)
                
                if self.esquiva_paso == 1:
                    # Paso 2: Avanzar lento para rodear
                    self.get_logger().info('Esquiva paso 2: Avanzar lento para rodear (4s)')
                    self.mover_adelante(4.0, velocidad=0.1)
                    time.sleep(0.5)
                    
                    # Paso 3: Girar 90 grados a la izquierda
                    self.get_logger().info('Esquiva paso 3: Girar izquierda')
                    self.target_yaw = self.current_yaw - math.pi / 2.0  # 90 grados izquierda
                    self.girando = True
                    self.esquiva_paso = 3
                    
                elif self.esquiva_paso == 3:
                    # Paso 4: Avanzar lento para completar el rodeo
                    self.get_logger().info('Esquiva paso 4: Avanzar lento para completar rodeo (4s)')
                    self.mover_adelante(4.0, velocidad=0.1)
                    time.sleep(0.5)
                    
                    # Paso 5: Girar 45 grados a la izquierda para reorientarse
                    self.get_logger().info('Esquiva paso 5: Reorientarse')
                    self.target_yaw = self.current_yaw - math.pi / 4.0  # 45 grados izquierda
                    self.girando = True
                    self.esquiva_paso = 5
                    
                elif self.esquiva_paso == 5:
                    # Esquiva completada - Ahora comenzar el dibujo del 11
                    self.get_logger().info('ESQUIVA COMPLETADA - Iniciando dibujo del numero 11')
                    self.esquivando = False
                    self.esquiva_completada = True
                    self.obstaculo_procesado = True
                    self.delete_obstaculo()
                    # Avanzar un poco más para posicionarse mejor y COMENZAR DIRECTAMENTE EL PRIMER PALO
                    self.get_logger().info('Posicionandose para primer palo del 11')
                    self.mover_adelante(2.0)
                    self.movimiento_completado = False
                    self.current_step = 2  # CORREGIDO: Saltar directamente al paso 2 (primer palo)
            else:
                twist = Twist()
                twist.angular.z = -0.8 if error < 0 else 0.8
                self.publisher_.publish(twist)
            return
            
        # Control de giro normal
        elif self.girando:
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
                twist.angular.z = -0.8 if error < 0 else 0.8
                self.publisher_.publish(twist)
            return
            
        # Ejecutar secuencia - PRIMERO la esquiva, LUEGO el dibujo del 11
        if self.current_step == 0:
            # Paso 0: Acercarse al obstáculo y esquivarlo
            if not self.obstaculo_spawneado:
                self.get_logger().info('Acercandose al obstaculo...')
                # Avanzar hasta estar cerca del obstáculo
                twist = Twist()
                twist.linear.x = 0.15
                start = time.time()
                while time.time() - start < 3.0 and rclpy.ok():
                    self.publisher_.publish(twist)
                    time.sleep(0.1)
                
                # Spawnear obstaculo y ejecutar esquiva
                self.spawn_obstaculo()
                time.sleep(1)
                self.ejecutar_esquiva()
            elif self.esquiva_completada:
                # El dibujo del 11 comienza directamente en el paso 2
                self.movimiento_completado = False
                self.current_step = 2
            
        elif self.current_step == 1:
            # ESTE PASO YA NO SE USA - ELIMINADO PARA EVITAR GIRO EXTRA
            self.current_step = 2
            
        elif self.current_step == 2:
            if not self.movimiento_completado:
                self.get_logger().info('Paso 2: Primer palo del 11 (6s)')
                self.mover_adelante(6.0)
            else:
                self.movimiento_completado = False
                self.current_step = 3
            
        elif self.current_step == 3:
            if not self.giro_completado:
                self.get_logger().info('Paso 3: Giro 90 izquierda')
                self.empezar_giro_90(direccion=-1)
            else:
                self.giro_completado = False
                self.current_step = 4
            
        elif self.current_step == 4:
            if not self.movimiento_completado:
                self.get_logger().info('Paso 4: Espacio entre palos (2s)')
                self.mover_adelante(2.0)
            else:
                self.movimiento_completado = False
                self.current_step = 5
            
        elif self.current_step == 5:
            if not self.giro_completado:
                self.get_logger().info('Paso 5: Giro 90 izquierda')
                self.empezar_giro_90(direccion=-1)
            else:
                self.giro_completado = False
                self.current_step = 6
            
        elif self.current_step == 6:
            if not self.movimiento_completado:
                self.get_logger().info('Paso 6: Segundo palo del 11 (6s)')
                self.mover_adelante(6.0)
            else:
                self.movimiento_completado = False
                self.current_step = 7
            
        elif self.current_step == 7:
            self.is_drawing = False
            self.parar()
            self.get_logger().info('DIBUJO DEL NUMERO 11 CON ESQUIVA TERMINADO')
            
    def ejecutar_siguiente_paso(self):
        self.get_logger().info(f'Avanzando al paso {self.current_step + 1}')

def main(args=None):
    rclpy.init(args=args)
    node = DrawObstacleAvoidance()
    
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