#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class DrawNumberJetBot(Node):
    def __init__(self):
        super().__init__('draw_number_jetbot')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Estado general / secuencia
        self.is_drawing = True
        self.current_step = 0

        # Orientación
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.girando = False

        # Movimiento lineal (no bloqueante)
        self.moviendo = False
        self.move_end_time = 0.0
        self.move_speed = 0.15  # m/s por defecto

        # Parámetros de control
        self.Kp_turn = 0.5      # ganancia proporcional para giro
        self.MAX_ANGULAR = 0.5  # rad/s límite angular
        self.ANGLE_TOLERANCE = 0.02  # rad (~1.1°)

        self.get_logger().info('Iniciando dibujo del número 11 (no bloqueante)')

        # Timer principal (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    # --------------------------
    # ODOMETRÍA Y ORIENTACIÓN
    # --------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # Calcular yaw (ángulo de orientación)
        self.current_yaw = math.atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z))

    # --------------------------
    # OPERACIONES INICIADORAS (no bloqueantes)
    # --------------------------
    def start_move(self, segundos, speed=None):
        """Inicia un movimiento hacia adelante no bloqueante durante 'segundos'."""
        if speed is not None:
            self.move_speed = speed
        self.moviendo = True
        self.move_end_time = time.time() + float(segundos)
        self.get_logger().info(f'Iniciando movimiento por {segundos}s a {self.move_speed} m/s')

    def start_turn_90(self, direccion=-1):
        """Inicia un giro de 90° (no bloqueante). direccion: -1 izquierda, 1 derecha"""
        # Calcula target en [-pi,pi] sumando y normalizando
        self.target_yaw = self.normalize_angle(self.current_yaw + direccion * math.pi / 2.0)
        self.girando = True
        self.get_logger().info(f'Iniciando giro 90° {"izquierda" if direccion==-1 else "derecha"}: target={self.target_yaw:.3f} rad')

    def normalize_angle(self, ang):
        """Normaliza un ángulo a [-pi, pi]."""
        a = ang
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def stop_robot(self):
        twist = Twist()
        self.publisher_.publish(twist)

    # --------------------------
    # BUCLE DE CONTROL PRINCIPAL (no bloqueante)
    # --------------------------
    def control_loop(self):
        if not self.is_drawing:
            return

        now = time.time()

        # 1) Si estamos moviendo linealmente, mantener hasta tiempo cumplido
        if self.moviendo:
            if now < self.move_end_time and rclpy.ok():
                twist = Twist()
                twist.linear.x = self.move_speed
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                return
            else:
                # Fin del movimiento
                self.moviendo = False
                self.stop_robot()
                self.get_logger().info('Movimiento completado')
                self.ejecutar_siguiente_paso()
                return

        # 2) Si estamos girando, aplicar P-controller hasta alcanzar tolerancia
        if self.girando:
            # error = target - current (normalizado)
            error = self.target_yaw - self.current_yaw
            # normalizar en -pi..pi
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi

            # Comprobación de finalización
            if abs(error) < self.ANGLE_TOLERANCE:
                self.girando = False
                self.stop_robot()
                self.get_logger().info(f'Giro completado. yaw={self.current_yaw:.3f}')
                self.ejecutar_siguiente_paso()
                return
            else:
                # P-controller con saturación
                turn = self.Kp_turn * error
                # limitar
                if turn > self.MAX_ANGULAR:
                    turn = self.MAX_ANGULAR
                elif turn < -self.MAX_ANGULAR:
                    turn = -self.MAX_ANGULAR

                twist = Twist()
                twist.angular.z = turn
                twist.linear.x = 0.0
                self.publisher_.publish(twist)
                return

        # 3) Si no estamos moviendo ni girando, arrancar la acción del step actual
        if self.current_step == 0:
            # Paso 1: Primer palo (12s)
            self.get_logger().info('Paso 1: Primer palo (12s)')
            self.start_move(12.0)
            # no incrementamos el paso ahora; se hace cuando termine mover

        elif self.current_step == 1:
            # Paso 2: Giro 90 izquierda
            self.get_logger().info('Paso 2: Giro 90 izquierda')
            self.start_turn_90(direccion=-1)

        elif self.current_step == 2:
            # Paso 3: Espacio (5s)
            self.get_logger().info('Paso 3: Espacio (5s)')
            self.start_move(5.0)

        elif self.current_step == 3:
            # Paso 4: Giro 90 izquierda
            self.get_logger().info('Paso 4: Giro 90 izquierda')
            self.start_turn_90(direccion=-1)

        elif self.current_step == 4:
            # Paso 5: Segundo palo (12s)
            self.get_logger().info('Paso 5: Segundo palo (12s)')
            self.start_move(12.0)

        elif self.current_step == 5:
            # Terminamos
            self.is_drawing = False
            self.stop_robot()
            self.get_logger().info('✅ DIBUJO TERMINADO')

    def ejecutar_siguiente_paso(self):
        # Avanza al siguiente paso cuando una acción (movimiento/giro) termina
        self.current_step += 1
        self.get_logger().info(f'Avanzando al paso {self.current_step}')

def main(args=None):
    rclpy.init(args=args)
    node = DrawNumberJetBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

