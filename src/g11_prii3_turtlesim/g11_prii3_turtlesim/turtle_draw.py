import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import SetBool, Empty
import time

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Nodo controller inicializado (turtle_draw.py)')

        # Variables internas
        self.drawing_paused = False
        self.current_step = 0

        # Clientes de servicios
        self.cli_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.cli_clear = self.create_client(Empty, '/clear')

        # Publicador de velocidad
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Servicios personalizados
        self.srv_control = self.create_service(SetBool, 'draw_control', self.control_callback)
        self.srv_reset = self.create_service(Empty, 'reset_drawing', self.reset_callback)

        # Temporizador (0.5 seg entre pasos)
        self.timer = self.create_timer(0.5, self.draw_step)

        self.get_logger().info('Servicios disponibles:')
        self.get_logger().info('- /draw_control (pausar / reanudar)')
        self.get_logger().info('- /reset_drawing (reiniciar dibujo con limpieza)')

    # ------------------ SERVICIOS ------------------ #

    def control_callback(self, request, response):
        """Pausa o reanuda el dibujo."""
        if request.data:
            self.get_logger().info('Reanudando dibujo...')
            self.drawing_paused = False
            response.message = "Dibujo reanudado"
        else:
            self.get_logger().info('Pausando dibujo...')
            self.drawing_paused = True
            response.message = "Dibujo pausado"
        response.success = True
        return response

    def reset_callback(self, request, response):
        """Reinicia el dibujo desde cero y limpia la pantalla."""
        self.get_logger().info('Reiniciando dibujo y limpiando pantalla...')
        self.current_step = 0
        self.drawing_paused = False

        self.clear_screen()
        self.set_pen(255, 255, 255, 5, 1)
        self.teleport(3.0, 2.0, 1.57)
        self.set_pen(255, 255, 255, 7, 0)
        return response

    # ------------------ FUNCIONES AUXILIARES ------------------ #

    def clear_screen(self):
        """Limpia la pantalla."""
        if self.cli_clear.wait_for_service(timeout_sec=2.0):
            self.cli_clear.call_async(Empty.Request())
        else:
            self.get_logger().warn("Servicio /clear no disponible")

    def set_pen(self, r, g, b, width, off):
        """Cambia color/grosor del lápiz o lo apaga."""
        if self.cli_pen.wait_for_service(timeout_sec=2.0):
            req = SetPen.Request()
            req.r, req.g, req.b = r, g, b
            req.width = width
            req.off = off
            self.cli_pen.call_async(req)
        else:
            self.get_logger().warn("Servicio /set_pen no disponible")

    def teleport(self, x, y, theta):
        """Teletransporta sin dejar trazo."""
        if self.cli_teleport.wait_for_service(timeout_sec=2.0):
            req = TeleportAbsolute.Request()
            req.x, req.y, req.theta = x, y, theta
            self.cli_teleport.call_async(req)
        else:
            self.get_logger().warn("Servicio /teleport_absolute no disponible")

    def move_up(self, duration):
        """Sube en línea recta durante cierto tiempo."""
        twist = Twist()
        twist.linear.x = 6.0
        self.publisher.publish(twist)
        time.sleep(3.0)
        twist.linear.x = 0.0
        self.publisher.publish(twist)

    # ------------------ DIBUJO PRINCIPAL ------------------ #

    def draw_step(self):
        if self.drawing_paused:
            return

        if self.current_step == 0:
            self.get_logger().info("Teletransportando al inicio del primer 1...")
            self.set_pen(255, 255, 255, 5, 1)
            self.teleport(3.0, 2.0, 1.57)
            self.set_pen(255, 255, 255, 7, 0)

        elif self.current_step == 1:
            self.get_logger().info("Dibujando el primer 1...")
            self.move_up(15.0) 
            
        elif self.current_step == 2:
            self.get_logger().info("Teletransportando rápidamente al segundo 1...")
            self.set_pen(255, 255, 255, 5, 1)
            self.teleport(6.0, 2.0, 1.57)  
            time.sleep(0.5)  
            self.set_pen(255, 255, 255, 7, 0)

        elif self.current_step == 3:
            self.get_logger().info("Dibujando el segundo 1...")
            self.move_up(15.0) 

        elif self.current_step >= 4:
            self.get_logger().info("Dibujo del número 11 completado.")
            self.destroy_timer(self.timer)

        self.current_step += 1


# ------------------ MAIN ------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDraw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

