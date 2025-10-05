#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleTurtleTest(Node):
    def __init__(self):
        super().__init__('simple_turtle_test')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Esperar un momento para que el publisher se establezca
        time.sleep(1)
        
        self.get_logger().info("TEST: Moviendo hacia adelante...")
        
        # Mover hacia adelante por 2 segundos
        twist = Twist()
        twist.linear.x = 1.0
        self.pub.publish(twist)
        
        # Timer para detenerse después de 2 segundos
        self.create_timer(2.0, self.stop)

    def stop(self):
        self.get_logger().info("TEST: Deteniendo...")
        twist = Twist()
        twist.linear.x = 0.0
        self.pub.publish(twist)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SimpleTurtleTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
