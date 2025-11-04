#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBot3Mover(Node):
    def __init__(self):
        super().__init__('turtlebot3_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('TurtleBot3 Mover Node Started')
        
    def move_forward(self, duration=2.0, speed=0.2):
        """Mueve el TurtleBot3 hacia adelante"""
        twist = Twist()
        twist.linear.x = speed
        self.publisher_.publish(twist)
        time.sleep(duration)
        
        # Detener el robot
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info(f'Moved forward for {duration} seconds')
        
    def rotate(self, duration=2.0, speed=1.0):
        """Rota el TurtleBot3"""
        twist = Twist()
        twist.angular.z = speed
        self.publisher_.publish(twist)
        time.sleep(duration)
        
        # Detener el robot
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info(f'Rotated for {duration} seconds')
        
    def stop(self):
        """Detiene completamente el robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    
    mover = TurtleBot3Mover()
    
    try:
        # Ejemplo de movimiento
        mover.get_logger().info('Starting movement sequence...')
        
        # Mover hacia adelante
        mover.move_forward(duration=3.0, speed=0.2)
        time.sleep(1)
        
        # Rotar
        mover.rotate(duration=2.0, speed=0.5)
        time.sleep(1)
        
        # Mover hacia adelante de nuevo
        mover.move_forward(duration=2.0, speed=0.1)
        
        mover.get_logger().info('Movement sequence completed')
        
    except KeyboardInterrupt:
        mover.get_logger().info('Keyboard interrupt received')
    finally:
        mover.stop()
        mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()