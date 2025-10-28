#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import os

class TurtleBot3Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.linear_speed = 0.2
        self.angular_speed = 1.0
        
        self.get_logger().info('TurtleBot3 Teleop Keyboard Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W: Move forward')
        self.get_logger().info('  S: Move backward')
        self.get_logger().info('  A: Turn left')
        self.get_logger().info('  D: Turn right')
        self.get_logger().info('  Q: Increase speed')
        self.get_logger().info('  E: Decrease speed')
        self.get_logger().info('  Space: Stop')
        self.get_logger().info('  Ctrl+C: Exit')
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x03':  # Ctrl+C
                    break
                    
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == 'q':
                    self.linear_speed = min(0.5, self.linear_speed + 0.05)
                    self.angular_speed = min(2.0, self.angular_speed + 0.1)
                    self.get_logger().info(f'Speed increased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}')
                elif key == 'e':
                    self.linear_speed = max(0.05, self.linear_speed - 0.05)
                    self.angular_speed = max(0.1, self.angular_speed - 0.1)
                    self.get_logger().info(f'Speed decreased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}')
                    
                self.publisher_.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
        finally:
            # Detener el robot al salir
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = TurtleBot3Teleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()