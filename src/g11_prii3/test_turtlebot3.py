#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestTurtleBot3(Node):
    def __init__(self):
        super().__init__('test_turtlebot3')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(5.0, self.publish_command)
        self.count = 0
        
    def publish_command(self):
        twist = Twist()
        
        if self.count % 4 == 0:
            # Mover adelante
            twist.linear.x = 0.2
            self.get_logger().info('Moving forward')
        elif self.count % 4 == 1:
            # Girar izquierda
            twist.angular.z = 1.0
            self.get_logger().info('Turning left')
        elif self.count % 4 == 2:
            # Mover adelante
            twist.linear.x = 0.2
            self.get_logger().info('Moving forward')
        else:
            # Girar derecha
            twist.angular.z = -1.0
            self.get_logger().info('Turning right')
            
        self.publisher_.publish(twist)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestTurtleBot3()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.publisher_.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()