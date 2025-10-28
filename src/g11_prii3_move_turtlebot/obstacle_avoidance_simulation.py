#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class ObstacleAvoidanceSimulation(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_simulation')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.safe_distance = 1.0
        self.obstacle_detected = False
        self.base_speed = 0.2
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Obstacle Avoidance Simulation iniciado')
        
    def lidar_callback(self, msg):
        # Analizar sectores del LIDAR
        num_readings = len(msg.ranges)
        center_index = num_readings // 2
        
        front_sector = 30
        left_sector = 60
        right_sector = 60
        
        front_start = center_index - int(front_sector * num_readings / 360 / 2)
        front_end = center_index + int(front_sector * num_readings / 360 / 2)
        
        left_start = front_end
        left_end = left_start + int(left_sector * num_readings / 360)
        
        right_end = front_start
        right_start = right_end - int(right_sector * num_readings / 360)
        
        front_distances = [msg.ranges[i] for i in range(front_start, front_end) 
                          if i < len(msg.ranges) and msg.ranges[i] > 0.1]
        left_distances = [msg.ranges[i] for i in range(left_start, left_end) 
                         if i < len(msg.ranges) and msg.ranges[i] > 0.1]
        right_distances = [msg.ranges[i] for i in range(right_start, right_end) 
                          if i < len(msg.ranges) and msg.ranges[i] > 0.1]
        
        min_front = min(front_distances) if front_distances else float('inf')
        min_left = min(left_distances) if left_distances else float('inf')
        min_right = min(right_distances) if right_distances else float('inf')
        
        # Lógica de evitación avanzada
        if min_front < self.safe_distance:
            self.obstacle_detected = True
            if min_left > min_right and min_left > self.safe_distance:
                self.get_logger().info('Esquivando hacia DERECHA')
            elif min_right > min_left and min_right > self.safe_distance:
                self.get_logger().info('Esquivando hacia IZQUIERDA')
            else:
                self.get_logger().warn('Obstaculo cercano - DETENIENDO')
        else:
            self.obstacle_detected = False
            
    def control_loop(self):
        twist = Twist()
        
        if not self.obstacle_detected:
            twist.linear.x = self.base_speed
            twist.angular.z = 0.0
        else:
            # Comportamiento de esquiva
            twist.linear.x = self.base_speed * 0.3
            twist.angular.z = self.base_speed * 2.0  # Girar para esquivar
        
        self.cmd_pub.publish(twist)
        
    def destroy_node(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceSimulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()