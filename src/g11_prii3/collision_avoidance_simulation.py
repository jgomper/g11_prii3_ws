#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CollisionAvoidanceSimulation(Node):
    def __init__(self):
        super().__init__('collision_avoidance_simulation')
        
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
        
        self.get_logger().info('Collision Avoidance Simulation iniciado')
        self.get_logger().info('Avanzando en línea recta - Buscando obstáculos...')
        
    def lidar_callback(self, msg):
        # Solo detectar obstáculos en el frente (±45 grados)
        num_readings = len(msg.ranges)
        center = num_readings // 2
        sector = 90  # ±45 grados
        start = center - int(sector * num_readings / 360 / 2)
        end = center + int(sector * num_readings / 360 / 2)
        
        min_dist = float('inf')
        for i in range(start, end):
            if i < len(msg.ranges) and 0.1 < msg.ranges[i] < 5.0:
                if msg.ranges[i] < min_dist:
                    min_dist = msg.ranges[i]
        
        if min_dist < self.safe_distance:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.get_logger().warn(f'OBSTACULO DETECTADO AL FRENTE a {min_dist:.2f}m - PARANDO')
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.get_logger().info('Camino libre - AVANZANDO EN LÍNEA RECTA')
            
    def control_loop(self):
        twist = Twist()
        
        if not self.obstacle_detected:
            # MOVIMIENTO EN LÍNEA RECTA
            twist.linear.x = self.base_speed
            twist.angular.z = 0.0
        else:
            # DETENER completamente cuando hay obstáculo
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_pub.publish(twist)
        
    def destroy_node(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceSimulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()