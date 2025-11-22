#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ArucoBagSimple(Node):
    def __init__(self):
        super().__init__('aruco_bag_simple')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        
        self.detected_ids = set()
        
        self.get_logger().info('Procesador de rosbag simple inicializado')
        
    def image_callback(self, msg):
        try:
            # Convertir manualmente sin cv_bridge
            if msg.encoding == 'rgb8':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn(f'Formato no soportado: {msg.encoding}')
                return
            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                for marker_id in ids:
                    self.detected_ids.add(marker_id[0])
                
                image_with_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    center = np.mean(corners[i][0], axis=0).astype(int)
                    
                    cv2.putText(
                        image_with_markers, 
                        f"ID: {marker_id}", 
                        (center[0] - 30, center[1] - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, 
                        (0, 255, 0), 
                        2
                    )
                
                cv2.putText(
                    image_with_markers,
                    f"IDs: {sorted(list(self.detected_ids))}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )
                
                cv2.imshow('ArUco Detection - Rosbag', image_with_markers)
                cv2.waitKey(1)
                
                self.get_logger().info(f'IDs detectados: {sorted(list(self.detected_ids))}')
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main():
    rclpy.init()
    node = ArucoBagSimple()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
