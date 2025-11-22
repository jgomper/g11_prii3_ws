#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ArucoDetectorSimple(Node):
    def __init__(self):
        super().__init__('aruco_detector_simple')
        
        # Usar QoS confiable para imágenes
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Suscriptor a la imagen de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Definir el diccionario ArUco
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        
        self.get_logger().info('Nodo detector de ArUco simple inicializado')
        
    def image_callback(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV manualmente
            # Asumimos formato RGB8
            if msg.encoding == 'rgb8':
                # Crear array numpy desde los datos de la imagen
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                # Convertir RGB to BGR para OpenCV
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn(f'Formato de imagen no soportado: {msg.encoding}')
                return
            
            # Convertir a escala de grises
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detectar marcadores ArUco
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            # Procesar detecciones
            if ids is not None:
                # Dibujar marcadores detectados
                image_with_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
                
                # Añadir texto con IDs
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    center = np.mean(corners[i][0], axis=0).astype(int)
                    
                    cv2.putText(
                        image_with_markers, 
                        f"ID: {marker_id}", 
                        (center[0] - 40, center[1] - 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.8,
                        (0, 255, 0), 
                        2
                    )
                
                # Mostrar imagen (opcional)
                cv2.imshow('ArUco Detection', image_with_markers)
                cv2.waitKey(1)
                
                # Log de detecciones
                id_list = [str(id[0]) for id in ids]
                self.get_logger().info(f'ArUco IDs detectados: {", ".join(id_list)}')
                
            else:
                # Mostrar imagen original si no hay detecciones
                cv2.imshow('ArUco Detection', cv_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {str(e)}')

def main():
    rclpy.init()
    node = ArucoDetectorSimple()
    
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
