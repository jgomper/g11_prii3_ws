#!/usr/bin/env python3
"""
aruco_detector_autonomous.py
Detector de ArUcos para navegación autónoma - Publica IDs detectados
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import cv2.aruco as aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ArucoDetectorAutonomous(Node):
    def __init__(self):
        super().__init__('aruco_detector_autonomous')
        
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
        
        # Publisher para el ArUco detectado (-1 si no se detecta nada)
        self.aruco_pub = self.create_publisher(Int32, '/detected_aruco', 10)
        
        # Definir el diccionario ArUco (usando DICT_5X5_50 como en el código anterior)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        
        # Variables para control de detección
        self.last_detected_id = -1
        self.consecutive_detections = 0
        self.detection_threshold = 3  # Número de detecciones consecutivas para confirmar
        
        self.get_logger().info('Detector de ArUcos para navegación autónoma inicializado')
        
    def image_callback(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV
            if msg.encoding == 'rgb8':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn(f'Formato de imagen no soportado: {msg.encoding}')
                return
            
            # Convertir a escala de grises
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detectar marcadores ArUco
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            current_detection = -1
            
            if ids is not None:
                # Tomar el primer ArUco detectado
                current_detection = ids[0][0]
                
                # Dibujar marcadores detectados
                image_with_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
                
                # Añadir texto con ID
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
                cv2.imshow('ArUco Detection - Autonomous', image_with_markers)
                cv2.waitKey(1)
                
                # Log de detección
                self.get_logger().info(f'ArUco ID {current_detection} detectado', throttle_duration_sec=2.0)
                
            else:
                # Mostrar imagen original si no hay detecciones
                cv2.imshow('ArUco Detection - Autonomous', cv_image)
                cv2.waitKey(1)
            
            # Lógica de confirmación de detección
            if current_detection == self.last_detected_id:
                self.consecutive_detections += 1
            else:
                self.consecutive_detections = 1
                self.last_detected_id = current_detection
            
            # Publicar detección confirmada
            if self.consecutive_detections >= self.detection_threshold and current_detection != -1:
                aruco_msg = Int32()
                aruco_msg.data = current_detection
                self.aruco_pub.publish(aruco_msg)
                self.get_logger().info(f"ArUco {current_detection} confirmado y publicado")
                
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {str(e)}')

def main():
    rclpy.init()
    node = ArucoDetectorAutonomous()
    
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