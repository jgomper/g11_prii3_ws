#!/usr/bin/env python3
"""
aruco_detector_autonomous.py
Detector de ArUcos mejorado para navegación autónoma - FILTRADO IDs 5,6,17
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import cv2.aruco as aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time

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
        
        # Definir el diccionario ArUco - USAR DICT_4X4_50 que es más común
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        
        # MEJORAR PARÁMETROS DE DETECCIÓN
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03  # Más sensible a marcadores pequeños
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.minDistanceToBorder = 3
        self.parameters.minOtsuStdDev = 5.0
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.parameters.maxErroneousBitsInBorderRate = 0.35
        self.parameters.errorCorrectionRate = 0.6
        
        # IDs VÁLIDOS que queremos detectar
        self.valid_aruco_ids = [5, 6, 17]
        
        # Variables para control de detección
        self.last_detected_id = -1
        self.consecutive_detections = 0
        self.detection_threshold = 2  # Reducido para mayor sensibilidad
        self.last_publish_time = 0
        self.publish_interval = 1.0  # Publicar cada 1 segundo máximo
        
        self.get_logger().info('Detector de ArUcos MEJORADO - Solo IDs 5, 6, 17')
        self.get_logger().info('Usando diccionario DICT_4X4_50')
        
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
            
            # MEJORA: Aplicar filtro para mejorar contraste
            gray = cv2.equalizeHist(gray)
            
            # Detectar marcadores ArUco
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            current_detection = -1
            detection_confirmed = False
            
            if ids is not None:
                # Filtrar solo los IDs válidos (5, 6, 17)
                valid_detections = []
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    if marker_id in self.valid_aruco_ids:
                        valid_detections.append(marker_id)
                
                if valid_detections:
                    # Tomar el primer ArUco válido detectado
                    current_detection = valid_detections[0]
                    
                    # Dibujar marcadores detectados
                    image_with_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
                    
                    # Añadir texto con ID y información (solo para IDs válidos)
                    for i in range(len(ids)):
                        marker_id = int(ids[i][0])
                        if marker_id in self.valid_aruco_ids:
                            center = np.mean(corners[i][0], axis=0).astype(int)
                            
                            # Calcular tamaño del marcador para verificar calidad
                            perimeter = cv2.arcLength(corners[i][0], True)
                            
                            cv2.putText(
                                image_with_markers, 
                                f"ID: {marker_id} (VALIDO)", 
                                (center[0] - 60, center[1] - 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.6,
                                (0, 255, 0), 
                                2
                            )
                            self.get_logger().info(f'✅ ArUco ID {marker_id} detectado (VÁLIDO)', 
                                                  throttle_duration_sec=1.0)
                        else:
                            # Marcar IDs no válidos en rojo
                            center = np.mean(corners[i][0], axis=0).astype(int)
                            cv2.putText(
                                image_with_markers, 
                                f"ID: {marker_id} (NO VALIDO)", 
                                (center[0] - 60, center[1] - 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.6,
                                (0, 0, 255), 
                                2
                            )
                            self.get_logger().info(f'❌ ArUco ID {marker_id} detectado (NO VÁLIDO)', 
                                                  throttle_duration_sec=2.0)
                    
                    # Mostrar imagen
                    cv2.imshow('ArUco Detection - Autonomous', image_with_markers)
                    cv2.waitKey(1)
                    
                else:
                    # Mostrar imagen con detecciones no válidas
                    image_with_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
                    cv2.imshow('ArUco Detection - Autonomous', image_with_markers)
                    cv2.waitKey(1)
                    self.get_logger().info('Solo se detectaron ArUcos no válidos', throttle_duration_sec=2.0)
                
            else:
                # Mostrar imagen original si no hay detecciones
                cv2.imshow('ArUco Detection - Autonomous', cv_image)
                cv2.waitKey(1)
                
                # Log de no detección (menos frecuente)
                self.get_logger().info('Buscando ArUcos válidos (5, 6, 17)...', throttle_duration_sec=3.0)
            
            # Lógica de confirmación de detección MEJORADA (solo para IDs válidos)
            if current_detection in self.valid_aruco_ids:
                if current_detection == self.last_detected_id:
                    self.consecutive_detections += 1
                else:
                    self.consecutive_detections = 1
                    self.last_detected_id = current_detection
                
                # Publicar detección confirmada (más permisivo)
                current_time = time.time()
                if (self.consecutive_detections >= self.detection_threshold and 
                    current_detection != -1 and
                    current_time - self.last_publish_time > self.publish_interval):
                    
                    aruco_msg = Int32()
                    aruco_msg.data = current_detection
                    self.aruco_pub.publish(aruco_msg)
                    self.last_publish_time = current_time
                    self.get_logger().info(f"🚀 PUBLICADO: ArUco {current_detection} confirmado y publicado")
            else:
                # Resetear si se detecta un ID no válido
                self.last_detected_id = -1
                self.consecutive_detections = 0
                
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
