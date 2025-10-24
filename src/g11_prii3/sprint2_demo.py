#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
import threading

class Sprint2Demo(Node):
    def __init__(self):
        super().__init__('sprint2_demo')
        
        # Cliente para el servicio de control de dibujo
        self.drawing_client = self.create_client(SetBool, 'control_drawing')
        
        while not self.drawing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio control_drawing...')
            
        self.get_logger().info('Sprint 2 Demo Node started - Grupo 11')
        
    def run_demo(self):
        """Ejecuta la demostración completa del Sprint 2"""
        self.get_logger().info('=== DEMOSTRACIÓN SPRINT 2 - GRUPO 11 ===')
        
        # Esperar inicialización
        time.sleep(5)
        
        # Paso 1: Iniciar dibujo del número 11
        self.get_logger().info('Paso 1: Iniciando dibujo del número 11...')
        
        # Paso 2: Pausar dibujo después de 10 segundos
        time.sleep(10)
        self.get_logger().info('Paso 2: Pausando dibujo...')
        self.control_drawing(False)  # Pausar
        
        time.sleep(3)
        
        # Paso 3: Reanudar dibujo
        self.get_logger().info('Paso 3: Reanudando dibujo...')
        self.control_drawing(True)  # Reanudar
        
        # Paso 4: Esperar a que termine el dibujo
        time.sleep(20)
        
        # Paso 5: Demostrar evitación de obstáculos
        self.get_logger().info('Paso 4: Modo de evitación de obstáculos activo')
        self.get_logger().info('Coloca obstáculos frente al robot para probar la evitación...')
        
        # Mantener el nodo activo
        try:
            while rclpy.ok():
                time.sleep(1)
        except KeyboardInterrupt:
            pass
            
    def control_drawing(self, command):
        """Llama al servicio para controlar el dibujo"""
        request = SetBool.Request()
        request.data = command
        
        future = self.drawing_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Control drawing: {future.result().message}')
            return True
        else:
            self.get_logger().error('Error al llamar al servicio')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = Sprint2Demo()
    
    # Ejecutar demostración en hilo separado
    demo_thread = threading.Thread(target=node.run_demo)
    demo_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        demo_thread.join()

if __name__ == '__main__':
    main()