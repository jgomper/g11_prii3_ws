#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        
        self.Angle = 360
        self.Dist = 50

        # Parámetros (reemplazan dynamic_reconfigure)
        self.declare_parameter('laser_angle', 360.0)
        self.declare_parameter('distance', 50.0)

        # Suscriptor y publicador
        self.sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        self.pub = self.create_publisher(
            LaserScan,
            'filteredscan',
            10
        )

        # Timer opcional para monitorizar parámetros dinámicamente
        self.timer = self.create_timer(1.0, self.update_params)

        self.get_logger().info('LaserFilter node started.')

    def update_params(self):
        """Actualiza parámetros en tiempo de ejecución."""
        self.Angle = float(self.get_parameter('laser_angle').value)
        self.Dist = float(self.get_parameter('distance').value)

    def laser_callback(self, data: LaserScan):
        newdata = LaserScan()
        newdata.header = data.header
        newdata.angle_min = data.angle_min
        newdata.angle_max = data.angle_max
        newdata.angle_increment = data.angle_increment
        newdata.time_increment = data.time_increment
        newdata.scan_time = data.scan_time
        newdata.range_min = data.range_min
        newdata.range_max = data.range_max

        # Convertir a listas modificables
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)

        length = len(newdata.ranges)
        Index = int(self.Angle / 2 * length / 360)

        # Filtrado por distancia
        for i in range(length):
            if newdata.ranges[i] > self.Dist:
                newdata.ranges[i] = 0.0
                if len(newdata.intensities) > i:
                    newdata.intensities[i] = 0.0

        # Filtrado por ángulo
        for i in range(Index, length - Index):
            newdata.ranges[i] = 0.0
            if len(newdata.intensities) > i:
                newdata.intensities[i] = 0.0

        self.pub.publish(newdata)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()