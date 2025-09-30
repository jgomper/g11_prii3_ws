import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jgomper/Escritorio/UPV/proyecto_3/g11_prii3_ws/install/g11_prii3_turtlesim'
