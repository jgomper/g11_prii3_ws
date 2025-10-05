Proyecto: Control de Turtlesim - “11”

Este proyecto pertenece al Grupo 11 de la asignatura Robótica Inteligente (PRII3).
Desarrollado por: jgomper
Profesor evaluador: pamuobe


Descripción del proyecto

El paquete g11_prii3_turtlesim implementa un nodo de control para dibujar el número “11”
utilizando el simulador turtlesim de ROS 2.

El primer "1" y el segundo "1" se dibujan de forma separada mediante teletransporte
y control de servicios (/turtle1/teleport_absolute, /turtle1/set_pen).


Estructura del workspace

g11_prii3_ws/
│
├── src/
│   └── g11_prii3_turtlesim/
│       ├── launch/
│       │   └── turtlesim_launch.py
│       ├── g11_prii3_turtlesim/
│       │   └── turtle_draw.py
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       └── resource/
│           └── g11_prii3_turtlesim
│
├── install/
├── build/
└── log/


Requisitos

Ubuntu 20.04 o superior
ROS 2 Foxy (o superior compatible)
Python 3.8+
turtlesim instalado:
	sudo apt install ros-foxy-turtlesim


Ejecución
	1. Cargar el workspace
		cd ~/Escritorio/UPV/proyecto_3/g11_prii3_ws
		colcon build
		source install/setup.bash
	2. Ejecutar el lanzamiento
		ros2 launch g11_prii3_turtlesim turtlesim_launch.py

La tortuga dibujará el número “11” en pantalla.


Control de versión

Repositorio GitHub:
	https://github.com/jgomper/g11_prii3_ws






















