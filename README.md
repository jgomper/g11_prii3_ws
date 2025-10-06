# Sprint 1: Control de Turtlesim - “11” <!-- Título principal -->

**Este proyecto pertenece al Grupo 11 de la asignatura Proyectos PRII3.**  
Desarrollado por: **Juan Gómez-Rivas Pérez**, **David Cantó Fuentes**, y **Alvaro Fernández Serrano**  
Profesor evaluador: **pamuobe**

---

## <u>Descripción del proyecto</u>

El paquete `g11_prii3_turtlesim` implementa un nodo de control para dibujar el número “11” utilizando el simulador turtlesim de ROS 2.

El primer "1" y el segundo "1" se dibujan de forma separada mediante teletransporte  
y control de servicios (`/turtle1/teleport_absolute`, `/turtle1/set_pen`).

---

## <u>Estructura del workspace</u>

```
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
```

---

## <u>Requisitos</u>

- **Ubuntu 20.04**
- **ROS 2 Foxy** (o superior compatible)
- **Python 3.8+**
- **turtlesim instalado:**
  ```bash
  sudo apt install ros-foxy-turtlesim
  ```

---

## <u>Ejecución</u>

1. **Cargar el workspace**
    ```bash
    cd ~/Escritorio/UPV/proyecto_3/g11_prii3_ws
    colcon build
    source install/setup.bash
    ```
2. **Ejecutar el lanzamiento**
    ```bash
    ros2 launch g11_prii3_turtlesim turtlesim_launch.py
    ```

La tortuga dibujará el número “11” en pantalla, si queremos pausar, reunudar y reinicar el dibujo, debremos de ejecutar diferentes codigos, estos son:
- **Pausar la ejecución**
	```bash
	ros2 service call /draw_control std_srvs/srv/SetBool "{data: false}"
	```
- **Reanudar la ejecución**
	```bash
	ros2 service call /draw_control std_srvs/srv/SetBool "{data: true}"
	```
- **Reiniciar la ejecución**
	```bash
	ros2 service call /reset_drawing std_srvs/srv/Empty "{}"
	```
---

## <u>Control de versión</u>

Repositorio GitHub:
[https://github.com/jgomper/g11_prii3_ws](https://github.com/jgomper/g11_prii3_ws)






















