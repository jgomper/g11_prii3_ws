# Sprint 2: Simulación de TurtleBot3 <!-- Título principal -->
# README_S2.md - TurtleBot3 Simulation - Simulacion por Gazebo - Tarea

## <u>Descripción</u>
Este proyecto contiene los paquetes ROS2 para la simulación y control del TurtleBot3 en diferentes entornos de Gazebo.


## <u>Estructura del workspace</u>

```
g11_prii3_ws/
├── src/g11_prii3/
│ ├── g11_prii3_move_jetbot.py # Control para JetBot
│ ├── g11_prii3_move_turtlebot.py # Movimiento automático TurtleBot3
│ ├── turtlebot3_teleop_keyboard.py # Teleoperación con teclado
│ ├── test_turtlebot3.py # Script de pruebas
│ ├── prii3_turtlesim_node.py # Nodo para turtlesim
│ ├── drawer_number_gazebo.py # Dibujo de números en Gazebo
│ └── init.py
├── launch/
│ ├── sprint1.launch.py
│ ├── drawer_number_gazebo.launch.py
│ └── turtlebot3_empty_world.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

---

## <u>Prerequisitos</u>
### Instalación de Dependencias

# Actualizar sistema
```bash
sudo apt update
```

# Instalar TurtleBot3 para ROS2 Foxy
```bash
sudo apt install ros-foxy-turtlebot3*
sudo apt install ros-foxy-turtlebot3-gazebo
sudo apt install ros-foxy-turtlebot3-simulations
```

# Instalar Gazebo
```bash
sudo apt install gazebo11 libgazebo11-dev
```

# Dependencias adicionales de ROS
```bash
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-ros2-control
sudo apt install ros-foxy-ros2-controllers
```
---

## <u>Ejecución</u>

1. **Cargar el workspace**
```bash
cd ~/Escritorio/UPV/proyecto_3/g11_prii3_ws
colcon build
source install/setup.bash
```
2. **Mundos de Simulación Disponibles**
- **Mundo Vacío**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
    
- **Mundo TurtleBot3**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

- **Casa TurtleBot3**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

3. **Métodos de Control**
- **Teleoperación con Teclado (Recomendado)**
```bash
source install/setup.bash
ros2 run g11_prii3 turtlebot3_teleop_keyboard
```
Controles:

    	W: Mover hacia adelante
	S: Mover hacia atrás
    	A: Girar izquierda
	D: Girar derecha
    	Espacio: Detener
    	Q: Aumentar velocidad
    	E: Disminuir velocidad
    	Ctrl+C: Salir

- **Movimiento Automático**
```bash
source install/setup.bash
ros2 run g11_prii3 g11_prii3_move_turtlebot
```

- **Script de Pruebas**
```bash
source install/setup.bash
ros2 run g11_prii3 test_turtlebot3
```
Dibujar número:
```bash
cd ~/Escritorio/UPV/proyecto_3/g11_prii3_ws/src/g11_prii3_move_turtlebot
python3 draw_number_simulation.py
```
Pausar, reanudar y reiniciar dibujo (abrir otro terminal):
```bash
ros2 service call /stop_drawing std_srvs/srv/Empty     # Pausa el dibujo
ros2 service call /resume_drawing std_srvs/srv/Empty   # Reanuda el dibujo
ros2 service call /reset_drawing std_srvs/srv/Empty    # Reinicia el dibujo desde el paso 0
```
---

## <u>Control de versión</u>

Repositorio GitHub:
[https://github.com/jgomper/g11_prii3_ws](https://github.com/jgomper/g11_prii3_ws)


## <u>Control de versión</u>

Repositorio GitHub:  
[https://github.com/jgomper/g11_prii3_ws](https://github.com/jgomper/g11_prii3_ws)




















