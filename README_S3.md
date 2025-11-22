# Sprint 3: Simulación de TurtleBot3 y JetBot, con simbolos ArUco <!-- Título principal -->

**Este proyecto pertenece al Grupo 11 de la asignatura Proyectos PRII3.**  
Desarrollado por: **Juan Gómez-Rivas Pérez**, **David Cantó Fuentes**, y **Alvaro Fernández Serrano**  
Profesor evaluador: **pamuobe**

---

## <u>Descripción del proyecto</u>

Este proyecto implementa la simulación del TurtleBot3 Waffle en el mundo F1L3 de Gazebo, incluyendo la configuración de archivos launch personalizados, la gestión de mundos personalizados, y algoritmos de SLAM y Navegación.

---

## <u>Estructura del workspace (NO ACTUALIZADO)</u>
```bash
g11_prii3_ws/
├── src/
│ ├── g11_prii3/
│ │ ├── prii3_turtlesim_node.py # Nodo para turtlesim (Sprint 1)
│ │ ├── laser_filter.py # Filtro LIDAR
│ │ └── sprint2_demo.py # Demo Sprint 2
│ ├── g11_prii3_move_turtlebot/
│ │ ├── draw_number_simulation.py # Dibujo básico del "11"
│ │ ├── draw_collision_avoidance.py # Dibujo con detección de colisiones
│ │ ├── draw_obstacle_avoidance.py # Dibujo con esquiva de obstáculos
│ │ ├── obstacle_spawner.py # Spawneo de obstáculos
│ │ ├── collision_avoidance_simulation.py # Simulación de colisiones
│ │ ├── obstacle_avoidance_simulation.py # Simulación de obstáculos
│ │ ├── trajectory_controller.py # Controlador de trayectorias
│ │ ├── test_turtlebot3.py # Script de pruebas
│ │ ├── turtlebot3_teleop_keyboard.py # Teleoperación con teclado
│ │ └── g11_prii3_move_turtlebot.py # Movimiento automático
│ └── g11_prii3_move_jetbot/
│ └── draw_number_jetbot.py # Dibujo para JetBot
├── launch/
│ ├── sprint2_complete.launch.py # Lanzamiento completo
│ ├── sprint2_obstacle_avoidance.launch.py # Esquiva de obstáculos
│ └── sprint2_draw_number_jetbot.launch.py # Dibujo con JetBot
├── package.xml
├── setup.py
└── setup.cfg
```
---

## <u>Requisitos</u>

- **Ubuntu 20.04** (ROS 2 Humble)
- **ROS 2 Foxy** (o superior compatible)
- **Python 3.8+**
- **Gazebo Garden** (o versión compatible)

### Instalación de dependencias (NO ACTUALIZADO):

```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade -y

# Instalar TurtleBot3 para ROS 2 Humble
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-simulations

# Instalar Gazebo
sudo apt install gazebo-garden libgazebo-garden-dev

# Dependencias adicionales de ROS
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

## <u>Ejecución</u>
**1. Compilación del workspace**
```bash
cd ~/Escritorio/UPV/proyecto_3/g11_prii3_ws
colcon build --packages-select g11_prii3
source install/setup.bash
```

**2. Diferentes ejecuciones**
- Lanzar el Mundo F1L3 con TurtleBot3:
```bash
cd ~/Escritorio/UPV/proyecto_3/g11_prii3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch g11_prii3 f1l3_world.launch.py
```

- SLAM (Simultaneous Localization and Mapping)
    1. Lanzar SLAM para Crear el Mapa
```bash
# Terminal 1: Lanzar el mundo y el robot
source ~/Escritorio/UPV/proyecto_3/g11_prii3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch g11_prii3 f1l3_world.launch.py

# Terminal 2: Lanzar SLAM Toolbox
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Lanzar RViz para visualización
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run rviz2 rviz2 -d /opt/ros/foxy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

    2. Lanzar SLAM para Crear el Mapa   
```bash
# Terminal 4: Control por teclado
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

    3. Guardar el Mapa
```bash
# Terminal 5: Guardar el mapa (IMPORTANTE: usar nombre único)
source /opt/ros/foxy/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/mapa_f1l3_g11
```

- NAVEGACIÓN 
    1. Navegación con RViz
```bash
# Terminal 1: Lanzar el mundo y el robot
source ~/Escritorio/UPV/proyecto_3/g11_prii3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch g11_prii3 f1l3_world.launch.py

# Terminal 2: Lanzar Navigation2 con el mapa creado
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=~/mapa_f1l3_g11.yaml

# Terminal 3: Lanzar RViz
source /opt/ros/foxy/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/foxy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

    2. Navegación con Nodos del Sprint   
```bash
# Terminal 1: Lanzar el mundo y el robot
source ~/Escritorio/UPV/proyecto_3/g11_prii3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch g11_prii3 f1l3_world.launch.py

# Terminal 2: Lanzar Navigation2
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=~/mapa_f1l3_g11.yaml

# Terminal 3: Ejecutar waypoint navigator
source ~/Escritorio/UPV/proyecto_3/g11_prii3_ws/install/setup.bash
ros2 run g11_prii3_nav_turtlebot waypoint_navigator
```

---

## <u>CHAT DE DEEPSEEK
https://chat.deepseek.com/share/g4mfmzvifwxsh27ts8

---

## <u>Características técnicas</u>

**Algoritmos implementados:**
- Control PID para giros precisos
- Detección LIDAR para evitación de colisiones
- Maniobras de esquiva con giros de 45° y 90°
- Spawneo dinámico de obstáculos en tiempo de ejecución
- Control no bloqueante para ejecución fluida

**Servicios ROS disponibles:**
- Parada/reanudación del dibujo
- Reinicio de trayectorias
- Spawneo/eliminación de obstáculos

## <u>Control de versión</u>

**Repositorio GitHub:**
https://github.com/jgomper/g11_prii3_ws

**Rama principal:** main  
**Estructura:** Paquetes separados para TurtleBot3 (simulación) y JetBot (robot real)












