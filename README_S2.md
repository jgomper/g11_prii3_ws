# Sprint 2: Simulación de TurtleBot3 y JetBot <!-- Título principal -->

**Este proyecto pertenece al Grupo 11 de la asignatura Proyectos PRII3.**  
Desarrollado por: **Juan Gómez-Rivas Pérez**, **David Cantó Fuentes**, y **Alvaro Fernández Serrano**  
Profesor evaluador: **pamuobe**

---

## <u>Descripción del proyecto</u>

El paquete `g11_prii3` implementa nodos de control para dibujar el número "11" y realizar evitación de obstáculos utilizando TurtleBot3 en Gazebo y JetBot.

### Características principales:
- **Dibujo autónomo** del número "11"
- **Evitación de colisiones** con detección por LIDAR
- **Evitación de obstáculos** con esquiva dinámica
- **Spawneo dinámico** de obstáculos en Gazebo
- **Control no bloqueante** para movimientos fluidos

---

## <u>Estructura del workspace</u>
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

### Instalación de dependencias:

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

**2. Lanzamiento de simulaciones**
- Dibujo básico del número "11":
```bash
ros2 launch g11_prii3 sprint2_complete.launch.py
```

- Evitación de obstáculos con spawneo dinámico
```bash
ros2 launch g11_prii3 sprint2_collision_avoidance.launch.py
```

- Esquivación de obstáculos con spawneo dinámico
```bash
ros2 launch g11_prii3 sprint2_obstacle_avoidance.launch.py
```

- Dibujo básico del número "11" con JetBot
```bash
ros2 launch g11_prii3 sprint2_draw_number_jetbot.launch.py
```

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












