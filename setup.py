from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'g11_prii3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'g11_prii3',
        'g11_prii3_move_turtlebot', 
        'g11_prii3_move_jetbot',
        'g11_prii3_nav_turtlebot',
        'g11_prii3_nav_jetbot'
    ],
    package_dir={
        'g11_prii3': 'src/g11_prii3',
        'g11_prii3_move_turtlebot': 'src/g11_prii3_move_turtlebot',
        'g11_prii3_move_jetbot': 'src/g11_prii3_move_jetbot', 
        'g11_prii3_nav_turtlebot': 'src/g11_prii3_nav_turtlebot',
        'g11_prii3_nav_jetbot': 'src/g11_prii3_nav_jetbot'
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # AÑADIR ESTAS LÍNEAS PARA WORLDS Y LAUNCH:
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        #(os.path.join('share', package_name, 'worlds'), glob('worlds/**/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # AGREGAR ARCHIVOS YAML PARA NAVEGACIÓN
        (os.path.join('share', package_name, 'g11_prii3_nav_turtlebot'), 
         glob('src/g11_prii3_nav_turtlebot/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jgomper',
    maintainer_email='jgomper@upv.es',
    description='G11 PRII3 package for TurtleBot3 simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodos en g11_prii3
            'prii3_turtlesim_node = g11_prii3.prii3_turtlesim_node:main',
            'laser_filter = g11_prii3.laser_filter:main',
            'sprint2_demo = g11_prii3.sprint2_demo:main',
            
            # Nodos en g11_prii3_move_turtlebot
            'collision_avoidance_simulation = g11_prii3_move_turtlebot.collision_avoidance_simulation:main',
            'obstacle_avoidance_simulation = g11_prii3_move_turtlebot.obstacle_avoidance_simulation:main',
            'trajectory_controller = g11_prii3_move_turtlebot.trajectory_controller:main',
            'obstacle_spawner = g11_prii3_move_turtlebot.obstacle_spawner:main',
            'draw_number_simulation = g11_prii3_move_turtlebot.draw_number_simulation:main',
            'draw_collision_avoidance = g11_prii3_move_turtlebot.draw_collision_avoidance:main',
            'draw_obstacle_avoidance = g11_prii3_move_turtlebot.draw_obstacle_avoidance:main',
            'test_turtlebot3 = g11_prii3_move_turtlebot.test_turtlebot3:main',
            'turtlebot3_teleop_keyboard = g11_prii3_move_turtlebot.turtlebot3_teleop_keyboard:main',
            'g11_prii3_move_turtlebot = g11_prii3_move_turtlebot.g11_prii3_move_turtlebot:main',
            
            # Nodos en g11_prii3_move_jetbot
            'draw_number_jetbot = g11_prii3_move_jetbot.draw_number_jetbot:main',

            # Nodos en g11_prii3_nav_turtlebot
            'waypoint_navigator = g11_prii3_nav_turtlebot.waypoint_navigator:main',
            'predefined_navigation = g11_prii3_nav_turtlebot.predefined_navigation:main',
            'autonomous_navigation = g11_prii3_nav_turtlebot.autonomous_navigation:main',
            'aruco_detector_autonomous = g11_prii3_nav_turtlebot.aruco_detector_autonomous:main',

            # Nodos en g11_prii3_nav_jetbot
            'aruco_detector = g11_prii3_nav_jetbot.aruco_detector:main',
            'aruco_bag_processor = g11_prii3_nav_jetbot.aruco_bag_processor:main',
        ],
    },
)