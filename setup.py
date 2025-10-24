from setuptools import setup
import os
from glob import glob

package_name = 'g11_prii3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},  # ¡ESTA LÍNEA ES CLAVE!
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
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
            'prii3_turtlesim_node = g11_prii3.prii3_turtlesim_node:main',
            'drawer_number_gazebo = g11_prii3.drawer_number_gazebo:main',
            'g11_prii3_move_jetbot = g11_prii3.g11_prii3_move_jetbot:main',
            'g11_prii3_move_turtlebot = g11_prii3.g11_prii3_move_turtlebot:main',
            'turtlebot3_teleop_keyboard = g11_prii3.turtlebot3_teleop_keyboard:main',
            'test_turtlebot3 = g11_prii3.test_turtlebot3:main',
            'laser_filter = g11_prii3.laser_filter:main',
            'draw_number_simulation = g11_prii3.draw_number_simulation:main',
            'collision_avoidance_simulation = g11_prii3.collision_avoidance_simulation:main',
            'obstacle_avoidance_simulation = g11_prii3.obstacle_avoidance_simulation:main',
            'obstacle_spawner = g11_prii3.obstacle_spawner:main',
            'trajectory_controller = g11_prii3.trajectory_controller:main',
        ],
    },
)