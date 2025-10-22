from setuptools import setup

package_name = 'g11_prii3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],  # ¡IMPORTANTE! Lista vacía
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sprint1.launch.py', 'launch/drawer_number_gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jgomper',
    maintainer_email='jgomper@example.com',
    description='Paquete para el proyecto 3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prii3_turtlesim_node = g11_prii3.prii3_turtlesim_node:main',
            'drawer_number_gazebo = g11_prii3.drawer_number_gazebo:main',
        ],
    },
)
