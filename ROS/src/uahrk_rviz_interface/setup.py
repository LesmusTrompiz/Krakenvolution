from setuptools import setup

package_name = 'uahrk_rviz_interface'
widgets = "uahrk_rviz_interface/widgets"

setup(
    name=package_name,
    version='0.0.0',    
    packages=[package_name, widgets],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trompiz',
    maintainer_email='javier.ortiz@edu.uah.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_interface_main = uahrk_rviz_interface.App:main',
            'map_markers_main = uahrk_rviz_interface.MapMarkers:main',
            'represent_robot_main = uahrk_rviz_interface.RepresentRobot:main',
            'represent_enemy_obstacles_main = uahrk_rviz_interface.RepresentEnemyObstacles:main'
        ],
    },
)
