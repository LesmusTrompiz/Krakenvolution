from setuptools import setup

package_name = 'uahrk_hmi_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'hmi_controller = uahrk_hmi_controller.HMIControllerNode:main',
        ],
    },
)
