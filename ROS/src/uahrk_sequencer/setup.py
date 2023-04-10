import os
from glob import glob
from setuptools import setup

package_name = 'uahrk_sequencer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*'))
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
            'sequencer = uahrk_sequencer.SequencerNode:main',
        ],
    },
)
