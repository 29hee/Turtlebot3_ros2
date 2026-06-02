from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'tf_tutorial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hee',
    maintainer_email='shining9lee@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_simulator = tf_tutorial_pkg.odom_simulator:main',
            'tf_listener = tf_tutorial_pkg.tf_listener:main',
            'tf_yolo = tf_tutorial_pkg.tf_yolo_broadcaster:main',
        ],
    },
)
