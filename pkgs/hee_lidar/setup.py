import glob
from setuptools import find_packages, setup

package_name = 'hee_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/heegaze_launch.py']),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*')),
        ('share/' + package_name + '/worlds', ['worlds/room_world.world']),
        ('share/' + package_name + '/rviz', ['rviz/turtlebot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hee',
    maintainer_email='shining9lee@gmail.com',
    description='TODO: Package description',
    # license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_robot = hee_lidar.move_robot:main',
            'detect_things = hee_lidar.detect_things:main',
            'control_robot = hee_lidar.control_robot:main'
        ],
    },
)
