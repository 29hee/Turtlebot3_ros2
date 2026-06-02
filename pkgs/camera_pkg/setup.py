import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'image_proc = camera_pkg.image_processor:main',
            'image_pub = camera_pkg.image_publisher:main',
            'image_yolo = camera_pkg.image_yolo:main',
            'image_canny = camera_pkg.image_canny:main',
            'image_pose = camera_pkg.img_pose:main',
            'yolo_pub = camera_pkg.yolo_publisher:main'
        ],
    },
)
