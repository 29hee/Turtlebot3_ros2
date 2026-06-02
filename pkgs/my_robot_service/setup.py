from setuptools import find_packages, setup

package_name = 'my_robot_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'led_server = my_robot_service.led_server:main',
            'led_client = my_robot_service.led_client:main',
            'add_server = my_robot_service.add_server:main',
            'add_client = my_robot_service.add_client:main',
        ],
    },
)
