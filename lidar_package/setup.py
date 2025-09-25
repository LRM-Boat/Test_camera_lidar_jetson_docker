from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # Явно добавляем установку package.xml
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Добавляем все launch файлы из директории launch
        ('share/' + package_name + '/launch', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='anton.belolipetskij@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_package.velodyne_launch:generate_launch_description',
        ],
    },
)

