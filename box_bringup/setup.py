from glob import glob

from setuptools import find_packages, setup

package_name = 'box_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeong',
    maintainer_email='todo@todo.todo',
    description='전체 실행 launch',
    license='TODO',
    entry_points={'console_scripts': []},
)
