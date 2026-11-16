from glob import glob

from setuptools import find_packages, setup

package_name = 'box_place'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.json')),   # place_config.json
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.todo',
    description='적재 파트',
    license='TODO',
    entry_points={
        'console_scripts': [
            'place_template = box_place.place_template:main',
            'place_main = box_place.place_main:main',          # 실제 로봇 적재
        ],
    },
)
