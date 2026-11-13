from setuptools import find_packages, setup

package_name = 'box_place'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.todo',
    description='적재 파트',
    license='TODO',
    entry_points={
        'console_scripts': [
            'place = box_place.place:main',
        ],
    },
)
