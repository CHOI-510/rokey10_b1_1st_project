from setuptools import find_packages, setup

package_name = 'box_control'

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
    description='관제 노드 + 관제 테스트용 fake 노드',
    license='TODO',
    entry_points={
        'console_scripts': [
            'control = box_control.control:main',
            'monitor_gui = box_control.monitor_gui:main',
            'fake_scan = box_control.fake_scan:main',
            'fake_judge = box_control.fake_judge:main',
            'fake_place = box_control.fake_place:main',
            'fake_inventory_check = box_control.fake_inventory_check:main',
            'test_scan_motion = box_control.test_scan_motion:main',
            'test_judge_motion = box_control.test_judge_motion:main',
            'test_place_motion = box_control.test_place_motion:main',
            'robot_test = box_control.robot_test:main',
        ],
    },
)
