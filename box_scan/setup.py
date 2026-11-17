from setuptools import find_packages, setup

package_name = 'box_scan'

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
    description='탐지 파트',
    license='TODO',
    entry_points={
        'console_scripts': [
            'scan_template = box_scan.scan_template:main',
            'scan_main = box_scan.scan_main:main',            # 실제 탐지 (scan_algorithm.py)
            'scan_original = box_scan.scan_original:main',    # 원본 단독 실행 (비교·비상용, 관제와 동시 실행 금지)
        ],
    },
)
