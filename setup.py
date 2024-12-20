from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'isaac_cam_conveyor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf/kicker'), glob('urdf/kicker/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agillies8',
    maintainer_email='andrew.gillies@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = isaac_cam_conveyor.image_saver_node:main',
            'yolo_detection_actor = isaac_cam_conveyor.yolo_detection_actor:main',
        ],
        # 'launch': [
        #     'kicker_controller_launch = isaac_cam_conveyor.kicker_controller.launch:generate_launch_description',
        # ],
    },
)
