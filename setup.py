from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pcl_processing_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='pananwatanyoo@tudelft.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcl_processor = pcl_processing_ros2.pcl_processor:main'
            'pcl_publish_manual = pcl_processing_ros2.pcl_publish_manual:main'
        ],
    },
)
