from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robonomics_ros2_pubsub'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ipfs_files'), glob(os.path.join('ipfs_files', package_name))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='fingerling42@proton.me',
    description='Package for using Robonomics functions from ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robonomics_ros2_pubsub = robonomics_ros2_pubsub.robonomics_ros2_pubsub:main',
        ],
    },
)
