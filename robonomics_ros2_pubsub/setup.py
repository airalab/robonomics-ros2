from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robonomics_ros2_pubsub'

setup(
    name=package_name,
    version='0.0.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='berman@robonomics.network',
    description='Package for sending topic content to Robonomics datalog and getting content vise versa',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robonomics_ros2_sender = robonomics_ros2_pubsub.robonomics_ros2_sender:main',
            'robonomics_ros2_receiver = robonomics_ros2_pubsub.robonomics_ros2_receiver:main',
        ],
    },
)
