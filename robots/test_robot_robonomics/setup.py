from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_robot_robonomics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='fingerling42@proton.me',
    description='Generic node for using basic Robonomics handler',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_robot_robonomics_node = test_robot_robonomics.test_robot_robonomics_node:main',
        ],
    },
)
