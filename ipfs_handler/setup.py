from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ipfs_handler'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'ipfs_files'), glob('ipfs_files/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='berman@robonomics.network',
    description='Package for upload / download rosbag files to InterPlanetary File System',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipfs_handler_node = ipfs_handler.ipfs_handler_node:main',
        ],
    },
)
