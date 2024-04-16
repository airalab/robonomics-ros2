from setuptools import find_packages, setup

package_name = 'robonomics_robot_handler'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='fingerling42@proton.me',
    description='Package with basic class for robot that use Robonomics ROS 2 Pubsub',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
