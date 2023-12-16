import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Launch main Robonomics ROS 2 node
    robonomics_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robonomics_ros2_pubsub'), 'launch'),
            '/robonomics_ros2_pubsub_launch.py'])
    )

    # Launch turtlesim node
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )

    # Launch Robonomics handler for turtlesim
    turtlesim_robonomics = Node(
        package='robonomics_ros2_examples',
        executable='turtlesim_robonomics_handler',
    )

    # Add node to launching
    ld.add_action(robonomics_ros2)
    ld.add_action(turtlesim)
    ld.add_action(turtlesim_robonomics)
    return ld
