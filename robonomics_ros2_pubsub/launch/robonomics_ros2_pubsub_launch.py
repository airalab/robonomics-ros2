import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Preparing config file with params
    config = os.path.join(
        get_package_share_directory('robonomics_ros2_pubsub'),
        'config',
        'robonomics_params.yaml'
    )

    # Get args for IPFS handler
    with open(config, 'r') as config_file:
        params_dict = yaml.load(config_file, Loader=yaml.SafeLoader)
        ipfs_files_path = params_dict['/ipfs_handler_node']['ros__parameters']['ipfs_files_path']
        if ipfs_files_path is False:
            ipfs_launch_arguments = {}.items()
        else:
            ipfs_launch_arguments = {
                'ipfs_files_path': ipfs_files_path,
            }.items()

    # Creating node for sender with params from config
    sender_node = Node(
        package='robonomics_ros2_pubsub',
        executable='robonomics_ros2_sender',
        parameters=[config]
    )

    # Creating node for receiver
    receiver_node = Node(
        package='robonomics_ros2_pubsub',
        executable='robonomics_ros2_receiver',
        parameters=[config]
    )

    # Create IPFS handler from its package
    ipfs_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ipfs_handler'), 'launch'),
            '/ipfs_handler_launch.py']),
        launch_arguments=ipfs_launch_arguments
    )

    # Add node to launching
    ld.add_action(sender_node)
    ld.add_action(receiver_node)
    ld.add_action(ipfs_handler)
    return ld
