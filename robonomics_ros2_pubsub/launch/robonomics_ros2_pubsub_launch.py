import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # Preparing config file with params
    config = os.path.join(
        get_package_share_directory('robonomics_ros2_pubsub'),
        'config',
        'account_params.yaml'
    )

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

    # Add node to launching
    ld.add_action(sender_node)
    ld.add_action(receiver_node)
    return ld
