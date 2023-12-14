from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Creating node for IPFS handler
    ipfs_handler_node = Node(
        package='ipfs_handler',
        executable='ipfs_handler_node'
    )

    ld.add_action(ipfs_handler_node)
    return ld
