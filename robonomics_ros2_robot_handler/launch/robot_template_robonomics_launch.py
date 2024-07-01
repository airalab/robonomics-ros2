from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Launch arguments
launch_args = [
    DeclareLaunchArgument(
        name='pubsub_params_path',
        description='Path to config file with parameters',
        default_value=''
    ),
    DeclareLaunchArgument(
        name='namespace',
        description='Robot namespace',
        default_value=''
    ),
]


def generate_launch_description():
    ld = LaunchDescription(launch_args)

    # Robonomics pubsub node with param path
    robonomics_pubsub_node = Node(
        package='robonomics_ros2_pubsub',
        executable='robonomics_ros2_pubsub',
        emulate_tty=True,
        parameters=[{
            'pubsub_params_path': LaunchConfiguration('pubsub_params_path')
        }],
    )

    # Robonomics handler for your robot
    # your_robot_robonomics = Node(
    #    package='your_robot_robonomics',
    #    executable='your_robot_robonomics',
    #    emulate_tty=True,
    # )

    # Run all nodes with same namespace
    namespace_launch_action = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            robonomics_pubsub_node,
            # your_robot_robonomics
            # other_nodes
        ]
    )

    ld.add_action(namespace_launch_action)

    return ld
