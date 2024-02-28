import subprocess
import os

from ament_index_python.packages import get_package_share_directory

from launch import SomeActionsType
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, DeclareLaunchArgument
from launch.event_handlers import OnProcessIO
from launch.events.process import ProcessIO
from launch.substitutions import FindExecutable, LaunchConfiguration, TextSubstitution


def process_status(process_name):
    """
    Check if process is already running
    :param process_name: Name of process
    :return: Running status
    """
    try:
        subprocess.check_output(["pgrep", process_name])
        return True
    except subprocess.CalledProcessError:
        return False


def on_matching_output(match_msg: str, action: SomeActionsType):
    """
    Create event handler that waits for an output message and then returns actions
    :param match_msg: string with msg for checking
    :param action: actions for executing
    :return: Actions
    """
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if match_msg in line:
                return action

    return on_output


def generate_launch_description():
    ld = LaunchDescription()

    # Declare path to directory with IPFS files
    ipfs_files_path_arg = DeclareLaunchArgument(
        name='ipfs_files_path',
        description='Directory for storing IPFS files',
        default_value=TextSubstitution(
            text=get_package_share_directory('ipfs_handler') + "/ipfs_files/"
        )
    )
    ld.add_action(ipfs_files_path_arg)

    # Action for starting IPFS daemon in console and print it output
    ipfs_daemon = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ipfs'),
            ' daemon'
        ]],
        shell=True,
        output='screen',
    )

    # Message from IPFS daemon if it is ready
    ipfs_ready_msg = 'Daemon is ready'

    # Creating node for IPFS handler with path to IPFS files
    ipfs_handler_node = Node(
        package='ipfs_handler',
        executable='ipfs_handler_node',
        parameters=[{
            'ipfs_files_path': LaunchConfiguration('ipfs_files_path')
        }],
    )

    # Event for starting IPFS handler node if IPFS daemon printed ready msg
    ipfs_daemon_event = RegisterEventHandler(
        OnProcessIO(
            target_action=ipfs_daemon,
            on_stdout=on_matching_output(
                match_msg=ipfs_ready_msg,
                action=[
                    ipfs_handler_node,
                ]
            )
        )
    )

    # Check if IPFS is already running and start it otherwise
    if process_status('ipfs'):
        ld.add_action(LogInfo(
            msg='Found already run IPFS daemon')
        )
        ld.add_action(ipfs_handler_node)
    else:
        ld.add_action(ipfs_daemon)
        ld.add_action(ipfs_daemon_event)

    return ld
