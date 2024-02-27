import subprocess

from launch import SomeActionsType
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessIO
from launch.events.process import ProcessIO
from launch.substitutions import FindExecutable


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

    # Creating node for IPFS handler
    ipfs_handler_node = Node(
        package='ipfs_handler',
        executable='ipfs_handler_node'
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
