from typing_extensions import Self, Any, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.srv import GetParameters

from robonomics_ros2_interfaces.srv import (RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch,
                                            RobonomicsROS2ReceiveDatalog, RobonomicsROS2GetRWSUsers)

from robonomics_ros2_interfaces.msg import RobonomicsROS2ReceivedLaunch


class BasicRobonomicsHandler(Node):

    def __init__(self) -> None:
        """
        Class with basic function for handling Robonomics ROS2 pubsub
        """
        super().__init__(
            node_name='robonomics_ros2_robot_handler',
        )

        sender_callback_group = MutuallyExclusiveCallbackGroup()

        # File name for launch parameter
        self.param_file_name = ''

        # Service for getting parameters from pubsub
        self.get_pubsub_parameter_client = self.create_client(
            GetParameters,
            'robonomics_ros2_pubsub/get_parameters'
        )
        while not self.get_pubsub_parameter_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Pubsub parameter service not available, waiting again...')

        # Make request to get pubsub parameters with IPFS path and RWS user list
        request = GetParameters.Request()
        request.names = ['ipfs_dir_path']
        future = self.get_pubsub_parameter_client.call_async(request)
        while future.result() is None:
            pass
        self.ipfs_dir_path = future.result().values[0].string_value

        # Create client for sending datalog
        self.send_datalog_client = self.create_client(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            callback_group=sender_callback_group,
        )
        while not self.send_datalog_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Send datalog service not available, waiting again...')

        # Create client for sending launch
        self.send_launch_client = self.create_client(
            RobonomicsROS2SendLaunch,
            'robonomics/send_launch',
            callback_group=sender_callback_group,
        )
        while not self.send_launch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Send launch service not available, waiting again...')

        # Crete client for receiving datalog
        self.receive_datalog_client = self.create_client(
            RobonomicsROS2ReceiveDatalog,
            'robonomics/receive_datalog',
        )
        while not self.receive_datalog_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Receive datalog service not available, waiting again...')

        # Create subscriber for launch
        self.launch_file_subscriber = self.create_subscription(
            RobonomicsROS2ReceivedLaunch,
            'robonomics/launch_file_name',
            self.launch_file_subscriber_callback,
            10,
        )

        # Create client for getting RWS users
        self.get_rws_users_client = self.create_client(
            RobonomicsROS2GetRWSUsers,
            'robonomics/get_rws_users',
            callback_group=sender_callback_group
        )

    def send_datalog_request(self,
                             datalog_file_name: str,
                             encrypt_recipient_addresses: Optional[List[str]]
                             ) -> str:
        """
        Request function to send datalog
        :param  datalog_file_name: File name that will be uploaded to IPFS
        :param  encrypt_recipient_addresses: Addresses for file encryption, if empty, encryption will not be performed
        :return: Hash of the datalog transaction
        """

        # Preparing a request
        request = RobonomicsROS2SendDatalog.Request()
        request.datalog_file_name = datalog_file_name
        request.encrypt_recipient_addresses = encrypt_recipient_addresses

        # Making a request and wait for its execution
        future = self.send_datalog_client.call_async(request)

        while future.result() is None:
            pass

        datalog_hash = str(future.result().datalog_hash)
        return datalog_hash

    def send_launch_request(self,
                            param_file_name: str,
                            target_address: str,
                            encrypt_status: bool = True,
                            ) -> str:
        """
        Request function to send launch command
        :param param_file_name: Name of file that contains parameter
        :param target_address: Address to be triggered with launch
        :param encrypt_status: Check if the parameter file needs to be encrypted with a target address, default is True
        :return: hash of the launch transaction
        """

        # Preparing a request
        request = RobonomicsROS2SendLaunch.Request()
        request.param_file_name = param_file_name
        request.target_address = target_address
        request.encrypt_status = encrypt_status

        # Making a request and wait for its execution
        future = self.send_launch_client.call_async(request)

        while future.result() is None:
            pass

        launch_hash = str(future.result().launch_hash)
        return launch_hash

    def receive_datalog_request(self,
                                sender_address: str,
                                datalog_file_name: str = '',
                                ) -> [float, str]:
        """
        Request function to get last datalog from address
        :param sender_address: Robonomics address from which is needed to receive the datalog
        :param datalog_file_name: name for IPFS file, default will be IPFS hash
        :return: timestamp of datalog in sec and string or file name, downloaded from IPFS
        """

        # Preparing a request
        request = RobonomicsROS2ReceiveDatalog.Request()
        request.sender_address = sender_address
        request.datalog_file_name = datalog_file_name

        # Making a request and wait for its execution
        future = self.receive_datalog_client.call_async(request)

        while future.result() is None:
            pass

        timestamp = float(future.result().timestamp.sec) + float(future.result().timestamp.nanosec) * 10 ** -9
        datalog_content = str(future.result().datalog_content)

        return [timestamp, datalog_content]

    def launch_file_subscriber_callback(self, msg: RobonomicsROS2ReceivedLaunch) -> None:
        """

        :param msg: launch sender address and file name with param
        :return: None
        """
        launch_sender_address = msg.launch_sender_address
        self.get_logger().info('Got launch from: %s' % launch_sender_address)
        self.param_file_name = msg.param_file_name

    def get_rws_users_request(self) -> [str]:
        """
        Request function to get all users from RWS subscription
        :return: List with RWS users addresses
        """
        request = RobonomicsROS2GetRWSUsers.Request()

        # Making a request and wait for its execution
        future = self.get_rws_users_client.call_async(request)

        while future.result() is None:
            pass

        return future.result().rws_users_list

    def __enter__(self) -> Self:
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """
        Exit the object runtime context
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """
