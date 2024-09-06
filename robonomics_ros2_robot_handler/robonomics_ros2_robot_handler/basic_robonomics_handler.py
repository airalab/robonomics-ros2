from typing_extensions import Self
from typing import Any, List, Optional

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
        param_callback_group = ReentrantCallbackGroup()

        # File name for launch parameter
        self._ipfs_dir_path: str = ''
        self.param: str = ''

        # Service for getting parameters from pubsub
        self._get_pubsub_parameter_client = self.create_client(
            GetParameters,
            'robonomics_ros2_pubsub/get_parameters',
            callback_group=param_callback_group,
        )

        # Create client for sending datalog
        self._send_datalog_client = self.create_client(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            callback_group=sender_callback_group,
        )

        # Create client for sending launch
        self._send_launch_client = self.create_client(
            RobonomicsROS2SendLaunch,
            'robonomics/send_launch',
            callback_group=sender_callback_group,
        )

        # Crete client for receiving datalog
        self._receive_datalog_client = self.create_client(
            RobonomicsROS2ReceiveDatalog,
            'robonomics/receive_datalog',
        )

        # Create subscriber for launch
        self._launch_param_subscriber = self.create_subscription(
            RobonomicsROS2ReceivedLaunch,
            'robonomics/received_launch',
            self.launch_param_subscriber_callback,
            10,
        )

        # Create client for getting RWS users
        self._get_rws_users_client = self.create_client(
            RobonomicsROS2GetRWSUsers,
            'robonomics/get_rws_users',
            callback_group=sender_callback_group
        )

        # Create timer to get IPFS dir from pubsub parameters
        self._timer_get_pubsub_params = self.create_timer(
            timer_period_sec=0.01,
            callback=self._timer_get_pubsub_params_callback,
            callback_group=param_callback_group
        )

    @property
    def ipfs_dir_path(self) -> str:
        return self._ipfs_dir_path

    def send_datalog_request(self,
                             datalog_content: str,
                             encrypt_recipient_addresses: Optional[List[str]],
                             is_file: bool = True
                             ) -> str:
        """
        Request function to send datalog
        :param  datalog_content: Just string or file name that will be uploaded to IPFS
        :param  encrypt_recipient_addresses: Addresses for file encryption, if empty, encryption will not be performed
        :param  is_file: True or False, is the datalog a file that needs to be uploaded to IPFS?
        :return: Hash of the datalog transaction
        """
        # Wait for service
        while not self._send_datalog_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Send datalog service not available, waiting again...')

        # Preparing a request
        request = RobonomicsROS2SendDatalog.Request()
        request.datalog_content = datalog_content
        request.encrypt_recipient_addresses = encrypt_recipient_addresses
        request.is_file = is_file

        # Making a request and wait for its execution
        future = self._send_datalog_client.call_async(request)

        while future.result() is None:
            pass

        datalog_hash = str(future.result().datalog_hash)
        return datalog_hash

    def send_launch_request(self,
                            param: str,
                            target_address: str,
                            is_file: bool = True,
                            encrypt_status: bool = True,
                            ) -> str:
        """
        Request function to send launch command
        :param param: Name of file that contains parameter
        :param target_address: Address to be triggered with launch
        :param  is_file: True or False, is the launch param a file that needs to be uploaded to IPFS?
        :param encrypt_status: Check if the parameter file needs to be encrypted with a target address, default is True
        :return: hash of the launch transaction
        """
        # Wait for service
        while not self._send_launch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Send launch service not available, waiting again...')

        # Preparing a request
        request = RobonomicsROS2SendLaunch.Request()
        request.param = param
        request.target_address = target_address
        request.is_file = is_file
        request.encrypt_status = encrypt_status

        # Making a request and wait for its execution
        future = self._send_launch_client.call_async(request)

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
        # Wait for service
        while not self._receive_datalog_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Receive datalog service not available, waiting again...')

        # Preparing a request
        request = RobonomicsROS2ReceiveDatalog.Request()
        request.sender_address = sender_address
        request.datalog_file_name = datalog_file_name

        # Making a request and wait for its execution
        future = self._receive_datalog_client.call_async(request)

        while future.result() is None:
            pass

        timestamp = float(future.result().timestamp.sec) + float(future.result().timestamp.nanosec) * 10 ** -9
        datalog_content = str(future.result().datalog_content)

        return [timestamp, datalog_content]

    def launch_param_subscriber_callback(self, msg: RobonomicsROS2ReceivedLaunch) -> None:
        """

        :param msg: launch sender address and file name with param
        :return: None
        """
        launch_sender_address = msg.launch_sender_address
        self.param = msg.param

    def get_rws_users_request(self) -> [str]:
        """
        Request function to get all users from RWS subscription
        :return: List with RWS users addresses
        """
        while not self._get_rws_users_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Get RWS users not available, waiting again...')

        request = RobonomicsROS2GetRWSUsers.Request()

        # Making a request and wait for its execution
        future = self._get_rws_users_client.call_async(request)

        while future.result() is None:
            pass

        return future.result().rws_users_list

    def _timer_get_pubsub_params_callback(self) -> None:

        # Just need to get parameters once
        self._timer_get_pubsub_params.cancel()

        while not self._get_pubsub_parameter_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Pubsub parameter service not available, waiting again...')

        # Make request to get pubsub parameters with IPFS path
        request = GetParameters.Request()
        request.names = ['ipfs_dir_path']
        future = self._get_pubsub_parameter_client.call_async(request)

        while future.result() is None:
            pass

        self._ipfs_dir_path = future.result().values[0].string_value

    def is_launch_ipfs(self) -> bool:
        # Make request to get pubsub parameters
        request = GetParameters.Request()
        request.names = ['launch_is_ipfs']
        future = self._get_pubsub_parameter_client.call_async(request)

        while future.result() is None:
            pass

        return future.result().values[0].bool_value


    def __enter__(self) -> Self:
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exception_type: Any, exception_value: Any, traceback: Any) -> None:
        """
        Exit the object runtime context
        :param exception_type: exception that caused the context to be exited
        :param exception_value: exception value
        :param traceback: exception traceback
        :return: None
        """
