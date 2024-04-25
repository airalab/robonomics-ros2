from typing_extensions import Self, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.srv import GetParameters

from robonomics_ros2_interfaces.srv import (RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch,
                                            RobonomicsROS2ReceiveDatalog)

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

        # Service for getting IPFS path parameter
        self.get_ipfs_path_parameter_client = self.create_client(
            GetParameters,
            'robonomics_ros2_pubsub/get_parameters'
        )
        while not self.get_ipfs_path_parameter_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Pubsub parameter service not available, waiting again...')

        # Make request to get IPFS path class variable
        request = GetParameters.Request()
        request.names = ["ipfs_dir_path"]
        future = self.get_ipfs_path_parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)  # rclpy instead of self.executor, because constructor
        # has not yet created an executor
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

    def send_datalog_request(self,
                             datalog_content: str,
                             ipfs_file_status: bool = True,
                             encrypt_status: bool = False
                             ) -> str:
        """
        Request function to send datalog
        :param datalog_content: string or file name, that will be uploaded to IPFS
        :param ipfs_file_status: status if datalog is needed to sent as IPFS file, default is True
        :param encrypt_status: status if IPFS file should be encrypted, default is False
        :return: hash of the datalog transaction
        """

        # Preparing a request
        request = RobonomicsROS2SendDatalog.Request()
        request.datalog_content = datalog_content
        request.ipfs_file_status = ipfs_file_status
        request.encrypt_status = encrypt_status

        # Making a request and wait for its execution
        future = self.send_datalog_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        datalog_hash = str(future.result().datalog_hash)
        return datalog_hash

    def send_launch_request(self,
                            param_file_name: str,
                            target_address: str
                            ) -> str:
        """
        Request function to send launch command
        :param param_file_name: name of file that contains parameter
        :param target_address: address to be triggered with launch
        :return: hash of the launch transaction
        """

        # Preparing a request
        request = RobonomicsROS2SendLaunch.Request()
        request.param_file_name = param_file_name
        request.target_address = target_address

        # Making a request and wait for its execution
        future = self.send_launch_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        launch_hash = str(future.result().launch_hash)
        return launch_hash

    def receive_datalog_request(self,
                                sender_address: str,
                                datalog_file_name: str = '',
                                decrypt_status: bool = False,
                                ) -> [float, str]:
        """
        Request function to get last datalog from address
        :param sender_address: Robonomics address from which is needed to receive the datalog
        :param datalog_file_name: name for IPFS file, default will be IPFS hash
        :param decrypt_status: status if IPFS file should be decrypted, default is False
        :return: timestamp of datalog in sec and string or file name, downloaded from IPFS
        """

        # Preparing a request
        request = RobonomicsROS2ReceiveDatalog.Request()
        request.sender_address = sender_address
        request.datalog_file_name = datalog_file_name
        request.decrypt_status = decrypt_status

        # Making a request and wait for its execution
        future = self.receive_datalog_client.call_async(request)
        self.executor.spin_until_future_complete(future)

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