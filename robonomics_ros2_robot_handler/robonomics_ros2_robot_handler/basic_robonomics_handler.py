from typing_extensions import Self, Any

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from robonomics_ros2_interfaces.srv import RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch


class BasicRobonomicsHandler(Node):

    def __init__(self) -> None:
        """
        Class with basic function for handling Robonomics ROS2 pubsub
        """
        super().__init__('robonomics_ros2_robot_handler')

        sender_callback_group = MutuallyExclusiveCallbackGroup()

        # Create client for sending datalog
        self.send_datalog_client = self.create_client(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            callback_group=sender_callback_group,
        )
        while not self.send_datalog_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Send datalog service not available, waiting again...')

        self.send_launch_client = self.create_client(
            RobonomicsROS2SendLaunch,
            'robonomics/send_launch',
            callback_group=sender_callback_group,
        )
        while not self.send_launch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Send launch service not available, waiting again...')

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