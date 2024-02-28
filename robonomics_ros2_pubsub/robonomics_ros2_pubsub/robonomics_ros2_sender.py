import os
import yaml

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from robonomicsinterface import Account, Datalog, Launch
from substrateinterface import KeypairType

from robonomics_ros2_interfaces.srv import RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch


class RobonomicsROS2Sender(Node):

    def __init__(self):
        """
        Class for creating node, that subscribe to the topic and send it data to Robonomics datalog
        """
        super().__init__('robonomics_ros2_sender')

        # Find config file with account params
        config = os.path.join(
            get_package_share_directory('robonomics_ros2_pubsub'),
            'config',
            'robonomics_params.yaml'
        )
        with open(config, 'r') as config_file:
            params_dict = yaml.load(config_file, Loader=yaml.SafeLoader)
            account_seed = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['seed']
            account_type = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['crypto_type']

        # Checking the type of account and creating it
        if account_type == 'ED25519':
            crypto_type = KeypairType.ED25519
        elif account_type == 'SR25519':
            crypto_type = KeypairType.SR25519
        else:
            crypto_type = -1
        self.account = Account(seed=account_seed, crypto_type=crypto_type)
        sender_callback_group = MutuallyExclusiveCallbackGroup()

        # Create service for sending datalog
        self.datalog = Datalog(self.account)
        self.srv_send_datalog = self.create_service(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            self.send_datalog_callback,
            callback_group=sender_callback_group,
        )

        # Create service for sending launch
        self.launch = Launch(self.account)
        self.srv_send_launch = self.create_service(
            RobonomicsROS2SendLaunch,
            'robonomics/send_launch',
            self.send_launch_callback,
            callback_group=sender_callback_group,
        )

    def send_datalog_callback(self, request, response):
        """
        Send datalog with specified string
        :param request: datalog string
        :param response: result message
        :return: response
        """
        self.datalog.record(request.datalog_content)
        response.result = 'Sent msg to datalog: %s' % request.datalog_content
        return response

    def send_launch_callback(self, request, response):
        """
        Send launch to specified address with specified param
        :param request: address, param
        :param response: result message
        :return: response
        """
        if request.param.startswith("Qm"):
            self.launch.launch(
                request.address,
                request.param
            )
            response.result = 'Sent launch to %s with param: %s' % (request.address, request.param)
            return response
        else:
            response.result = "Only IPFS hashed accepted as param for launch"
            return response

    def __enter__(self):
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exit the object runtime context
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with RobonomicsROS2Sender() as robonomics_ros2_sender:
        try:
            executor.add_node(robonomics_ros2_sender)
            executor.spin()
        except KeyboardInterrupt:
            robonomics_ros2_sender.get_logger().warn("Killing the Robonomics sender node...")
            executor.remove_node(robonomics_ros2_sender)


if __name__ == '__main__':
    main()
