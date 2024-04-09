from typing_extensions import Self, Any
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robonomicsinterface import Account
from substrateinterface import KeypairType


class RobonomicsROS2PubSub(Node):

    def __init__(self) -> None:
        """
        Class for creating ROS 2 node, that managing Robonomics functions
        """
        super().__init__('robonomics_ros2_pubsub')

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pubsub_params_path', rclpy.Parameter.Type.STRING),
            ]
        )
        # Path to YAML-file with parameters
        pubsub_params_path = self.get_parameter('pubsub_params_path').value
        with open(pubsub_params_path, 'r') as pubsub_config_file:
            pubsub_params_dict = yaml.load(pubsub_config_file, Loader=yaml.SafeLoader)

        # Load all params
        account_seed = pubsub_params_dict['account_seed']
        self.account_type = pubsub_params_dict['crypto_type']

        # Checking the type of account
        if self.account_type == 'ED25519':
            crypto_type = KeypairType.ED25519
        elif self.account_type == 'SR25519':
            self.get_logger().warn("An account with Schnorrkel/Ristretto (SR25519) type cannot use file encryption")
            crypto_type = KeypairType.SR25519
        else:
            crypto_type = -1
            self.get_logger().error("A specified account type is not supported")
            rclpy.shutdown()

        # Creating account and show its address
        account = Account(seed=account_seed, crypto_type=crypto_type)
        account_address = account.get_address()
        self.get_logger().info('My address is %s' % account_address)

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


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with RobonomicsROS2PubSub() as robonomics_ros2_pubsub:
        try:
            executor.add_node(robonomics_ros2_pubsub)
            executor.spin()
        except KeyboardInterrupt:
            robonomics_ros2_pubsub.get_logger().warn("Killing the Robonomics pubsub node...")
            executor.remove_node(robonomics_ros2_pubsub)


if __name__ == '__main__':
    main()
