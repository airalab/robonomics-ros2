from typing_extensions import Self, Any
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from robonomicsinterface import Account, Datalog, Launch, RWS
from substrateinterface import KeypairType
from substrateinterface.utils.ss58 import is_valid_ss58_address

from robonomics_ros2_interfaces.srv import RobonomicsROS2SendDatalog


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
        remote_node_url = pubsub_params_dict['remote_node_url']
        self.account_type = pubsub_params_dict['crypto_type']
        rws_owner_address = pubsub_params_dict['rws_owner_address']

        # Check if remote node url is not specified, use default
        if remote_node_url == '':
            remote_node_url = 'wss://kusama.rpc.robonomics.network'
        self.get_logger().info("Connected to Robonomics via: %s" % remote_node_url)

        # Checking the type of account
        if self.account_type == 'ED25519':
            crypto_type = KeypairType.ED25519
        elif self.account_type == 'SR25519':
            self.get_logger().warn("An account with Schnorrkel/Ristretto (SR25519) type cannot use file encryption")
            crypto_type = KeypairType.SR25519
        else:
            crypto_type = -1

        # Creating account and show its address
        try:
            account = Account(
                seed=account_seed,
                remote_ws=remote_node_url,
                crypto_type=crypto_type)
        except ValueError:
            self.get_logger().error("A specified account type is not supported")
            raise SystemExit

        account_address = account.get_address()
        self.get_logger().info('My address is %s' % account_address)

        # Checking if subscription exists and actives for initialization of datalog and launch
        robonomics_subscription = RWS(account)
        if rws_owner_address == '':
            self.get_logger().info('The address of the subscription owner is not specified, '
                                   'transactions will be performed as usual')
            rws_status = False
        elif is_valid_ss58_address(rws_owner_address, valid_ss58_format=32) is not True:
            self.get_logger().warn('Given subscription owner address is not correct, '
                                   'transactions will be performed as usual')
            rws_status = False
        elif robonomics_subscription.get_days_left(addr=rws_owner_address) is False:
            self.get_logger().warn('No subscription was found for the owner address, '
                                   'transactions will be performed as usual')
            rws_status = False
        elif robonomics_subscription.is_in_sub(rws_owner_address, account_address) is False:
            self.get_logger().warn('Account not added to the specified subscription, '
                                   'transactions will be performed as usual')
            rws_status = False
        else:
            self.get_logger().info('Robonomics subscription found, transactions will be performed with the RWS module')
            rws_status = True

        if rws_status is True:
            self.datalog = Datalog(account, rws_sub_owner=rws_owner_address)
            self.launch = Launch(account, rws_sub_owner=rws_owner_address)
        else:
            self.datalog = Datalog(account)
            self.launch = Launch(account)

        # Callback groups for allowing parallel running
        sender_callback_group = MutuallyExclusiveCallbackGroup()

        # Create service for sending datalog
        self.srv_send_datalog = self.create_service(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            self.send_datalog_callback,
            callback_group=sender_callback_group,
        )

    def send_datalog_callback(self,
                              request: RobonomicsROS2SendDatalog.Request,
                              response: RobonomicsROS2SendDatalog.Response
                              ) -> RobonomicsROS2SendDatalog.Response:
        """
        Send datalog with specified string
        :param request: datalog string
        :param response: hash of the datalog transaction
        :return: response
        """
        try:
            response.datalog_hash = self.datalog.record(request.datalog_content)
        except Exception as e:
            response.datalog_hash = ''
            self.get_logger().error('Datalog sending failed with exception: %s' % str(e))
        return response

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
        except (KeyboardInterrupt, SystemExit):
            robonomics_ros2_pubsub.get_logger().warn("Killing the Robonomics pubsub node...")
            executor.remove_node(robonomics_ros2_pubsub)
            executor.shutdown()


if __name__ == '__main__':
    main()
