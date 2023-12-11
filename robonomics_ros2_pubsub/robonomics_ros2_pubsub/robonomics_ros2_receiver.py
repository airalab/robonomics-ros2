import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from robonomicsinterface import Account, Datalog, Subscriber, SubEvent
from robonomicsinterface.utils import ipfs_32_bytes_to_qm_hash
from substrateinterface import KeypairType


class RobonomicsROS2Receiver(Node):

    def __init__(self):
        """
        Class for creating node, that subscribe to the topic, get address from there and
        receive the last datalog from Robonomics
        """
        super().__init__('robonomics_ros2_receiver')

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('seed', rclpy.Parameter.Type.STRING),
                ('crypto_type', rclpy.Parameter.Type.STRING),
            ]
        )

        # Get used parameters for account creation
        account_seed = self.get_parameter('seed')
        account_type = self.get_parameter('crypto_type')

        # Checking the type of account and creating it
        if account_type.value == 'ED25519':
            crypto_type = KeypairType.ED25519
        elif account_type.value == 'SR25519':
            crypto_type = KeypairType.SR25519
        else:
            crypto_type = -1
        self.account = Account(seed=account_seed.value, crypto_type=crypto_type)
        self.account_address = self.account.get_address()
        self.datalog = Datalog(self.account)
        self.get_logger().info('My address is %s' % self.account_address)

        # Create subscription to topic with sub_address
        self.sub_account_address = ''
        self.ros2_subscription = self.create_subscription(
            String,
            'robonomics/datalog_address',
            self.robonomics_address_callback,
            10)
        self.ros2_subscription

        # Publisher of datalog content for received address
        self.datalog_publisher = self.create_publisher(
            String,
            'robonomics/from/datalog',
            10
        )
        self.datalog_timer = self.create_timer(
            30,
            self.datalog_receiver_callback
        )  # 30 sec to publish datalog

        # Create subscription of launches for Robonomics node account itself
        self.robonomics_launch_subscriber = Subscriber(
            self.account,
            SubEvent.NewLaunch,
            addr=self.account_address,
            subscription_handler=self.launch_receiver_callback,
        )

        # Publisher of launch param
        self.launch_publisher = self.create_publisher(
            String,
            'robonomics/from/launch_param/ipfs',
            10
        )

    def robonomics_address_callback(self, msg):
        """
        Method for getting address of Robonomics account
        :param msg: String
        :return: None
        """
        self.sub_account_address = msg.data
        self.get_logger().info('Received launch address: %s' % self.sub_account_address)

    def datalog_receiver_callback(self):
        """
        Method for publish last datalog content from received Robonomics address
        :return: None
        """
        if self.sub_account_address != '':
            datalog_msg = String()
            datalog_msg.data = str(self.datalog.get_item(self.sub_account_address))
            self.datalog_publisher.publish(datalog_msg)

    def launch_receiver_callback(self, raw_data):
        """
        Method for publish launch param from Robonomics subscription
        :param raw_data: tuple[3]
        :return: None
        """
        launching_account_address = raw_data[0]
        launch_param_msg = String()
        launch_param_msg.data = ipfs_32_bytes_to_qm_hash(raw_data[2])
        self.get_logger().info("Getting launch from %s with param: %s" % (
            launching_account_address, launch_param_msg.data)
                               )
        self.launch_publisher.publish(launch_param_msg)

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
        self.robonomics_launch_subscriber.cancel()


def main(args=None):
    rclpy.init(args=args)

    with RobonomicsROS2Receiver() as robonomics_ros2_receiver:
        rclpy.spin(robonomics_ros2_receiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robonomics_ros2_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
