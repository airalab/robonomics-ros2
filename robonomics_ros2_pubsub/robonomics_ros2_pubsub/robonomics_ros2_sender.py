import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from robonomicsinterface import Account, Datalog, Launch
from substrateinterface import KeypairType


class RobonomicsROS2Sender(Node):

    def __init__(self):
        """
        Class for creating node, that subscribe to the topic and send it data to Robonomics datalog
        """
        super().__init__('robonomics_ros2_sender')

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

        # Create subscription to topic for datalog
        self.datalog = Datalog(self.account)
        self.subscription_datalog = self.create_subscription(
            String,
            'robonomics/to/datalog',
            self.datalog_sender_callback,
            10
        )
        self.subscription_datalog

        # Create subscription to topic with sub_address
        self.sub_account_address = ''
        self.ros2_subscription = self.create_subscription(
            String,
            'robonomics/launch_address',
            self.robonomics_address_callback,
            10)
        self.ros2_subscription

        # Create subscription to topic for launch param
        self.launch = Launch(self.account)
        self.subscription_launch_param = self.create_subscription(
            String,
            'robonomics/to/launch_param/ipfs',
            self.launch_sender_callback,
            10
        )
        self.subscription_launch_param

    def robonomics_address_callback(self, msg):
        """
        Method for getting address of Robonomics account
        :param msg: String
        :return: None
        """
        self.sub_account_address = msg.data
        self.get_logger().info('Received datalog address: %s' % self.sub_account_address)

    def datalog_sender_callback(self, msg):
        """
        Method that happened if the msg appears in topic for datalog
        :param msg: String with datalog
        :return: None
        """
        self.datalog.record(msg.data)
        self.get_logger().info('Sent msg to datalog: %s' % msg.data)

    def launch_sender_callback(self, msg):
        """
        Method that happened if the msg appears in topic for launch param
        :param msg: String with launch param
        :return: None
        """
        msg_param = msg.data
        if msg_param.startswith("Qm"):
            self.launch.launch(
                self.sub_account_address,
                msg_param
            )
            self.get_logger().info('Sent launch to %s with param: %s' % (self.sub_account_address, msg_param))
        else:
            self.get_logger().warn("Only IPFS hashed accepted as param for launch")


def main(args=None):
    rclpy.init(args=args)

    robonomics_ros2_sender = RobonomicsROS2Sender()

    rclpy.spin(robonomics_ros2_sender)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robonomics_ros2_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
