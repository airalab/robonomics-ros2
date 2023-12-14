import rclpy
from rclpy.node import Node

from robonomicsinterface import Account, Datalog, Launch
from substrateinterface import KeypairType

from robonomics_ros2_interfaces.srv import RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch


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

        # Create service for sending datalog
        self.datalog = Datalog(self.account)
        self.srv_send_datalog = self.create_service(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            self.send_datalog_callback
        )

        # Create service for sending launch
        self.launch = Launch(self.account)
        self.srv_send_launch = self.create_service(
            RobonomicsROS2SendLaunch,
            'robonomics/send_launch',
            self.send_launch_callback
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

    with RobonomicsROS2Sender() as robonomics_ros2_sender:
        try:
            rclpy.spin(robonomics_ros2_sender)
        except KeyboardInterrupt:
            robonomics_ros2_sender.get_logger().warn("Killing the sender...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robonomics_ros2_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
