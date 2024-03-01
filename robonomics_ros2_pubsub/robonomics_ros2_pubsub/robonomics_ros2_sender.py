import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from robonomics_ros2_interfaces.srv import RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch
from robonomics_ros2_pubsub.utils.crypto_utils import create_account, create_launch_datalog_instance


class RobonomicsROS2Sender(Node):

    def __init__(self):
        """
        Class for creating node, that subscribe to the topic and send it data to Robonomics datalog
        """
        super().__init__('robonomics_ros2_sender')

        self.account = create_account()

        # Callback group for allowing parallel running
        sender_callback_group = MutuallyExclusiveCallbackGroup()

        # Initialize datalog and launch
        [self.datalog, self.launch, rws_status] = create_launch_datalog_instance(self.account)
        self.get_logger().info(rws_status)

        # Create service for sending datalog
        self.srv_send_datalog = self.create_service(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            self.send_datalog_callback,
            callback_group=sender_callback_group,
        )

        # Create service for sending launch
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
        try:
            self.datalog.record(request.datalog_content)
            response.result = 'Sent msg to datalog: %s' % request.datalog_content
        except Exception as e:
            response.result = 'Datalog sending failed with exception: %s' % str(e)
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
