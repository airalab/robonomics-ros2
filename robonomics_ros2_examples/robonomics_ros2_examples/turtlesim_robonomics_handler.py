import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from robonomics_ros2_interfaces.srv import DownloadFromIPFS

import json
import time


class TurtlesimRobonomics(Node):

    def __init__(self):
        """
        Simple class for testing Robonomics ROS 2 with turtlesim package
        """
        super().__init__('turtlesim_robonomics_handler')

        self.cmd_vel_file_name = 'turtle_cmd_vel.json'
        self.ipfs_dir = 'ipfs_files'

        # Subscription for launch params (indicated callback group that cannot being executed
        # in parallel to avoid deadlocks)
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_launch_param = self.create_subscription(
            String,
            'robonomics/launch_param',
            self.subscriber_launch_param_callback,
            10,
            callback_group=subscriber_callback_group,
        )
        self.subscriber_launch_param  # prevent unused variable warning

        # Creating service client for IPFS handler (indicated callback group that cannot being executed
        # in parallel to avoid deadlocks)
        client_callback_group = MutuallyExclusiveCallbackGroup()
        self.ipfs_handler_client = self.create_client(
            DownloadFromIPFS,
            'ipfs/download',
            callback_group=client_callback_group,
        )
        # Wait for availability of IPFS service
        while not self.ipfs_handler_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('IPFS handler service not available, waiting again...')

        # Publisher of velocities, that got from launch params
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10,
        )

    def subscriber_launch_param_callback(self, msg):
        """
        Method that downloads IPFS file and publishes it content
        :param msg: string with IPFS cid
        :return: None
        """
        cid = msg.data
        # Send request to IPFS service
        response = self.ipfs_download_request(cid, self.cmd_vel_file_name)
        self.get_logger().info(response.result)
        # Publishing received content of file to topic
        self.publish_to_cmd_vel()

    def ipfs_download_request(self, cid, file_name):
        """
        Request function to download IPFS file
        :param cid: IPFS cid
        :param file_name: name for file
        :return: result
        """
        # Preparing request
        request = DownloadFromIPFS.Request()
        request.cid = cid
        request.file_name = file_name
        # Make request and wait for its execution
        future = self.ipfs_handler_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        return future.result()

    def publish_to_cmd_vel(self):
        """
        Method for publishing all messages to cmd_vel topic
        :return: None
        """
        msg = Twist()

        file = open(get_package_share_directory('ipfs_handler') + "/" + self.ipfs_dir + "/" + self.cmd_vel_file_name)
        data = json.load(file)

        for i in range(0, len(data['linear']['x'])):
            msg.linear.x = data['linear']['x'][i]
            msg.linear.y = data['linear']['y'][i]
            msg.linear.z = data['linear']['z'][i]

            msg.angular.x = data['angular']['x'][i]
            msg.angular.y = data['angular']['y'][i]
            msg.angular.z = data['angular']['z'][i]
            self.cmd_vel_publisher.publish(msg)
            time.sleep(2)
        self.get_logger().info("Finished publishing cmd vel")
        file.close()

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
    # Creating multithreaded executor to make proper callback orchestration
    executor = MultiThreadedExecutor()

    with TurtlesimRobonomics() as turtlesim_robonomics_handler:
        try:
            executor.add_node(turtlesim_robonomics_handler)
            executor.spin()
        except KeyboardInterrupt:
            turtlesim_robonomics_handler.get_logger().warn("Killing the turtlesim_robonomics_handler...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlesim_robonomics_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
