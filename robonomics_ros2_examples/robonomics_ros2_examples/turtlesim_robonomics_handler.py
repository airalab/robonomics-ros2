import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from robonomics_ros2_interfaces.srv import DownloadFromIPFS, UploadToIPFS, RobonomicsROS2SendDatalog

import json
import time


class TurtlesimRobonomics(Node):

    def __init__(self):
        """
        Simple class for testing Robonomics ROS 2 with turtlesim package
        """
        super().__init__('turtlesim_robonomics_handler')

        self.cmd_vel_file_name = 'turtle_cmd_vel.json'
        self.pose_file_name = 'turtle_pose.json'
        self.ipfs_dir = 'ipfs_files'

        # Callback group for avoiding deadlocks
        launch_callback_group = MutuallyExclusiveCallbackGroup()
        datalog_callback_group = MutuallyExclusiveCallbackGroup()
        # Subscription for launch params
        self.subscriber_launch_param = self.create_subscription(
            String,
            'robonomics/launch_param',
            self.subscriber_launch_param_callback,
            10,
        )
        self.subscriber_launch_param  # prevent unused variable warning

        # Subscription for turtlesim location data
        self.turtle_pose = Pose()
        self.subscriber_pose = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.subscriber_pose_callback,
            10,
        )
        self.subscriber_pose  # prevent unused variable warning

        # Creating service clients for IPFS handler
        self.ipfs_download_client = self.create_client(
            DownloadFromIPFS,
            'ipfs/download',
            callback_group=launch_callback_group,
        )
        # Wait for availability of IPFS service
        while not self.ipfs_download_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('IPFS handler service not available, waiting again...')

        self.ipfs_upload_client = self.create_client(
            UploadToIPFS,
            'ipfs/upload',
            callback_group=datalog_callback_group,
        )
        # Wait for availability of IPFS service
        while not self.ipfs_upload_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('IPFS handler service not available, waiting again...')

        self.send_datalog_client = self.create_client(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            callback_group=datalog_callback_group,
        )

        # Publisher of velocities, that got from launch params
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10,
        )

        # Timer for sending datalogs with turtle pose
        self.timer_pose = self.create_timer(
            60,
            self.timer_pose_callback,
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

    def subscriber_pose_callback(self, msg):
        """
        Method for receiving pose msgs from turtlesim
        :param msg: msg with turtlesim/msg/Pose type
        :return: None
        """
        self.turtle_pose = msg

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
        future = self.ipfs_download_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        return future.result()

    def send_datalog_request(self, datalog_content):
        """
        Request function to send datalog
        :param datalog_content: string with data
        :return: result
        """
        request = RobonomicsROS2SendDatalog.Request()
        request.datalog_content = datalog_content
        future = self.send_datalog_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        return future.result()

    def ipfs_upload_request(self, file_name):
        """
        Request function to upload IPFS file
        :param file_name: name for file
        :return: cid
        """
        request = UploadToIPFS.Request()
        request.file_name = file_name
        future = self.ipfs_upload_client.call_async(request)
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

    def timer_pose_callback(self):
        """
        Timer callback that publish datalogs with IPFS hash each timer period
        :return: None
        """
        # Preparing file
        file = open(get_package_share_directory('ipfs_handler') + "/" + self.ipfs_dir + "/" + self.pose_file_name, 'w')
        data = {
            'x': float(self.turtle_pose.x),
            'y': float(self.turtle_pose.y),
            'theta': float(self.turtle_pose.theta),
            'linear_velocity': float(self.turtle_pose.linear_velocity),
            'angular_velocity': float(self.turtle_pose.angular_velocity),
        }
        json_object = json.dumps(data, indent=4)
        file.write(json_object)
        file.close()

        # Upload file to IPFS
        response_ipfs = self.ipfs_upload_request(self.pose_file_name)

        # Sending datalog
        response_datalog = self.send_datalog_request(response_ipfs.cid)
        self.get_logger().info(response_datalog.result)

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

    with TurtlesimRobonomics() as turtlesim_robonomics_handler:
        try:
            executor.add_node(turtlesim_robonomics_handler)
            executor.spin()
        except KeyboardInterrupt:
            turtlesim_robonomics_handler.get_logger().warn("Killing the turtlesim_robonomics_handler...")
            executor.remove_node(turtlesim_robonomics_handler)


if __name__ == '__main__':
    main()

