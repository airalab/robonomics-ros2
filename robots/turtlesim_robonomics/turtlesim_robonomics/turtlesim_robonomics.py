import rclpy
from rclpy.executors import MultiThreadedExecutor
from robonomics_ros2_interfaces.msg import RobonomicsROS2ReceivedLaunch

from robonomics_ros2_robot_handler.basic_robonomics_handler import BasicRobonomicsHandler
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import json
import os
import time


class TurtlesimRobonomics(BasicRobonomicsHandler):

    def __init__(self) -> None:
        super().__init__()

        self.pose_file_name = 'turtle_pose.json'
        self.param = 'turtle_cmd_vel.json'

        # Subscription for turtlesim location data
        self.turtle_pose = Pose()
        self.subscriber_pose = self.create_subscription(
            Pose,
            'turtle/pose',
            self.subscriber_pose_callback,
            10,
        )
        self.subscriber_pose  # prevent unused variable warning

        # Timer for sending datalogs with turtle pose
        self.timer_pose = self.create_timer(
            120,  # period in sec
            self.timer_pose_callback,
        )

        # Publisher of velocities, that got from launch params
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'turtle/cmd_vel',
            10,
        )

    def launch_param_subscriber_callback(self, msg: RobonomicsROS2ReceivedLaunch) -> None:
        super().launch_param_subscriber_callback(msg)
        if self.is_launch_ipfs():
            self.publish_to_cmd_vel()

    def subscriber_pose_callback(self, msg: Pose) -> None:
        """
        Method for receiving pose msgs from turtlesim
        :param msg: msg with turtlesim/msg/Pose type
        :return: None
        """
        self.turtle_pose = msg

    def timer_pose_callback(self) -> None:
        """
        Timer callback that publish datalogs with IPFS hash each timer period
        :return: None
        """
        # Preparing file
        file = open(os.path.join(self.ipfs_dir_path, self.pose_file_name), 'w')
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

        rws_users_list = self.get_rws_users_request()

        self.send_datalog_request(self.pose_file_name, encrypt_recipient_addresses=rws_users_list)

    def publish_to_cmd_vel(self) -> None:
        """
        Method for publishing all messages to cmd_vel topic
        :return: None
        """
        msg = Twist()

        file = open(os.path.join(self.ipfs_dir_path, self.param), 'r')
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


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with TurtlesimRobonomics() as turtlesim_robonomics:
        try:
            executor.add_node(turtlesim_robonomics)
            executor.spin()
        except (KeyboardInterrupt, SystemExit):
            turtlesim_robonomics.get_logger().warn("Killing the Turtlesim Robonomics node...")
            executor.remove_node(turtlesim_robonomics)
            executor.shutdown()


if __name__ == '__main__':
    main()
