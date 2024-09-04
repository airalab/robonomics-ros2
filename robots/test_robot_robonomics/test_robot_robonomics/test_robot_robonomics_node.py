import rclpy
from rclpy.executors import MultiThreadedExecutor
from robonomics_ros2_robot_handler.basic_robonomics_handler import BasicRobonomicsHandler


class TestRobotRobonomicsNode(BasicRobonomicsHandler):

    def __init__(self) -> None:
        super().__init__()


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with TestRobotRobonomicsNode() as test_robot_robonomics_node:
        try:
            executor.add_node(test_robot_robonomics_node)
            executor.spin()
        except (KeyboardInterrupt, SystemExit):
            executor.remove_node(test_robot_robonomics_node)
            executor.shutdown()


if __name__ == '__main__':
    main()
