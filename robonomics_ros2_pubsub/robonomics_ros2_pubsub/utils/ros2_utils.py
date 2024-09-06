from rclpy.node import Node

ANSI_COLOR_GREEN = '\x1b[32m'
ANSI_COLOR_BLUE = '\x1b[34m'
ANSI_COLOR_RESET = '\x1b[0m'


def log_process_start(ros2_node: Node, msg: str):
    """Print log in blue using ANSI codes for some process starting"""
    ros2_node.get_logger().info(ANSI_COLOR_BLUE + msg + ANSI_COLOR_RESET)


def log_process_end(ros2_node: Node, msg: str):
    """Print log in green using ANSI codes for some process ending"""
    ros2_node.get_logger().info(ANSI_COLOR_GREEN + msg + ANSI_COLOR_RESET)
