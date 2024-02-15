import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


from robonomics_ros2_interfaces.srv import UploadToIPFS, DownloadFromIPFS

import ipfs_api
import ipfshttpclient2

import os
import glob


class IPFSHandlerNode(Node):

    def __init__(self):
        """
        Class for processing IPFS files
        """
        super().__init__('ipfs_handler_node')

        # Check if IPFS daemon is ready
        try:
            self.get_logger().info("My IPFS ID is: " + ipfs_api.my_id())
            self.ipfs_dir = 'ipfs_files'
            self.get_logger().info("My IPFS files directory is: " +
                                   get_package_share_directory('ipfs_handler') + "/" + self.ipfs_dir + "/")
        except ipfshttpclient2.exceptions.ConnectionError:
            self.get_logger().error("Check if IPFS daemon is working")
            self.executor.remove_node(self)

        ipfs_handler_callback_group = ReentrantCallbackGroup()

        self.srv_upload = self.create_service(
            UploadToIPFS,
            'ipfs/upload',
            self.upload_callback,
            callback_group=ipfs_handler_callback_group,
        )

        self.srv_download = self.create_service(
            DownloadFromIPFS,
            'ipfs/download',
            self.download_callback,
            callback_group=ipfs_handler_callback_group,
        )

    def upload_callback(self, request, response):
        """
        Service for pushing files to IPFS, required a working IPFS daemon
        :param request: file name
        :param response: CID of file
        :return: response
        """
        response.cid = ipfs_api.publish(get_package_share_directory('ipfs_handler')
                                        + "/" + self.ipfs_dir + "/" + request.file_name)
        return response

    def download_callback(self, request, response):
        """
        Service for download files from IPFS, required a working IPFS daemon
        :param request: file's CID
        :param response: file name
        :return: response
        """
        ipfs_api.download(request.cid, get_package_share_directory('ipfs_handler') +
                          "/" + self.ipfs_dir + "/" + request.file_name)
        response.result = "File downloaded"
        return response

    def __enter__(self):
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exit the object runtime context and delete all file from IPFS dir
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """
        files_to_remove = glob.glob(get_package_share_directory('ipfs_handler') + "/" + self.ipfs_dir + "/*")
        for file in files_to_remove:
            os.remove(file)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with IPFSHandlerNode() as ipfs_handler_node:
        try:
            executor.add_node(ipfs_handler_node)
            executor.spin()
        except KeyboardInterrupt:
            ipfs_handler_node.get_logger().warn("Killing the IPFS handler...")
            executor.remove_node(ipfs_handler_node)


if __name__ == '__main__':
    main()

