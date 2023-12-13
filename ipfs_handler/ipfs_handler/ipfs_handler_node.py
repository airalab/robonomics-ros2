import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from robonomics_ros2_interfaces.srv import UploadToIPFS, DownloadFromIPFS

import ipfs_api


class IPFSHandlerNode(Node):

    def __init__(self):
        """
        Class for processing IPFS files
        """
        super().__init__('ipfs_handler_node')

        self.srv_upload = self.create_service(
            UploadToIPFS,
            'ipfs/upload',
            self.upload_callback
        )

        self.srv_download = self.create_service(
            DownloadFromIPFS,
            'ipfs/download',
            self.download_callback
        )

    def upload_callback(self, request, response):
        """
        Service for pushing files to IPFS, required a working IPFS daemon
        :param request: file name
        :param response: CID of file
        :return: response
        """
        try:
            response.cid = ipfs_api.publish(get_package_share_directory('ipfs_handler')+"/ipfs_files/"+request.file_name)
            return response
        except ConnectionRefusedError:
            return self.get_logger().error("Check if IPFS daemon is working")

    def download_callback(self, request, response):
        """
        Service for download files from IPFS, required a working IPFS daemon
        :param request: file's CID
        :param response: file name
        :return: response
        """
        try:
            ipfs_api.download(request.cid, get_package_share_directory('ipfs_handler')+"/ipfs_files/"+request.file_name)
            response.result = "File downloaded to " + get_package_share_directory('ipfs_handler') + "/ipfs_files/"
            return response
        except ConnectionRefusedError:
            return self.get_logger().error("Check if IPFS daemon is working")


def main(args=None):
    rclpy.init(args=args)

    ipfs_handler_node = IPFSHandlerNode()

    rclpy.spin(ipfs_handler_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ipfs_handler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
