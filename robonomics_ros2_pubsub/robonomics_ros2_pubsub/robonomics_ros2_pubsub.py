from typing import Any, Dict
from typing_extensions import Self

import yaml
import os
import time
import requests
import websocket

import ipfshttpclient2
import ipfs_api
from pinatapy import PinataPy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory

import robonomicsinterface as rbi
from substrateinterface import KeypairType
from substrateinterface.utils.ss58 import is_valid_ss58_address

from robonomics_ros2_interfaces.srv import (RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch,
                                            RobonomicsROS2ReceiveDatalog, RobonomicsROS2GetRWSUsers)
from robonomics_ros2_interfaces.msg import RobonomicsROS2ReceivedLaunch
from robonomics_ros2_pubsub.utils.crypto_utils import ipfs_upload, ipfs_download, encrypt_file, decrypt_file
from robonomics_ros2_pubsub.utils.ros2_utils import log_process_start, log_process_end


class RobonomicsROS2PubSub(Node):

    def __init__(self) -> None:
        """
        Class for creating ROS 2 node, that managing Robonomics functions
        """
        super().__init__('robonomics_ros2_pubsub')

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pubsub_params_path',
                 rclpy.Parameter.Type.STRING,
                 ParameterDescriptor(description='Path to config file with parameters')),
                ('ipfs_dir_path',
                 rclpy.Parameter.Type.STRING,
                 ParameterDescriptor(description='Path to directory with IPFS files')),
            ]
        )
        # Path to YAML-file with parameters
        pubsub_params_path: str = self.get_parameter('pubsub_params_path').value
        with open(pubsub_params_path, 'r') as pubsub_config_file:
            pubsub_params_dict: Dict = yaml.load(pubsub_config_file, Loader=yaml.SafeLoader)

        # Load all params
        account_seed: str = pubsub_params_dict['account_seed']
        self._remote_node_url: str = pubsub_params_dict['remote_node_url']
        self._account_type: str = pubsub_params_dict['crypto_type']
        self._rws_owner_address: str = pubsub_params_dict['rws_owner_address']
        self._ipfs_dir_path: str = pubsub_params_dict['ipfs_dir_path']
        pinata_api_key: str = pubsub_params_dict['pinata_api_key']
        pinata_api_secret_key: str = pubsub_params_dict['pinata_api_secret_key']
        self._ipfs_gateway: str = pubsub_params_dict['ipfs_gateway']

        # Checking the type of account
        if self._account_type == 'ED25519':
            crypto_type: int = KeypairType.ED25519
        elif self._account_type == 'SR25519':
            self.get_logger().warn("An account with Schnorrkel/Ristretto (SR25519) type cannot use file encryption")
            crypto_type: int = KeypairType.SR25519
        else:
            crypto_type: int = -1

        # Creating account
        try:
            self.__account = rbi.Account(
                seed=account_seed,
                remote_ws=self._remote_node_url,
                crypto_type=crypto_type)
        except ValueError as e:
            self.get_logger().error("Problem with account creation: %s" % str(e))
            raise SystemExit

        # Checking connection to parachain
        try:
            rbi.ChainUtils(remote_ws=self._remote_node_url).get_block_hash(0)
            self.get_logger().info("Connected to Robonomics via: %s" % self.__account.remote_ws)
        except (requests.exceptions.RequestException, websocket._exceptions.WebSocketException) as e:
            self.get_logger().error("Problem with connecting to Robonomics: %s" % e)
            raise SystemExit

        account_address: str = self.__account.get_address()
        self.get_logger().info('My address is %s' % account_address)

        # Trying to create valid Robonomics subscription
        try:
            rws_sub_owner = None
            self.__robonomics_subscription = rbi.RWS(self.__account)

            if self._rws_owner_address != '':
                rws_is_subscribed = self.__robonomics_subscription.is_in_sub(self._rws_owner_address, account_address)
                rws_days_left = self.__robonomics_subscription.get_days_left(addr=self._rws_owner_address)

                if rws_is_subscribed is True and rws_days_left is not False:
                    self.get_logger().info(
                        'Robonomics subscription found, transactions will be performed with the RWS module')
                    rws_sub_owner = self._rws_owner_address

                    self._srv_get_rws_users = self.create_service(
                        RobonomicsROS2GetRWSUsers,
                        'robonomics/get_rws_users',
                        self.get_rws_users_callback
                    )
            else:
                self.get_logger().info(
                    'No subscription was found, transactions will be performed per XRT')

            self.__datalog = rbi.Datalog(self.__account, rws_sub_owner=rws_sub_owner)
            self.__launch = rbi.Launch(self.__account, rws_sub_owner=rws_sub_owner)

        except Exception as e:
            self.get_logger().warn('Problem with creating RWS subscription, transactions will be performed per XRT: %s' % str(e))
            self.__datalog = rbi.Datalog(self.__account)
            self.__launch = rbi.Launch(self.__account)

        # Checking IPFS daemon is running
        try:
            with ipfshttpclient2.connect():
                self.get_logger().info("IPFS daemon is found, my IPFS ID is: " + ipfs_api.my_id())
        except ipfshttpclient2.exceptions.ConnectionError:
            self.get_logger().error('IPFS daemon is not found, shutting down...')
            raise SystemExit

        # Checking Pinata connection and API keys
        self.__pinata_api = None
        if pinata_api_key != '' and pinata_api_secret_key != '':
            self.get_logger().info('Found Pinata API keys, trying to connect...')
            # Try to connect to Pinata, retries 10 times if errors occur
            self.__pinata_api = PinataPy(pinata_api_key, pinata_api_secret_key)

            for connection_attempt in range(0, 10):
                try:
                    if 'status' and 'reason' in self.__pinata_api.user_pinned_data_total():
                        self.__pinata_api = None
                        self.get_logger().error('Pinata API keys are incorrect')
                    else:
                        self.get_logger().info('Pinning IPFS files to Pinata is activated')

                    break
                except requests.exceptions.ConnectionError:
                    self.get_logger().warn('Cannot connect to Pinata, trying again...')
                    time.sleep(5)
            else:
                self.get_logger().error('Cannot connect to Pinata after 10 retries, shutting down...')
                raise SystemExit

        # Checking IPFS directory, if not, use default
        if self._ipfs_dir_path == '' or os.path.isdir(self._ipfs_dir_path) is False:
            self._ipfs_dir_path = os.path.join(get_package_share_directory('robonomics_ros2_pubsub'), 'ipfs_files')
        else:
            self._ipfs_dir_path = os.path.abspath(self._ipfs_dir_path)
        self.get_logger().info("My IPFS files directory is: " + self._ipfs_dir_path)

        # Set IPFS path parameter
        ipfs_dir_path_param = Parameter(
            'ipfs_dir_path',
            rclpy.Parameter.Type.STRING,
            self._ipfs_dir_path)
        self.set_parameters([ipfs_dir_path_param])

        # Callback groups for allowing parallel running
        sender_callback_group = MutuallyExclusiveCallbackGroup()

        # Create service for sending datalog
        self._srv_send_datalog = self.create_service(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            self.send_datalog_callback,
            callback_group=sender_callback_group,
        )

        # Create service for sending launch
        self._srv_send_launch = self.create_service(
            RobonomicsROS2SendLaunch,
            'robonomics/send_launch',
            self.send_launch_callback,
            callback_group=sender_callback_group,
        )

        # Create service for receiving datalog from specified address
        self._srv_receive_datalog = self.create_service(
            RobonomicsROS2ReceiveDatalog,
            'robonomics/receive_datalog',
            self.receive_datalog_callback,
        )

        # Create subscription of launches for Robonomics node account itself
        self._robonomics_launch_subscriber = rbi.Subscriber(
            self.__account,
            rbi.SubEvent.NewLaunch,
            addr=account_address,
            subscription_handler=self.receive_launch_callback,
        )

        # Publisher of file name of launch parameters
        self._launch_file_publisher = self.create_publisher(
            RobonomicsROS2ReceivedLaunch,
            'robonomics/launch_file_name',
            10
        )

    @property
    def remote_node_url(self) -> str:
        """Get Robonomics node URL"""
        return self._remote_node_url

    @property
    def account_type(self) -> str:
        """Get account type"""
        return self._account_type

    @property
    def rws_owner_address(self) -> str:
        """Get RWS owner address"""
        return self._rws_owner_address

    @property
    def ipfs_dir_path(self) -> str:
        """Get IPFS directory path"""
        return self._ipfs_dir_path

    @property
    def ipfs_gateway(self) -> str:
        """Get IPFS gateway URL"""
        return self._ipfs_gateway

    def send_datalog_callback(self,
                              request: RobonomicsROS2SendDatalog.Request,
                              response: RobonomicsROS2SendDatalog.Response
                              ) -> RobonomicsROS2SendDatalog.Response:
        """
        Send datalog with specified content
        :param request: datalog file name and list with addresses for file encryption
        :param response: hash of the datalog transaction
        :return: response
        """
        log_process_start(self, 'Sending new datalog...')
        try:
            file_path = str(os.path.join(self._ipfs_dir_path, request.datalog_file_name))

            # Check if encryption is needed
            if request.encrypt_recipient_addresses:
                file_path = encrypt_file(self, file_path, self.__account, request.encrypt_recipient_addresses)

            # Upload file to IPFS and Pinata
            datalog_cid: str = ipfs_upload(file_path, self.__pinata_api)
            self.get_logger().info('IPFS CID of datalog: %s' % datalog_cid)

            response.datalog_hash = self.__datalog.record(datalog_cid)
            log_process_end(self, 'Datalog is sent with hash: %s' % response.datalog_hash)

        except Exception as e:
            response.datalog_hash = ''
            self.get_logger().error('Datalog sending failed with exception: %s' % str(e))

        return response

    def send_launch_callback(self,
                             request: RobonomicsROS2SendLaunch.Request,
                             response: RobonomicsROS2SendLaunch.Response
                             ) -> RobonomicsROS2SendLaunch.Response:
        """
        Send launch to specified address with specified param
        :param request: file name with param content, target address, encrypt status
        :param response: hash of the datalog transaction
        :return: response
        """
        log_process_start(self, 'Sending new launch to %s...' % request.target_address)
        try:
            # Check if target address is valid
            if is_valid_ss58_address(request.target_address, valid_ss58_format=32) is True:
                file_path = str(os.path.join(self._ipfs_dir_path, request.param_file_name))

                # Check if encryption is needed
                if request.encrypt_status is True:
                    file_path = encrypt_file(self, file_path, self.__account, [request.target_address])

                # Upload file to IPFS and Pinata
                param_cid: str = ipfs_upload(file_path, self.__pinata_api)
                self.get_logger().info('IPFS CID of launch param: %s' % param_cid)

                response.launch_hash = self.__launch.launch(request.target_address, param_cid)
                log_process_end(self, 'Launch is sent with hash: %s' % response.launch_hash)
            else:
                raise ValueError('Invalid target address')

        except Exception as e:
            response.launch_hash = ''
            self.get_logger().error('Launch sending failed with exception: %s' % str(e))

        return response

    def receive_datalog_callback(self,
                                 request: RobonomicsROS2ReceiveDatalog.Request,
                                 response: RobonomicsROS2ReceiveDatalog.Response,
                                 ) -> RobonomicsROS2ReceiveDatalog.Response:
        """
        Get last datalog from specified address
        :param request: address, decrypt status
        :param response: timestamp and datalog content
        :return: response
        """
        log_process_start(self, 'Receiving new datalog from %s...' % request.sender_address)
        try:
            # Check if address of datalog sender is valid
            if is_valid_ss58_address(request.sender_address, valid_ss58_format=32) is True:
                [timestamp, datalog_content] = self.__datalog.get_item(request.sender_address)
                datalog_content = str(datalog_content)

                # Check if datalog content is IPFS hash, otherwise return only its string
                if datalog_content.startswith('Qm'):
                    self.get_logger().info('Found IPFS hash in datalog: %s' % datalog_content)

                    # Check if datalog file name is set, if not then use IPFS hash as a name
                    if request.datalog_file_name == '':
                        file_path = str(os.path.join(self._ipfs_dir_path, datalog_content))
                    else:
                        file_path = str(os.path.join(self._ipfs_dir_path, request.datalog_file_name))

                    # Download from IPFS
                    ipfs_download(ros2_node=self, cid=datalog_content, file_path=file_path, gateway=self._ipfs_gateway)

                    # Decrypt file if it is needed
                    file_path = decrypt_file(self, file_path, self.__account, request.sender_address)

                    log_process_end(self, 'Datalog received successfully')
                    response.datalog_content = file_path
                else:
                    response.datalog_content = datalog_content

                # Get timestamp with nanosec
                response.timestamp.sec = int(timestamp // 1000)
                response.timestamp.nanosec = int((timestamp % 1000) * 10 ** 6)

            else:
                raise ValueError("Invalid datalog sender address")

        except Exception as e:
            response.datalog_content = ''
            response.timestamp.sec = 0
            response.timestamp.nanosec = 0
            self.get_logger().error('Datalog receiving failed with exception: %s' % str(e))

        return response

    def receive_launch_callback(self, launch_raw_data: tuple[str, str, str]) -> None:
        """
        Event handler when launch appears, which will download IPFS file from parameter and send its name to the topic.
        :param launch_raw_data: tuple with addresses and launch parameter
        :return: None
        """
        launch_sender_address: str = launch_raw_data[0]
        launch_param: str = launch_raw_data[2]

        log_process_start(self, "Getting launch from %s..." % launch_sender_address)

        try:
            # Only IPFS hashes are permitted to use in launch parameters
            ipfs_hash = rbi.utils.ipfs_32_bytes_to_qm_hash(launch_param)
            self.get_logger().info("IPFS CID in launch param: %s" % ipfs_hash)

            file_path = str(os.path.join(self._ipfs_dir_path, ipfs_hash))

            # Download from IPFS
            ipfs_download(ros2_node=self, cid=ipfs_hash, file_path=file_path, gateway=self._ipfs_gateway)

            # Decrypt file if it is needed
            file_path = decrypt_file(self, file_path, self.__account, launch_sender_address)

            # Prepare ROS msg and send it to topic
            received_launch_msg = RobonomicsROS2ReceivedLaunch()
            received_launch_msg.param_file_name = str(os.path.basename(file_path))
            received_launch_msg.launch_sender_address = str(launch_sender_address)

            self._launch_file_publisher.publish(received_launch_msg)
            log_process_end(self, 'Launch file received successfully')

        except Exception as e:
            self.get_logger().error('Launch receiving failed with exception: %s' % str(e))

    def get_rws_users_callback(
            self,
            request: RobonomicsROS2GetRWSUsers.Request,
            response: RobonomicsROS2GetRWSUsers.Response) -> RobonomicsROS2GetRWSUsers.Response:
        """
        Get all users from RWS subscription
        :param request: None
        :param response: RWS users list
        :return: response
        """
        request
        response.rws_users_list = self.__robonomics_subscription.get_devices(self._rws_owner_address)
        return response

    def __enter__(self) -> Self:
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exception_type: Any, exception_value: Any, traceback: Any) -> None:
        """
        Exit the object runtime context
        :param exception_type: exception that caused the context to be exited
        :param exception_value: exception value
        :param traceback: exception traceback
        :return: None
        """
        self._robonomics_launch_subscriber.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with RobonomicsROS2PubSub() as robonomics_ros2_pubsub:
        try:
            executor.add_node(robonomics_ros2_pubsub)
            executor.spin()
        except (KeyboardInterrupt, SystemExit):
            robonomics_ros2_pubsub.get_logger().warn("Killing the Robonomics pubsub node...")
            executor.remove_node(robonomics_ros2_pubsub)
            executor.shutdown()


if __name__ == '__main__':
    main()
