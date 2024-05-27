from typing_extensions import Self, Any
import yaml
import os

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

from robonomicsinterface import Account, Datalog, Launch, RWS, Subscriber, SubEvent
from robonomicsinterface.utils import ipfs_32_bytes_to_qm_hash
from substrateinterface import KeypairType
from substrateinterface.utils.ss58 import is_valid_ss58_address

from robonomics_ros2_interfaces.srv import (RobonomicsROS2SendDatalog, RobonomicsROS2SendLaunch,
                                            RobonomicsROS2ReceiveDatalog, RobonomicsROS2GetRWSUsers)
from robonomics_ros2_interfaces.msg import RobonomicsROS2ReceivedLaunch

from robonomics_ros2_pubsub.utils.crypto_utils import ipfs_upload, ipfs_download, encrypt_file, decrypt_file


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
        pubsub_params_path = self.get_parameter('pubsub_params_path').value
        with open(pubsub_params_path, 'r') as pubsub_config_file:
            pubsub_params_dict = yaml.load(pubsub_config_file, Loader=yaml.SafeLoader)

        # Load all params
        account_seed = pubsub_params_dict['account_seed']
        remote_node_url = pubsub_params_dict['remote_node_url']
        self.account_type = pubsub_params_dict['crypto_type']
        self.rws_owner_address = pubsub_params_dict['rws_owner_address']
        self.ipfs_dir_path = pubsub_params_dict['ipfs_dir_path']
        pinata_api_key = pubsub_params_dict['pinata_api_key']
        pinata_api_secret_key = pubsub_params_dict['pinata_api_secret_key']

        # Check if remote node url is not specified, use default
        if remote_node_url == '':
            remote_node_url = 'wss://kusama.rpc.robonomics.network'
        self.get_logger().info("Connected to Robonomics via: %s" % remote_node_url)

        # Checking the type of account
        if self.account_type == 'ED25519':
            crypto_type = KeypairType.ED25519
        elif self.account_type == 'SR25519':
            self.get_logger().warn("An account with Schnorrkel/Ristretto (SR25519) type cannot use file encryption")
            crypto_type = KeypairType.SR25519
        else:
            crypto_type = -1

        # Creating account and show its address
        try:
            self.account = Account(
                seed=account_seed,
                remote_ws=remote_node_url,
                crypto_type=crypto_type)
        except ValueError:
            self.get_logger().error("A specified account type is not supported")
            raise SystemExit

        account_address = self.account.get_address()
        self.get_logger().info('My address is %s' % account_address)

        # Checking if subscription exists and actives for initialization of datalog and launch
        self.robonomics_subscription = RWS(self.account)
        if self.rws_owner_address == '':
            self.get_logger().info('The address of the subscription owner is not specified, '
                                   'transactions will be performed as usual')
            rws_status = False
        elif is_valid_ss58_address(self.rws_owner_address, valid_ss58_format=32) is not True:
            self.get_logger().warn('Given subscription owner address is not correct, '
                                   'transactions will be performed as usual')
            rws_status = False
        elif self.robonomics_subscription.get_days_left(addr=self.rws_owner_address) is False:
            self.get_logger().warn('No subscription was found for the owner address, '
                                   'transactions will be performed as usual')
            rws_status = False
        elif self.robonomics_subscription.is_in_sub(self.rws_owner_address, account_address) is False:
            self.get_logger().warn('Account not added to the specified subscription, '
                                   'transactions will be performed as usual')
            rws_status = False
        else:
            self.get_logger().info('Robonomics subscription found, transactions will be performed with the RWS module')
            rws_status = True

        if rws_status is True:
            self.datalog = Datalog(self.account, rws_sub_owner=self.rws_owner_address)
            self.launch = Launch(self.account, rws_sub_owner=self.rws_owner_address)

            self.srv_get_rws_users = self.create_service(
                RobonomicsROS2GetRWSUsers,
                'robonomics/get_rws_users',
                self.get_rws_users_callback
            )
        else:
            self.datalog = Datalog(self.account)
            self.launch = Launch(self.account)

        # Checking IPFS daemon
        try:
            with ipfshttpclient2.connect():
                self.get_logger().info('IPFS daemon is found')
        except ipfshttpclient2.exceptions.ConnectionError:
            self.get_logger().error('IPFS daemon is not found, check if it is working')
            raise SystemExit
        self.get_logger().info("My IPFS ID is: " + ipfs_api.my_id())

        # Checking IPFS directory, if not, use default
        if self.ipfs_dir_path == '' or os.path.isdir(self.ipfs_dir_path) is False:
            self.ipfs_dir_path = os.path.join(get_package_share_directory('robonomics_ros2_pubsub'), 'ipfs_files')
        else:
            self.ipfs_dir_path = os.path.abspath(self.ipfs_dir_path)
        self.get_logger().info("My IPFS files directory is: " + self.ipfs_dir_path)

        # Set IPFS path parameter
        ipfs_dir_path_param = Parameter(
            'ipfs_dir_path',
            rclpy.Parameter.Type.STRING,
            self.ipfs_dir_path)
        self.set_parameters([ipfs_dir_path_param])

        # Set Pinata API if keys
        self.pinata_api = None
        try:
            if pinata_api_key != '' and pinata_api_secret_key != '':
                self.pinata_api = PinataPy(pinata_api_key, pinata_api_secret_key)
                self.get_logger().info('Pinning IPFS files to Pinata is activated')
        except Exception as e:
            self.get_logger().error('Pinata API keys are incorrect: %s' % e)

        # Callback groups for allowing parallel running
        sender_callback_group = MutuallyExclusiveCallbackGroup()

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

        # Create service for receiving datalog from specified address
        self.srv_receive_datalog = self.create_service(
            RobonomicsROS2ReceiveDatalog,
            'robonomics/receive_datalog',
            self.receive_datalog_callback,
        )

        # Create subscription of launches for Robonomics node account itself
        self.robonomics_launch_subscriber = Subscriber(
            self.account,
            SubEvent.NewLaunch,
            addr=account_address,
            subscription_handler=self.receive_launch_callback,
        )

        # Publisher of file name of launch parameters
        self.launch_file_publisher = self.create_publisher(
            RobonomicsROS2ReceivedLaunch,
            'robonomics/launch_file_name',
            10
        )

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
        try:
            file_path = str(os.path.join(self.ipfs_dir_path, request.datalog_file_name))

            # Check if encryption is needed
            if request.encrypt_recipient_addresses:
                [file_path, status] = encrypt_file(file_path, self.account, request.encrypt_recipient_addresses)
                self.get_logger().info('Encrypting file for target address is finished with status: %s' % status)

            # Upload file to IPFS
            datalog_cid = ipfs_upload(file_path)

            # Upload to Pinata
            if self.pinata_api is not None:
                self.pinata_api.pin_file_to_ipfs(file_path, save_absolute_paths=False)

            self.get_logger().info('Sending datalog with IPFS CID: %s' % datalog_cid)
            response.datalog_hash = self.datalog.record(datalog_cid)

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
        try:
            # Check if target address is valid
            if is_valid_ss58_address(request.target_address, valid_ss58_format=32) is True:
                file_path = str(os.path.join(self.ipfs_dir_path, request.param_file_name))

                # Check if encryption is needed and recipient address is valid
                if request.encrypt_status is True:
                    [file_path, status] = encrypt_file(file_path, self.account, [request.target_address])
                    self.get_logger().info('Encrypting file for target address is finished with status: %s' % status)

                # Upload file to IPFS
                param_cid = ipfs_upload(file_path)

                # Upload to Pinata
                if self.pinata_api is not None:
                    self.pinata_api.pin_file_to_ipfs(file_path, save_absolute_paths=False)

                self.get_logger().info('Sending launch to %s with parameter: %s' % (request.target_address, param_cid))
                response.launch_hash = self.launch.launch(request.target_address, param_cid)
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
        try:
            # Check if address of datalog sender is valid
            if is_valid_ss58_address(request.sender_address, valid_ss58_format=32) is True:
                [timestamp, datalog_content] = self.datalog.get_item(request.sender_address)
                datalog_content = str(datalog_content)

                # Check if datalog content is IPFS hash
                if datalog_content.startswith('Qm'):
                    self.get_logger().info(
                        'Receiving datalog from %s with IPFS hash: %s' % (request.sender_address, datalog_content)
                    )

                    # Check if datalog file name is set, if not then use IPFS hash as a name
                    if request.datalog_file_name == '':
                        file_path = str(os.path.join(self.ipfs_dir_path, datalog_content))
                    else:
                        file_path = str(os.path.join(self.ipfs_dir_path, request.datalog_file_name))

                    # Download from IPFS
                    ipfs_download(cid=datalog_content, file_path=file_path)

                    # Decrypt file if it is needed
                    [file_path, decrypt_status] = decrypt_file(file_path, self.account, request.sender_address)
                    if decrypt_status is True:
                        self.get_logger().info('Datalog file is decrypted')

                    response.datalog_content = file_path

                    # Get timestamp with nanosec
                    response.timestamp.sec = int(timestamp // 1000)
                    response.timestamp.nanosec = int((timestamp % 1000) * 10 ** 6)

                else:
                    # Else if datalog content is just string
                    raise ValueError("Datalog is not IPFS hash")

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
        launch_sender_address = launch_raw_data[0]
        launch_param = launch_raw_data[2]

        try:
            # Only IPFS hashes are permitted to use in launch parameters
            ipfs_hash = ipfs_32_bytes_to_qm_hash(launch_param)
            self.get_logger().info("Getting launch from %s with param: %s" % (launch_sender_address, ipfs_hash))

            file_path = str(os.path.join(self.ipfs_dir_path, ipfs_hash))

            # Download from IPFS
            ipfs_download(cid=ipfs_hash, file_path=file_path)

            # Decrypt file if it is needed
            [file_path, decrypt_status] = decrypt_file(file_path, self.account, launch_sender_address)
            if decrypt_status is True:
                self.get_logger().info('File with launch parameters is decrypted')

            # Prepare ROS msg and send it to topic
            received_launch_msg = RobonomicsROS2ReceivedLaunch()
            received_launch_msg.param_file_name = str(os.path.basename(file_path))
            received_launch_msg.launch_sender_address = str(launch_sender_address)

            self.launch_file_publisher.publish(received_launch_msg)

        except Exception as e:
            self.get_logger().error('Launch receiving failed with exception: %s' % str(e))

    def get_rws_users_callback(
            self,
            request: RobonomicsROS2GetRWSUsers.Request,
            response: RobonomicsROS2GetRWSUsers.Response,
        ) -> RobonomicsROS2GetRWSUsers.Response:
        """
        Get all users from RWS subscription
        :param request: None
        :param response: RWS users list
        :return: response
        """
        response.rws_users_list = self.robonomics_subscription.get_devices(self.rws_owner_address)
        return response

    def __enter__(self) -> Self:
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """
        Exit the object runtime context
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """
        self.robonomics_launch_subscriber.cancel()


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
