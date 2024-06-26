from robonomicsinterface import Account

import typing
import requests
import json
import os
import ipfs_api

from requests.adapters import HTTPAdapter
from urllib3.util import Retry
from rclpy.node import Node
from pinatapy import PinataPy
from substrateinterface import Keypair, KeypairType
from scalecodec.utils.ss58 import ss58_decode

from robonomics_ros2_pubsub.utils.exceptions import FileNotEncryptedException


def ipfs_upload(file_path: str, pinata_api: PinataPy | None) -> str:
    """
    Function for pushing files to IPFS
    :param file_path: Path to the file
    :param pinata_api: Pinata API if exist
    :return: cid: CID of file
    """
    # Upload to IPFS via local node
    cid = ipfs_api.publish(file_path)

    # Upload to Pinata
    if pinata_api is not None:
        pinata_api.pin_file_to_ipfs(file_path, save_absolute_paths=False)

    return cid


def ipfs_download(ros2_node: Node, cid: str, file_path: str, gateway: str) -> None:
    """
    Function for download files from IPFS
    :param ros2_node: Node object for sending logs
    :param cid: File CID
    :param file_path: Full path for saving file
    :param gateway: IPFS gateway to download file
    :return: None
    """
    if gateway != '':
        ros2_node.get_logger().info('Found IPFS gateway, will try to use it for downloading')
        url = gateway + '/' + cid
        retry_num: int = 5
        try:
            # Define the retry strategy
            retry_strategy = Retry(
                total=retry_num,
                backoff_factor=3,
                status_forcelist=[429, 500, 502, 503, 504],
            )

            # Create an HTTP adapter with the retry strategy and mount it to session
            adapter = HTTPAdapter(max_retries=retry_strategy)

            # Create a new session object
            session = requests.Session()
            session.mount('http://', adapter)
            session.mount('https://', adapter)

            response = session.get(url, allow_redirects=True)

            open(file_path, 'wb').write(response.content)

            return

        except Exception as e:
            ros2_node.get_logger().error(
                'Gateway is not available after %i retries with error: %s' % (retry_num, str(e))
            )

    ros2_node.get_logger().info('Will try to use local IPFS node for downloading')
    ipfs_api.download(cid, file_path)
    return


def encrypt_data(data: typing.Union[bytes, str],
                 sender_keypair: Keypair,
                 recipient_address: str) -> str:
    """
    Encrypt data with sender keypair and recipient address
    :param data:                Data to encrypt
    :param sender_keypair:      Sender account Keypair
    :param recipient_address:   Recipient address
    :return:                    Encrypted data
    """

    # Get recipient public key from its address
    recipient_public_key = bytes.fromhex(ss58_decode(recipient_address))

    # Encrypt data
    encrypted_data = sender_keypair.encrypt_message(data, recipient_public_key)
    encrypted_data_hex = f"0x{encrypted_data.hex()}"
    return encrypted_data_hex


def decrypt_data(encrypted_data: str,
                 recipient_keypair: Keypair,
                 sender_address: str) -> bytes:
    """
    Decrypt data with recipient keypair and sender address
    :param encrypted_data:      Data to decrypt
    :param recipient_keypair:   Recipient account Keypair
    :param sender_address:      Sender address
    :return:                    Decrypted data in bytes
    """

    # Get sender public key from its address
    sender_public_key = bytes.fromhex(ss58_decode(sender_address))

    # Decrypt data
    if encrypted_data[:2] == "0x":
        encrypted_data = encrypted_data[2:]
    bytes_encrypted = bytes.fromhex(encrypted_data)
    decrypted_data = recipient_keypair.decrypt_message(bytes_encrypted, sender_public_key)

    return decrypted_data


def encrypt_file(ros2_node: Node,
                 file_path: str,
                 encrypting_account: Account,
                 recipient_addresses: typing.List[str]) -> [str, str]:
    """
    Encrypt file with robot private key and recipient addresses
    :param ros2_node:           Node object for sending logs
    :param file_path:           File to encrypt
    :param encrypting_account:  An account on whose behalf the file is encrypted
    :param recipient_addresses: List with addresses that can open a file
    :return:                    Encrypted file path
    """
    encrypting_keypair: Keypair = encrypting_account.keypair

    # An additional account is generated to encrypt data
    random_seed = Keypair.generate_mnemonic()
    random_account = Account(random_seed, crypto_type=KeypairType.ED25519)

    # Encrypt original data
    with open(file_path, 'rb') as file:
        data = file.read()
        encrypted_data = encrypt_data(data, encrypting_keypair, random_account.get_address())

    # Create JSON-file with encrypted random private key for each recipient addresses and encrypted_data
    file_path_crypt = file_path.split('.')[0] + '_crypt.json'
    file_crypt_data = {'encrypted_keys': {}, 'data': encrypted_data}

    # For each unique recipient address try to encrypt random account seed
    for address in list(set(recipient_addresses)):
        try:
            encrypted_key = encrypt_data(random_seed, encrypting_keypair, address)
            file_crypt_data['encrypted_keys'][address] = encrypted_key
        except Exception as e:
            ros2_node.get_logger().warn('Encryption is failed for address %s with error: %s' % (address, e))

    # Add encrypted data and dump all to JSON file if some encryption is done
    if file_crypt_data['encrypted_keys']:
        json_object = json.dumps(file_crypt_data, indent=4)

        with open(file_path_crypt, 'w') as file_crypt:
            file_crypt.write(json_object)

        success_encrypted_addresses = list(file_crypt_data['encrypted_keys'].keys())
        ros2_node.get_logger().info('File encryption is done for addresses: %s' % str(success_encrypted_addresses))

        return file_path_crypt
    else:
        raise FileNotEncryptedException


def decrypt_file(file_path: str, decrypting_account: Account, sender_address: str) -> [str, bool]:
    """
    Decrypt file with robot private key and sender public key
    :param file_path: File to decrypt
    :param decrypting_account: An account which is going to decrypt file
    :param sender_address: An address that encrypted file
    :return: Decrypted file name and decryption status
    """

    # If decrypting account is in list of recipient addresses, then decrypt the data
    with open(file_path) as file_crypt:
        try:
            file_crypt_data = file_crypt.read()
            file_crypt_dict = json.loads(file_crypt_data)
        except ValueError:
            # Return same file if it not valid JSON
            decrypt_status = False
            return [file_path, decrypt_status]

        # Check if encryption is needed, if not return same file
        if 'encrypted_keys' not in file_crypt_dict:
            decrypt_status = False
            return [file_path, decrypt_status]

        decrypting_keypair: Keypair = decrypting_account.keypair
        decrypting_address = decrypting_account.get_address()

        if decrypting_address in file_crypt_dict['encrypted_keys']:
            # Decrypting seed used for data encryption
            decrypted_random_seed = decrypt_data(file_crypt_dict['encrypted_keys'][decrypting_address],
                                                 decrypting_keypair,
                                                 sender_address).decode()

            # Decrypting data using decrypted random account
            decrypted_random_account = Account(decrypted_random_seed, crypto_type=KeypairType.ED25519)
            decrypted_data = decrypt_data(file_crypt_dict['data'],
                                          decrypted_random_account.keypair,
                                          sender_address)
            # Save data to new file
            [file_name, file_ext] = os.path.splitext(file_path)
            file_dir = os.path.dirname(file_path)
            file_path_decrypt = os.path.join(file_dir,
                                             "{file_name}_decrypt{file_ext}".format(file_name=file_name,
                                                                                    file_ext=file_ext))
            with open(file_path_decrypt, 'wb') as file_decrypt:
                file_decrypt.write(decrypted_data)

            decrypt_status = True

            return [file_path_decrypt, decrypt_status]
        else:
            raise KeyError("Error in file decryption: account is not in recipient list")
