import os
from ament_index_python.packages import get_package_share_directory
import yaml
from robonomicsinterface import Account, Datalog, Launch, RWS
from substrateinterface import Keypair, KeypairType
from scalecodec.utils.ss58 import ss58_decode


def load_params() -> dict:
    """
    Load params from YAML config file
    :return: dictionary with params
    """
    # Find config file
    config = os.path.join(
        get_package_share_directory('robonomics_ros2_pubsub'),
        'config',
        'robonomics_params.yaml'
    )
    with open(config, 'r') as config_file:
        params_dict = yaml.load(config_file, Loader=yaml.SafeLoader)

    return params_dict


def create_launch_datalog_instance(account: Account) -> [Datalog, Launch, str]:
    """
    Initialize datalog object taking into account RWS
    :param account  Robonomics account with seed
    :return:        Datalog and Launch objects, subscription status
    """
    # Load params
    params_dict = load_params()
    rws_owner_address = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['rws_owner_address']

    # Prepare address and rws module for requests
    account_address = account.get_address()
    rws = RWS(account)

    if rws_owner_address == '':
        rws_status = 'The address of the subscription owner is not specified, transactions will be performed as usual'
        datalog = Datalog(account)
        launch = Launch(account)
    elif rws.get_days_left(addr=rws_owner_address) is False:
        rws_status = 'No subscription was found for the owner address, transactions will be performed as usual'
        datalog = Datalog(account)
        launch = Launch(account)
    elif rws.is_in_sub(rws_owner_address, account_address) is False:
        rws_status = 'Account not added to the specified subscription, transactions will be performed as usual'
        datalog = Datalog(account)
        launch = Launch(account)
    else:
        rws_status = 'Robonomics subscription found, transactions will be performed with the RWS module'
        datalog = Datalog(account, rws_sub_owner=rws_owner_address)
        launch = Launch(account, rws_sub_owner=rws_owner_address)

    return [datalog, launch, rws_status]


def create_account() -> Account:
    """
    Create account for Robonomics parachain
    :return: Robonomics account
    """
    # Load params
    params_dict = load_params()
    account_seed = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['seed']
    account_type = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['crypto_type']

    # Checking the type of account and creating it
    if account_type == 'ED25519':
        crypto_type = KeypairType.ED25519
    elif account_type == 'SR25519':
        crypto_type = KeypairType.SR25519
    else:
        crypto_type = -1

    account = Account(seed=account_seed, crypto_type=crypto_type)

    return account


def encrypt_file(file_name: str, file_dir: str) -> str:
    """
    Encrypt file with robot private key and recipient public key
    :param file_name: File to encrypt
    :param file_dir: Directory of file
    :return: Encrypted file name
    """
    account = create_account()
    keypair: Keypair = account.keypair

    # Load params
    params_dict = load_params()
    recipient_address = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['recipient_address']

    # Get recipient public key from its address
    recipient_public_key = bytes.fromhex(ss58_decode(recipient_address))

    file_name_crypt = file_name + '.crypt'

    # Create new encrypted file
    with open(file_dir + file_name, 'r') as file:
        with open(file_dir + file_name_crypt, 'w') as file_crypt:
            data = file.read()
            encrypted = keypair.encrypt_message(data, recipient_public_key)
            encrypted_data = f"0x{encrypted.hex()}"
            file_crypt.write(encrypted_data)

    return file_name_crypt


def decrypt_file(file_name_crypt: str, file_dir: str) -> None:
    """
    Decrypt file with robot private key and sender public key
    :param file_name_crypt: File to decrypt
    :param file_dir: Directory of file
    :return: Encrypted file name
    """
    account = create_account()
    keypair: Keypair = account.keypair

    # Load params
    params_dict = load_params()
    sender_address = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['sender_address']

    # Get sender public key from its address
    sender_public_key = bytes.fromhex(ss58_decode(sender_address))

    # Decrypting data
    with open(file_dir + file_name_crypt, 'r') as file_crypt:
        encrypted_data = file_crypt.read()
        if encrypted_data[:2] == "0x":
            encrypted_data = encrypted_data[2:]
        bytes_encrypted = bytes.fromhex(encrypted_data)
        decrypted_data = keypair.decrypt_message(bytes_encrypted, sender_public_key)

    # Save data to new file
    with open(file_dir + file_name_crypt, 'w') as file_decrypt:
        file_decrypt.write(decrypted_data.decode())
