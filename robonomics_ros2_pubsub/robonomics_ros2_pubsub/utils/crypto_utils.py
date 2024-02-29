import os
from ament_index_python.packages import get_package_share_directory
import yaml
from robonomicsinterface import Account
from substrateinterface import KeypairType


def create_account() -> Account:
    """
    Create account for Robonomics parachain
    :return: Robonomics account
    """

    # Find config file with account params
    config = os.path.join(
        get_package_share_directory('robonomics_ros2_pubsub'),
        'config',
        'robonomics_params.yaml'
    )
    with open(config, 'r') as config_file:
        params_dict = yaml.load(config_file, Loader=yaml.SafeLoader)
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
