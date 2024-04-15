from robonomicsinterface import Account

import ipfs_api
from substrateinterface import Keypair, KeypairType
from scalecodec.utils.ss58 import ss58_decode


def ipfs_upload(file_path: str) -> str:
    """
    Function for pushing files to IPFS
    :param file_path: Path to the file
    :return: cid: CID of file
    """
    cid = ipfs_api.publish(file_path)
    return cid


def ipfs_download(cid: str, file_path: str) -> None:
    """
    Function for download files from IPFS
    :param cid: File CID
    :param file_path: Full path for saving file
    :return: None
    """
    ipfs_api.download(cid, file_path)


def encrypt_file(file_path: str, encrypting_account: Account, recipient_address: str) -> str:
    """
    Encrypt file with robot private key and recipient public key
    :param file_path:           File to encrypt
    :param encrypting_account:  An account on whose behalf the file is encrypted
    :param recipient_address:   An address that can open a file
    :return:                    Encrypted file path
    """
    keypair: Keypair = encrypting_account.keypair

    # Get recipient public key from its address
    recipient_public_key = bytes.fromhex(ss58_decode(recipient_address))

    file_path_crypt = file_path + '.crypt'

    # Create new encrypted file
    with open(file_path, 'r') as file:
        with open(file_path_crypt, 'w') as file_crypt:
            data = file.read()
            encrypted = keypair.encrypt_message(data, recipient_public_key)
            encrypted_data = f"0x{encrypted.hex()}"
            file_crypt.write(encrypted_data)

    return file_path_crypt
#
#
# def decrypt_file(file_name_crypt: str, file_dir: str) -> None:
#     """
#     Decrypt file with robot private key and sender public key
#     :param file_name_crypt: File to decrypt
#     :param file_dir: Directory of file
#     :return: Encrypted file name
#     """
#     account = create_account()
#     keypair: Keypair = account.keypair
#
#     # Load params
#     params_dict = load_params()
#     sender_address = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['sender_address']
#
#     # Get sender public key from its address
#     sender_public_key = bytes.fromhex(ss58_decode(sender_address))
#
#     # Decrypting data
#     with open(file_dir + file_name_crypt, 'r') as file_crypt:
#         encrypted_data = file_crypt.read()
#         if encrypted_data[:2] == "0x":
#             encrypted_data = encrypted_data[2:]
#         bytes_encrypted = bytes.fromhex(encrypted_data)
#         decrypted_data = keypair.decrypt_message(bytes_encrypted, sender_public_key)
#
#     # Save data to new file
#     with open(file_dir + file_name_crypt, 'w') as file_decrypt:
#         file_decrypt.write(decrypted_data.decode())
