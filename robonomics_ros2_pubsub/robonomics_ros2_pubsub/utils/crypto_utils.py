import os
from ament_index_python.packages import get_package_share_directory
import yaml
from robonomicsinterface import Account, Datalog, Launch, RWS
from substrateinterface import Keypair, KeypairType
from scalecodec.utils.ss58 import ss58_decode

# def encrypt_file(file_name: str, file_dir: str) -> str:
#     """
#     Encrypt file with robot private key and recipient public key
#     :param file_name: File to encrypt
#     :param file_dir: Directory of file
#     :return: Encrypted file name
#     """
#     account = create_account()
#     keypair: Keypair = account.keypair
#
#     # Load params
#     params_dict = load_params()
#     recipient_address = params_dict['/robonomics_ros2_pubsub']['ros__parameters']['recipient_address']
#
#     # Get recipient public key from its address
#     recipient_public_key = bytes.fromhex(ss58_decode(recipient_address))
#
#     file_name_crypt = file_name + '.crypt'
#
#     # Create new encrypted file
#     with open(file_dir + file_name, 'r') as file:
#         with open(file_dir + file_name_crypt, 'w') as file_crypt:
#             data = file.read()
#             encrypted = keypair.encrypt_message(data, recipient_public_key)
#             encrypted_data = f"0x{encrypted.hex()}"
#             file_crypt.write(encrypted_data)
#
#     return file_name_crypt
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
