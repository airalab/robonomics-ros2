from robonomicsinterface import Account

import ipfs_api

from substrateinterface import Keypair
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


def decrypt_file(file_path: str, decrypting_account: Account, sender_address: str) -> str:
    """
    Decrypt file with robot private key and sender public key
    :param file_path: File to decrypt
    :param decrypting_account: An account which is going to decrypt file
    :param sender_address: An address that encrypted file
    :return: Decrypted file name
    """
    keypair: Keypair = decrypting_account.keypair

    # Get sender public key from its address
    sender_public_key = bytes.fromhex(ss58_decode(sender_address))

    file_path_decrypt = file_path + '.decrypt'

    # Decrypting data
    with open(file_path, 'r') as file_crypt:
        encrypted_data = file_crypt.read()
        if encrypted_data[:2] == "0x":
            encrypted_data = encrypted_data[2:]
        bytes_encrypted = bytes.fromhex(encrypted_data)
        decrypted_data = keypair.decrypt_message(bytes_encrypted, sender_public_key)

    # Save data to new file
    with open(file_path_decrypt, 'w') as file_decrypt:
        file_decrypt.write(decrypted_data.decode())

    return file_path_decrypt
