from robonomicsinterface import Account

import typing
import json
import os
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
                 sender_address: str) -> str:
    """
    Decrypt data with recipient keypair and sender address
    :param encrypted_data:      Data to decrypt
    :param recipient_keypair:   Recipient account Keypair
    :param sender_address:      Sender address
    :return:                    Decrypted data
    """

    # Get sender public key from its address
    sender_public_key = bytes.fromhex(ss58_decode(sender_address))

    # Decrypt data
    if encrypted_data[:2] == "0x":
        encrypted_data = encrypted_data[2:]
    bytes_encrypted = bytes.fromhex(encrypted_data)
    decrypted_data = recipient_keypair.decrypt_message(bytes_encrypted, sender_public_key)

    return decrypted_data.decode()


def encrypt_file(file_path: str, encrypting_account: Account, recipient_addresses: typing.List[str]) -> str:
    """
    Encrypt file with robot private key and recipient addresses
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
    with open(file_path, 'r') as file:
        data = file.read()
        encrypted_data = encrypt_data(data, encrypting_keypair, random_account.get_address())

    # Create JSON-file with encrypted random private key for each recipient addresses and encrypted_data
    file_path_crypt = file_path.split('.')[0] + '_crypt.json'
    file_crypt_data = {'encrypted_keys': {}}

    with open(file_path_crypt, 'w') as file_crypt:
        # For each recipient address encrypt random account seed
        for address in recipient_addresses:
            encrypted_key = encrypt_data(random_seed, encrypting_keypair, address)
            file_crypt_data['encrypted_keys'][address] = encrypted_key
        # Add encrypted data and dump all to JSON file
        file_crypt_data['data'] = encrypted_data
        json_object = json.dumps(file_crypt_data, indent=4)
        file_crypt.write(json_object)

    return file_path_crypt


def decrypt_file(file_path: str, decrypting_account: Account, sender_address: str) -> [str, bool]:
    """
    Decrypt file with robot private key and sender public key
    :param file_path: File to decrypt
    :param decrypting_account: An account which is going to decrypt file
    :param sender_address: An address that encrypted file
    :return: Decrypted file name and decryption status
    """

    # If decrypting account is in list of recipient addresses, then decrypt the data
    with open(file_path, 'r') as file_crypt:
        try:
            file_crypt_data = json.loads(file_crypt.read())

            # Check if encryption is needed, if not return same file
            if 'encrypted_keys' not in file_crypt_data:
                decrypt_status = False
                return [file_path, decrypt_status]

            decrypting_keypair: Keypair = decrypting_account.keypair
            decrypting_address = decrypting_account.get_address()

            if decrypting_address in file_crypt_data['encrypted_keys']:
                # Decrypting seed used for data encryption
                decrypted_random_seed = decrypt_data(file_crypt_data['encrypted_keys'][decrypting_address],
                                                     decrypting_keypair,
                                                     sender_address)

                # Decrypting data using decrypted random account
                decrypted_random_account = Account(decrypted_random_seed, crypto_type=KeypairType.ED25519)
                decrypted_data = decrypt_data(file_crypt_data['data'],
                                              decrypted_random_account.keypair,
                                              sender_address)

                # Save data to new file
                [file_name, file_ext] = os.path.splitext(file_path)
                file_dir = os.path.dirname(file_path)
                file_path_decrypt = os.path.join(file_dir,
                                                 "{file_name}_decrypt{file_ext}".format(file_name=file_name,
                                                                                        file_ext=file_ext))
                with open(file_path_decrypt, 'w') as file_decrypt:
                    file_decrypt.write(decrypted_data)

                decrypt_status = True

                return [file_path_decrypt, decrypt_status]
            else:
                raise KeyError("Error in file decryption: account is not in recipient list")

        except ValueError:
            # Return same file if it not valid JSON
            decrypt_status = False
            return [file_path, decrypt_status]