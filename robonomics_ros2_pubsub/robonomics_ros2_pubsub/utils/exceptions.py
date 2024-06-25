class FileNotEncryptedException(Exception):
    "Exception raised for errors while file encryption"
    def __init__(self):
        message = 'Not one of the presented addresses is suitable for encryption'
        super().__init__(message)
