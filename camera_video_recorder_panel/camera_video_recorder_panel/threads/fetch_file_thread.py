import paramiko
from python_qt_binding.QtCore import QThread


class FetchFileThread(QThread):

    def __init__(self, filePath, remoteVideoFilePath, host, port, username, password, fileSizeLabelUI):
        super(QThread, self).__init__()
        self.password = password
        self.username = username
        self.port = port
        self.host = host
        self.filePath = filePath
        self.remoteVideoFilePath = remoteVideoFilePath
        self.fileSizeLabelUI = fileSizeLabelUI

    def run(self):
        self.fetchRecording()

    def fetchRecording(self):
        # Establish SFTP connection
        with paramiko.Transport((self.host, self.port)) as transport:
            transport.connect(username=self.username, password=self.password)
            with paramiko.SFTPClient.from_transport(transport) as sftp:
                # Check if the remote file exists
                try:
                    remote_file_size = sftp.stat(self.remoteVideoFilePath).st_size
                    self.fileSizeLabelUI.setText(str(remote_file_size))

                    sftp.get(self.remoteVideoFilePath, self.filePath)
                except FileNotFoundError:
                    print(f"Remote file not found: {self.remoteVideoFilePath}")
