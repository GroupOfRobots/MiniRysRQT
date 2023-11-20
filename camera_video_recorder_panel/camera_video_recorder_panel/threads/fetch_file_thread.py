import time

import paramiko
from python_qt_binding.QtCore import QThread


class FetchFileThread(QThread):

    def __init__(self, filePath, fileName, host, port, username, password):
        super(QThread, self).__init__()
        self.password = password
        self.username = username
        self.port = port
        self.host = host
        self.filePath = filePath
        self.fileName = fileName

    def run(self):
        time.sleep(5)
        # self.fetchRecording()

    def fetchRecording(self):
        remote_path = '/home/minirys/ros2_rpi_camera/src/ros2_rpi_video_recorder/' + self.fileName  # Replace with the actual remote file path

        # Establish SFTP connection
        with paramiko.Transport((self.host, self.port)) as transport:
            transport.connect(username=self.username, password=self.password)

            with paramiko.SFTPClient.from_transport(transport) as sftp:
                # Check if the remote file exists
                try:
                    remote_file_size = sftp.stat(remote_path).st_size
                    print("remote_file_size", remote_file_size)

                    sftp.get(remote_path, self.filePath)
                    print(f"File downloaded to: {self.filePath}")
                except FileNotFoundError:
                    print(f"Remote file not found: {remote_path}")
