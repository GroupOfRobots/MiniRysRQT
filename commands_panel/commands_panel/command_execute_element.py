# This Python file uses the following encoding: utf-8
import paramiko
from shared.inner_communication import innerCommunication
from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem,QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json
import os
import subprocess

class CommandExecuteElementWidget(QWidget):
    def __init__(self, fileName=None, stack=None):
        super(CommandExecuteElementWidget, self).__init__()

        self.loadUi()
        self.commandButton.clicked.connect(self.commandButtonClicked)

    def loadUi(self):
        _, packagePath = get_resource('packages', 'commands_panel')
        uiFile = os.path.join(packagePath, 'share', 'commands_panel', 'resource', 'command_execute_element.ui')
        loadUi(uiFile, self)

    def commandButtonClicked(self):
        host = "192.168.102.11"
        port = 22
        username = "minirys"
        password = "minirys"

        command = "./prepare_gpio.sh"

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(host, port, username, password)

        stdin, stdout, stderr = ssh.exec_command(command)
        lines = stdout.readlines()
        # ssh.close()
        print(lines)
        # print('commandButtonClicked')
        print('commandButtonClicked ')
