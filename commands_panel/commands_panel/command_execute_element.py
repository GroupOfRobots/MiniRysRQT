# This Python file uses the following encoding: utf-8
import logging
from enum import Enum

import paramiko
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QIcon,QPixmap
from shared.inner_communication import innerCommunication
from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem,QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import threading

import json
import os
import subprocess

class RunStatusIcon(str, Enum):
    RUN = 'runArrow.png'
    STOP = 'stopIcon.png'

class CommandExecuteElementWidget(QWidget):
    def __init__(self, command=None):
        super(CommandExecuteElementWidget, self).__init__()
        _, self.packagePath = get_resource('packages', 'commands_panel')

        self.command = command

        self.loadUi()
        self.isCommandRunning = False
        self.commandLabelUI.setText(self.command.get('commandName', ''))
        self.commandButtonUI.clicked.connect(self.commandButtonClicked)

    def loadUi(self):
        uiFile = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'command_execute_element.ui')
        loadUi(uiFile, self)

    def commandButtonClicked(self):
        if self.isCommandRunning:
            self.ssh.close()
            self.commandThread.join()
            self.isCommandRunning = False
            self.setRunningStatusIcon(RunStatusIcon.RUN)
        else:
            self.setRunningStatusIcon(RunStatusIcon.STOP)
            self.commandThread = threading.Thread(target=self.runCommand)
            self.commandThread.start()

    def setRunningStatusIcon(self, iconName):
        iconPath = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'imgs', iconName)
        icon = QPixmap(iconPath)
        self.commandButtonUI.setIcon(QIcon(icon))

    def runCommand(self):
        self.isCommandRunning = True
        host = "192.168.102.11"
        port = 22
        username = "minirys"
        password = "minirys"

        command = self.command.command

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(host, port, username, password)

        stdin, stdout, stderr = self.ssh.exec_command(command)
        lines = stdout.readlines()
        self.ssh.close()

        print('COMMAND EXECUTION LOG:')
        print(lines)
        print()

        self.setRunningStatusIcon(RunStatusIcon.RUN)

