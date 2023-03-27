# This Python file uses the following encoding: utf-8
import logging
from enum import Enum

import paramiko
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QIcon, QPixmap
from shared.inner_communication import innerCommunication
from python_qt_binding.QtWidgets import QPushButton, QWidget, QLineEdit, QTableWidget, QTableWidgetItem, QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import threading
import time

import json
import os
import subprocess
import select



class RunStatusIcon(str, Enum):
    RUN = 'runArrow.png'
    STOP = 'stopIcon.png'


class CommandExecuteElementWidget(QWidget):
    def __init__(self, command=None, data=None, commandOutputSignal=None):
        super(CommandExecuteElementWidget, self).__init__()
        _, self.packagePath = get_resource('packages', 'commands_panel')
        self.command = command
        self.data = data
        self.commandOutputSignal = commandOutputSignal

        self.process = None

        self.loadUi()
        self.isCommandRunning = False
        self.commandLabelUI.setText(self.command.get('commandName', ''))
        self.commandButtonUI.clicked.connect(self.commandButtonClicked)

    def loadUi(self):
        uiFile = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'command_execute_element.ui')
        loadUi(uiFile, self)

    def commandButtonClicked(self):
        if self.isCommandRunning:
            self.stopCommand()

        else:
            self.setRunningStatusIcon(RunStatusIcon.STOP)
            self.commandThread = threading.Thread(target=self.runCommand)
            self.commandThread.start()

    def setRunningStatusIcon(self, iconName):
        iconPath = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'imgs', iconName)
        icon = QPixmap(iconPath)
        self.commandButtonUI.setIcon(QIcon(icon))

    def stopCommand(self):
        if self.command.get('executeViaSsh'):
            print("wwwwwwwwwwwwwwwwwwwwwwwwwwwww")
            self.stdout.channel.close()
            print("qqqqqqqqqqqqqqqqqqqqqqqqqqq")
            self.ssh.close()
        else:
            print(self.process)
            self.process.kill()

        self.commandThread.join()
        self.isCommandRunning = False
        self.setRunningStatusIcon(RunStatusIcon.RUN)

    def runCommand(self):
        self.isCommandRunning = True

        if self.command.get('executeViaSsh'):
            self.executeCommandViaSsh()
        else:
            self.executeCommandLocaly()
        self.isCommandRunning = False
        self.setRunningStatusIcon(RunStatusIcon.RUN)

    def executeCommandLocaly(self):
        command = self.command.get('command', '')

        self.process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = self.process.communicate()

        outputString = out.decode("utf-8")
        errorsString = ""
        if err is not None:
            errorsString = err.decode("utf-8")

        self.commandOutputSignal.emit([command, outputString, errorsString])

        # self.process.wait()

    def executeCommandViaSsh(self):
        sshData = self.data.get('ssh', {})

        host = sshData.get('host')
        port = sshData.get('port')
        username = sshData.get('username')
        password = sshData.get('password')

        command = self.command.get('command')
        print("command")
        print(command)

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            self.ssh.connect(host, port, username, password, timeout=5)

            transport = self.ssh.get_transport()
            channel = transport.open_session()
            channel.exec_command(command)
            # while True:
            rl, wl, xl = select.select([channel], [], [], 0.0)
            #     if len(rl) > 0:
            #         # Must be stdout
            print ("wl")
            print (wl,rl, xl)
        except BaseException as exception:
            self.commandOutputSignal.emit([command, "SSH ERROR", str(exception)])
