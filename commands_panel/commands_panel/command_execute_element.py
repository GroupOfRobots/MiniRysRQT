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

import json
import os
import subprocess


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
            print('awwwwww')
            # stdin, stdout, stderr = self.ssh.exec_command(chr(3))
            stdin, stdout, stderr = self.ssh.exec_command('\x003' )
            # stdin.close()
            # stdin, stdout, stderr = paramiko.send(chr(3))
            output = stdout.readlines()
            errors = stderr.readlines()
            outputString = ''.join(output)
            errorsString = ''.join(errors)
            print('outputString')
            print(outputString)
            # print(errorsString)
            transport = self.ssh.get_transport()
            # transport.send(chr(3))
            # transport.set_keepalive(1)
            print('awwwwwwwwwdddddd')
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

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            self.ssh.connect(host, port, username, password, timeout=5)

            transport = self.ssh.get_transport()
            channel = transport.open_session()
            # channel.exec_command("tail -f /var/log/everything/current")
            # stdin, stdout, stderr = channel.exec_command(command)
            # ODKOMENTOWAC
            stdin, stdout, stderr = self.ssh.exec_command(command)
            output = stdout.readlines()
            errors = stderr.readlines()
            outputString = ''.join(output)
            errorsString = ''.join(errors)
            # print(self.ssh.exit_status_ready())
            self.ssh.close()

            self.commandOutputSignal.emit([command, outputString, errorsString])
        except BaseException as exception:
            self.commandOutputSignal.emit([command, "SSH ERROR", str(exception)])
