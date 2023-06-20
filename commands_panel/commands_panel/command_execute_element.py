# This Python file uses the following encoding: utf-8
import logging
from enum import Enum
import re

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
            a = paramiko.SSHClient()
            a.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            sshData = self.data.get('ssh', {})

            host = sshData.get('host')
            port = sshData.get('port')
            username = sshData.get('username')
            password = sshData.get('password')
            a.connect(host, port, username, password, timeout=5)
            running_command = self.command.get('command')
            print(self.pids)
            for pid in self.pids:
                command = f'sudo kill -9 {pid}'
                print("lillllllll")
                print(command)
                try:
                    stdin, stdout, stderr = self.ssh.exec_command(command)
                    stdin.write('minirys\n')
                    stdin.flush()
                    print(stdin)
                    print(stdout)
                    output = stdout.readlines()
                    errors = stderr.readlines()
                    print(output)
                    print(errors)
                except Exception as exception:
                    print("aaaaaawwwwwwwwwww")
                # break


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
            self.channel = transport.open_session()

            # renamed_command="bash -c \"exec -a name1234" + command+"\""
            # print(renamed_command)

            renamed_command = command + " & echo $!"
            # renamed_command = command
            # print(renamed_command)

            stdin, stdout, stderr = self.ssh.exec_command(renamed_command)
            stdout.channel.set_combine_stderr(True)

            self.pids = []
            line = stdout.readline()
            # print(line)

            self.pid = int(line)
            self.pids.append(self.pid)

            self.commandOutputSignal.emit([command, line, ''])

            while self.isCommandRunning:
                line = stdout.readline()
                # error = stderr.readline()
                self.commandOutputSignal.emit([command, line, ''])
                # print(line)
                pattern = "process started with pid \[(\d+)\]"
                pattern2 = "\[(\d+)\]"
                match = re.search(pattern, line)
                match2 = re.search(pattern2, line)
                # print("match123456789", match, match2),
                if match:
                    pid = match.group(1)
                    # print("match", pid)
                    self.pids.append(int(pid))
                elif match2:
                    pid = match2.group(1)
                    # print("match", pid)
                    self.pids.append(int(pid))

                # print(line, '')
                if not line:
                    self.isCommandRunning = False
                    break
                # print(line, end="")
            print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            output = stdout.readlines()
            errors = stderr.readlines()
            outputString = ''.join(output)
            errorsString = ''.join(errors)
            # print(self.ssh.exit_status_ready())
            self.ssh.close()

            self.commandOutputSignal.emit([command, outputString, errorsString])
        except BaseException as exception:
            self.commandOutputSignal.emit([command, "SSH ERROR", str(exception)])
