# This Python file uses the following encoding: utf-8
import os
import re
import subprocess
import threading
from enum import Enum

import paramiko
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon, QPixmap
from python_qt_binding.QtWidgets import QWidget


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

        self.pids = []
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
            running_command = self.command.get('command')
            print("self.pids")
            print(self.pids)
            if self.pids and len(self.pids) and None:
                self.killProcessesWithPid()
                # break
            else:
                commands = re.split(r"\s*&&\s*", running_command)
                print(commands)
                try:
                    for command in commands:
                        executeCommand = f'pgrep -af "{command}"'
                        print(executeCommand)
                        stdin, stdout, stderr = self.ssh.exec_command(executeCommand)
                        # stdin.write('minirys\n')
                        stdin.flush()

                        line = stdout.readline()
                        print("line")
                        line = line.rstrip()
                        print(line.rstrip())
                        print(command)
                        print(line.endswith(str(command)))
                        if not line.endswith(command):
                            continue
                        match = re.match(r"^(\d+)", line)
                        if match:
                            pid = match.group(1)
                            print("PID:", pid)
                            pid = self.safeCast(match.group(1), int)
                            print(pid)
                            if pid is not None:
                                print("heeeeeeereeeeeee")
                                self.killProcessWithPid(pid)
                except Exception as exception:
                    print("aaaaaawwwwwwwwwww")

        else:
            print(self.process)
            self.process.kill()

        self.commandThread.join()
        self.isCommandRunning = False
        self.setRunningStatusIcon(RunStatusIcon.RUN)

    def safeCast(self, val, to_type, default=None):
        try:
            return to_type(val)
        except (ValueError, TypeError):
            print("qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq")
            return default

    def killProcessesWithPid(self):
        for pid in self.pids:
            self.killProcessWithPid(pid)

    def killProcessWithPid(self, pid):
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

            stdin, stdout, stderr = self.ssh.exec_command(command)
            stdout.channel.set_combine_stderr(True)

            self.pids = []
            line = stdout.readline()

            self.commandOutputSignal.emit([command, line, ''])

            while self.isCommandRunning:
                line = stdout.readline()
                self.commandOutputSignal.emit([command, line, ''])
                pattern = "process started with pid \[(\d+)\]"
                pattern2 = "\[(\d+)\]"
                match = re.search(pattern, line)
                match2 = re.search(pattern2, line)
                if match:
                    pid = match.group(1)
                    self.pids.append(int(pid))
                elif match2:
                    pid = match2.group(1)
                    self.pids.append(int(pid))

                if not line:
                    self.isCommandRunning = False
                    break
            output = stdout.readlines()
            errors = stderr.readlines()
            outputString = ''.join(output)
            errorsString = ''.join(errors)
            self.ssh.close()

            self.commandOutputSignal.emit([command, outputString, errorsString])
        except BaseException as exception:
            self.commandOutputSignal.emit([command, "SSH ERROR", str(exception)])
