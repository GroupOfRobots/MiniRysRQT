# This Python file uses the following encoding: utf-8
import os
import re
import signal
import subprocess
import sys
import threading
from enum import Enum

import paramiko
from ament_index_python import get_resource
from python_qt_binding.QtGui import QIcon, QPixmap
from python_qt_binding.QtWidgets import QWidget
from shared.enums import PackageNameEnum
from shared.utils.load_ui_file import loadUiFile


class RunStatusIcon(str, Enum):
    TEST = 'hourglass.png'
    RUNNING = 'runningIcon.png'
    RUN = 'runArrow.png'
    STOP = 'stopIcon.png'


class CommandExecuteElementWidget(QWidget):
    def __init__(self, command=None, data=None, commandOutputSignal=None):
        super(CommandExecuteElementWidget, self).__init__()
        _, self.packagePath = get_resource('packages', 'commands_panel')

        self.command = command
        self.data = data
        self.commandOutputSignal = commandOutputSignal

        self.processesPids = []
        self.process = None

        loadUiFile(self, PackageNameEnum.CommandsPanel, 'command_execute_element.ui')

        self.isCommandRunning = False
        self.commandLabelUI.setText(self.command.get('commandName', ''))
        self.commandButtonUI.clicked.connect(self.commandButtonClicked)

    def commandButtonClicked(self):
        if self.isCommandRunning:
            self.stopCommand()
        else:
            self.setRunningStatusIcon(RunStatusIcon.RUNNING)
            self.commandThread = threading.Thread(target=self.runCommand)
            self.commandThread.start()

    def setRunningStatusIcon(self, iconName):
        iconPath = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'imgs', iconName)
        icon = QPixmap(iconPath)
        self.commandButtonUI.setIcon(QIcon(icon))

    def stopCommand(self):
        if self.isSshCommand():
            running_command = self.command.get('command')
            print("self.pids")
            print(self.processesPids)
            if self.processesPids and len(self.processesPids) and None:
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
            # print("self.pr22222ocess123456")
            # # self.process.wait(timeout=0.1)
            try:
                self.process.wait(timeout=0.1)
            except subprocess.TimeoutExpired:
                print('Terminating the whole process group...', file=sys.stderr)
                print(self.process.pid)
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

            # self.process.send_signal(signal.SIGINT)
            # #
            # self.process.terminate()
            # self.process.kill()

        self.commandThread.join()
        self.commandExecutionFinished()

    def safeCast(self, val, to_type, default=None):
        try:
            return to_type(val)
        except (ValueError, TypeError):
            print("qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq")
            return default

    def killProcessesWithPid(self):
        for pid in self.processesPids:
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
        if self.isSshCommand():
            self.executeCommandViaSsh()
        else:
            self.executeCommandLocaly()
        self.commandExecutionFinished()

    def isSshCommand(self):
        return self.command.get('executeViaSsh')

    def commandExecutionFinished(self):
        self.isCommandRunning = False
        self.setRunningStatusIcon(RunStatusIcon.RUN)

    def executeCommandLocaly(self):
        command = self.command.get('command', '')
        commandName = self.command.get('commandName')

        self.process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, stdin=subprocess.PIPE, start_new_session=True)

        print("PID", self.process.pid)

        out, err = self.process.communicate()

        outputString = out.decode("utf-8")
        errorsString = ""
        if err is not None:
            errorsString = err.decode("utf-8")

        self.emitCommandOutput(commandName, command, outputString, errorsString, True)

    def executeCommandViaSsh(self):
        sshData = self.data.get('ssh', {})

        host = sshData.get('host')
        port = sshData.get('port')
        username = sshData.get('username')
        password = sshData.get('password')

        command = self.command.get('command')
        commandName = self.command.get('commandName', 'command')

        self.emitCommandOutput(commandName, command, '', '', True)

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            self.ssh.connect(host, port, username, password, timeout=5)

            transport = self.ssh.get_transport()
            self.channel = transport.open_session()

            stdin, stdout, stderr = self.ssh.exec_command(command)
            stdout.channel.set_combine_stderr(True)

            self.processesPids = []
            line = self.getLineWithoutNewLine(stdout)
            self.emitCommandOutput(commandName, command, line, '', False)

            while self.isCommandRunning:
                line = self.getLineWithoutNewLine(stdout)

                self.emitCommandOutput(commandName, command, line, '', False)
                pattern = "process started with pid \[(\d+)\]"
                pattern2 = "\[(\d+)\]"
                match = re.search(pattern, line)
                match2 = re.search(pattern2, line)
                if match:
                    pid = match.group(1)
                    self.processesPids.append(int(pid))
                elif match2:
                    pid = match2.group(1)
                    self.processesPids.append(int(pid))

                if not line:
                    self.isCommandRunning = False
                    break
            output = stdout.readlines()
            errors = stderr.readlines()
            outputString = ''.join(output)
            errorsString = ''.join(errors)
            self.ssh.close()

            # self.commandOutputSignal.emit([commandName, command, outputString, errorsString, False])
        except BaseException as exception:
            self.emitCommandOutput(commandName, command, "SSH ERROR", str(exception), False)

    def emitCommandOutput(self, commandName, command, output, errors, firstLog):
        self.commandOutputSignal.emit([commandName, command, output, errors, firstLog])

    def getLineWithoutNewLine(self, stdout):
        line = stdout.readline()
        return line.replace('\n', '')
