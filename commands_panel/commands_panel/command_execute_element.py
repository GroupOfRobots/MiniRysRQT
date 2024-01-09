# This Python file uses the following encoding: utf-8
import os
import re
import subprocess
import sys
import threading
from enum import Enum

import paramiko
from ament_index_python import get_resource
from python_qt_binding.QtGui import QIcon, QPixmap
from python_qt_binding.QtWidgets import QWidget
from shared.enums import PackageNameEnum
from shared.services.kill_local_process_service import KillLocalProcessThread
from shared.services.local_privileges_service import LocalPrivilegesService
from shared.utils.load_ui_file import loadUiFile
from shared.utils.ssh_data import getSSHPassword, getSSHData


class RunStatusIcon(str, Enum):
    TEST = 'hourglass.png'
    RUNNING = 'runningIcon.png'
    RUN = 'runArrow.png'
    STOP = 'stopIcon.png'


SUDO_COMMAND = "sudo "


class CommandExecuteElementWidget(QWidget):
    def __init__(self, command=None, data=None, commandOutputSignal=None):
        super(CommandExecuteElementWidget, self).__init__()
        _, self.packagePath = get_resource('packages', 'commands_panel')

        self.command = command
        self.commandToExecute = command.get('command', '')
        self.commandName = command.get('commandName', '')
        self.data = data
        self.commandOutputSignal = commandOutputSignal

        self.processesPids = []
        self.process = None

        self.isDuringKillingCommand = False

        self.localPrivilegesService = LocalPrivilegesService()
        inputDialogText = ("Your command includes \'sudo\' keyword \n"
                           "Hence this rqt window session \n"
                           "Will be run in \'sudo\' mode")
        self.localPrivilegesService.getPasswordInputDialog.connect(
            lambda: LocalPrivilegesService.addPrivilages(inputDialogText))

        loadUiFile(self, PackageNameEnum.CommandsPanel, 'command_execute_element.ui')

        self.isCommandRunning = False

        self.commandLabelUI.setText(self.commandName)
        self.commandButtonUI.clicked.connect(self.commandButtonClicked)

        self.setupSSHIcon()

    def setupSSHIcon(self):
        if self.isSshCommand():
            iconPath = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'imgs', 'ssh.png')
            icon = QPixmap(iconPath)
            self.sshButtonUI.setIcon(QIcon(icon))

    def commandButtonClicked(self):
        if self.isCommandRunning:
            self.stopCommand()
        else:
            if re.search(SUDO_COMMAND, self.commandToExecute):
                if LocalPrivilegesService.hasPrivileges() is False:
                    self.localPrivilegesService.getPasswordInputDialog.emit(True)

            self.setRunningStatusIcon(RunStatusIcon.RUNNING)

            self.commandThread = threading.Thread(target=self.runCommand)
            self.commandThread.start()

    def setRunningStatusIcon(self, iconName):
        iconPath = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'imgs', iconName)
        icon = QPixmap(iconPath)
        self.commandButtonUI.setIcon(QIcon(icon))

    def stopCommand(self):
        if self.pid is None:
            return
        if self.isDuringKillingCommand:
            return
        if self.isSshCommand():
            self.killProcessWithPid(self.pid)
            self.isDuringKillingCommand = True
            self.setRunningStatusIcon(RunStatusIcon.RUNNING)
        else:
            if LocalPrivilegesService.hasPrivileges() is False:
                self.localPrivilegesService.getPasswordInputDialog.emit(True)
            self.isDuringKillingCommand = True
            self.setRunningStatusIcon(RunStatusIcon.RUNNING)
            self.thread = KillLocalProcessThread(self.pid)
            self.thread.start()

    def killProcessWithPid(self, pid):
        command = f'sudo kill -9 {pid}'
        try:
            password = getSSHPassword(self.data)
            stdin, stdout, stderr = self.ssh.exec_command(command)
            stdin.write(password + '\n')
            stdin.flush()
        except Exception as exception:
            self.emitCommandOutput(self.commandName, command, "SSH KILL ERROR", str(exception), False)

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
        self.isDuringKillingCommand = False
        self.setRunningStatusIcon(RunStatusIcon.RUN)

    def executeCommandLocaly(self):
        # https://www.baeldung.com/linux/just-started-process-pid
        commandExecute = "bash -c 'echo $$ > /tmp/test.pid && exec " + self.commandToExecute + "'"
        # commandExecute = "bash -c 'echo $$ > /tmp/test.pid &&  " + self.commandToExecute + "'"
        # commandExecute = "cmd && bash -c 'echo $$ &&  " + self.commandToExecute + "'"
        # commandExecute = "bash -c 'echo $$ &&  exec " + self.commandToExecute + "'"
        commandExecute = "echo $$ && " + self.commandToExecute
        # commandExecute = "/bin/bash -c 'echo $$ &&  ls && sleep 200 && echo ls'"
        # os.system()
        aa = sys.platform
        print("aa", aa)
        # self.process = subprocess.Popen(commandExecute, stdout=subprocess.PIPE,
        #                                 stderr=subprocess.PIPE, stdin=subprocess.PIPE,  shell=True, start_new_session=True)
        self.process = subprocess.Popen(commandExecute, executable="/bin/bash", stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, stdin=subprocess.PIPE, shell=True,
                                        start_new_session=True)
        # print("_USE_POSIX_SPAWN", self.process._use_posix_spawn():())
        # self.process = subprocess.Popen(commandExecute, stdout=subprocess.PIPE,
        #                                 stderr=subprocess.PIPE, stdin=subprocess.PIPE, shell=True, start_new_session=True)

        self.emitCommandOutput(self.commandName, self.commandToExecute, '', '', True)
        line = self.getLineWithoutNewLine(self.process.stdout)
        self.setPid(line)

        pidOutput = "Process PID: " + line
        self.emitCommandOutput(self.commandName, self.commandToExecute, pidOutput, '', False)

        while self.isCommandRunning:
            line = self.getLineWithoutNewLine(self.process.stdout)
            self.emitCommandOutput(self.commandName, self.commandToExecute, line, '', False)

            if not line:
                self.isCommandRunning = False
                break

        out, err = self.process.communicate()

        if err is not None:
            errorsString = err.decode("utf-8")
            self.emitCommandOutput(self.commandName, self.commandToExecute, '', errorsString, False)

    def executeCommandViaSsh(self):
        host, port, username, password = getSSHData(self.data)
        self.emitCommandOutput(self.commandName, self.commandToExecute, '', '', True)

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            self.ssh.connect(host, port, username, password, timeout=5)

            transport = self.ssh.get_transport()
            self.channel = transport.open_session()
            commandExecute = "bash -c 'echo $$ &&  " + self.commandToExecute + "'"

            stdin, stdout, stderr = self.ssh.exec_command(commandExecute, get_pty=True)
            stdout.channel.set_combine_stderr(True)

            self.processesPids = []
            line = stdout.readline()
            line = line.replace('\n', '')
            self.setPid(line)

            self.emitCommandOutput(self.commandName, self.commandToExecute, line, '', False)

            while self.isCommandRunning:
                line = stdout.readline()
                line = line.replace('\n', '')
                self.emitCommandOutput(self.commandName, self.commandToExecute, line, '', False)
                if not line:
                    self.isCommandRunning = False
                    break
            output = stdout.readlines()
            errors = stderr.readlines()
            outputString = ''.join(output)
            errorsString = ''.join(errors)
            self.commandOutputSignal.emit([self.commandName, self.commandToExecute, outputString, errorsString, False])
            self.ssh.close()

        except BaseException as exception:
            self.emitCommandOutput(self.commandName, self.commandToExecute, "SSH ERROR", str(exception), False)

    def emitCommandOutput(self, commandName, command, output, errors, firstLog):
        self.commandOutputSignal.emit([commandName, command, output, errors, firstLog])

    def getLineWithoutNewLine(self, stdout):
        line = stdout.readline().decode("utf-8")
        return line.replace('\n', '')

    def setPid(self, line):
        try:
            self.pid = int(line)
            self.setRunningStatusIcon(RunStatusIcon.STOP)
        except Exception as exception:
            pass
