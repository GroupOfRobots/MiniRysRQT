# This Python file uses the following encoding: utf-8

import subprocess

import paramiko
from ament_index_python import get_resource
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QTableWidgetItem, QHeaderView, QAbstractItemView
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum

from .services.kill_local_process_service import KillLocalProcessService

from .services.display_process_service import DisplayProcessService

PID_PROCESS_PATTERN = r"\b(\d+)\b"
TIMER_INTERVAL=5000

class ProcessPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None, mainPanel=None):
        super(ProcessPanelWidget, self).__init__(stack, PackageNameEnum.ProcessPanel)
        _, self.packagePath = get_resource('packages', 'commands_panel')

        self.processPanel = mainPanel
        self.setRobotOnScreen()
        self.sshCheckboxUI.stateChanged.connect(self.setupExecution)

        self.timer = None
        self.ssh = None

        self.processTableWidgetUI.setEditTriggers(QAbstractItemView.NoEditTriggers)

        header = self.processTableWidgetUI.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)

        self.timer = QTimer()

        self.processPanel.closePanelSignal.connect(self.onDestroy)
        self.displayProcessService = DisplayProcessService(self.processTableWidgetUI, self.packagePath,
                                                           self.killProcess)

        self.setupExecution(self.sshCheckboxUI.checkState())

    def disconnectFunctions(self):
        try:
            self.timer.timeout.disconnect()
            self.commandSearchQueryLineEditUI.editingFinished.disconnect()
            self.searchButtonUI.clicked.disconnect()
        except:
            pass

    def setupSshExecution(self):
        sshChannelCreated = self.createSSHChannel()

        if sshChannelCreated:
            self.disconnectFunctions()
            self.commandSearchQueryLineEditUI.editingFinished.connect(self.executeSshCommand)
            self.searchButtonUI.clicked.connect(self.executeSshCommand)

            self.timer.timeout.connect(self.executeSshCommand)
            if not self.timer.isActive():
                self.timer.start(TIMER_INTERVAL)
            self.executeSshCommand()

        else:
            self.processTableWidgetUI.setRowCount(0)
            if self.timer.isActive():
                self.timer.stop()

    def setupLocalExecution(self):
        self.disconnectFunctions()
        self.commandSearchQueryLineEditUI.editingFinished.connect(self.executeLocalCommandAndClearFocus)
        self.searchButtonUI.clicked.connect(self.executeLocalCommand)

        self.closeSshConnection()

        self.timer.timeout.connect(self.executeLocalCommand)
        if not self.timer.isActive():
            self.timer.start(TIMER_INTERVAL)
        self.executeLocalCommand()

    def setupExecution(self, state):
        if state == Qt.Checked:
           self.setupSshExecution()
        else:
           self.setupLocalExecution()

    def createSSHChannel(self):
        sshData = self.data.get('ssh', {})

        host = sshData.get('host')
        port = sshData.get('port')
        username = sshData.get('username')
        password = sshData.get('password')

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            self.ssh.connect(host, port, username, password, timeout=5)
            return True
        except BaseException as exception:
            print("BaseException createSSHChannel")
            print(exception)
            return False

    def getCommand(self):
        command = "ps -aux"
        searchQuery = self.commandSearchQueryLineEditUI.text()
        if searchQuery:
            command = command + " | grep " + searchQuery
        return command

    def executeLocalCommandAndClearFocus(self):
        self.commandSearchQueryLineEditUI.clearFocus()
        self.executeLocalCommand()

    def executeLocalCommand(self):
        try:
            with subprocess.Popen(self.getCommand(), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  text=True) as process:
                output = [line.strip() for line in process.stdout]

            self.displayProcessService.displayProcesses(output)

        except Exception as exception:
            print(f"Exception executeLocalCommand: {exception}")

    def executeSshCommand(self):
        try:
            stdin, stdout, stderr = self.ssh.exec_command(self.getCommand())
            output = stdout.readlines()
            stdin.flush()

            self.displayProcessService.displayProcesses(output)

        except BaseException as exception:
            print("BaseException executeSshCommand")
            print(exception)

    def killProcess(self, pid):
        if self.sshCheckboxUI.checkState() == Qt.Checked:
            try:
                killCommand = f"sudo kill -9 {pid}"

                stdin, stdout, stderr = self.ssh.exec_command(killCommand)
                stdin.write('minirys\n')
                stdin.flush()
                self.executeSshCommand()

            except Exception as exception:
                print("Exception killProcess")
                print(exception)
        else:
            killed = KillLocalProcessService.kill(pid)
            if killed:
                self.executeLocalCommand()

    def closeSshConnection(self):
        if self.ssh:
            self.ssh.close()

    def onDestroy(self):
        if self.timer:
            self.timer.stop()
        self.closeSshConnection()
