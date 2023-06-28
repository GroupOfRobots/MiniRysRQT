# This Python file uses the following encoding: utf-8

import os
import re

import paramiko
from ament_index_python import get_resource
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QIcon, QPixmap, QCursor
from python_qt_binding.QtWidgets import QTableWidgetItem, QHeaderView, QPushButton, QAbstractItemView, QHBoxLayout, \
    QWidget
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum

PID_PROCESS_PATTERN = r"\b(\d+)\b"


class ProcessPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None, processPanel=None):
        super(ProcessPanelWidget, self).__init__(stack, PackageNameEnum.ProcessPanel)
        _, self.packagePath = get_resource('packages', 'commands_panel')

        self.processPanel = processPanel
        self.setRobotOnScreen()
        self.createSSHChannel()

        header = self.processTableWidgetUI.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)

        if self.ssh:
            self.executeSshCommand()
            self.searchButtonUI.clicked.connect(self.executeSshCommand)
            self.commandSearchQueryLineEditUI.editingFinished.connect(self.executeSshCommand)
            self.cyclicallExecuteSshCommand()

        self.processPanel.closePanelSignal.connect(self.onDestroy)

    def cyclicallExecuteSshCommand(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.executeSshCommand)
        self.timer.start(10000)

    def createSSHChannel(self):
        sshData = self.data.get('ssh', {})

        host = sshData.get('host')
        port = sshData.get('port')
        username = sshData.get('username')
        password = sshData.get('password')

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            pass
            self.ssh.connect(host, port, username, password, timeout=5)

        except BaseException as exception:
            print("BaseException createSSHChannel")
            print(exception)
        self.processTableWidgetUI.setEditTriggers(QAbstractItemView.NoEditTriggers)

    def executeSshCommand(self):
        try:
            command = "ps -aux"
            searchQuery = self.commandSearchQueryLineEditUI.text()
            if searchQuery:
                command = command + " | grep " + searchQuery
            stdin, stdout, stderr = self.ssh.exec_command(command)
            output = stdout.readlines()
            stdin.flush()

            self.processTableWidgetUI.setRowCount(len(output))

            index = -1
            for line in output:
                index += 1
                item = QTableWidgetItem(line)
                self.processTableWidgetUI.setItem(index, 0, item)

                pid = self.getProcessPid(line)
                if pid is None:
                    continue
                cellButton = self.createCellButton(pid)

                self.processTableWidgetUI.setCellWidget(index, 1, cellButton)
        except BaseException as exception:
            print("BaseException executeSshCommand")
            print(exception)

    def getProcessPid(self, line):
        pids = re.findall(PID_PROCESS_PATTERN, line)
        if len(pids):
            return int(pids[0])
        return None

    def createCellButton(self, pid):
        cellWidget = QWidget()
        layout = QHBoxLayout(cellWidget)
        layout.setContentsMargins(0, 0, 0, 0)

        button = QPushButton()
        button.setCursor(QCursor(Qt.PointingHandCursor))

        iconPath = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'imgs', "stopIcon.png")
        icon = QPixmap(iconPath)
        button.setIcon(QIcon(icon))
        layout.addWidget(button)
        layout.setAlignment(button, Qt.AlignCenter)

        button.clicked.connect(lambda: self.killProcess(pid))

        return cellWidget

    def killProcess(self, pid):
        try:
            killCommand = f"sudo kill -9 {pid}"

            stdin, stdout, stderr = self.ssh.exec_command(killCommand)
            stdin.write('minirys\n')
            stdin.flush()
            self.executeSshCommand()

        except Exception as exception:
            print("Exception killProcess")
            print(exception)

    def onDestroy(self):
        if self.timer:
            self.timer.stop()
        if self.ssh:
            self.ssh.close()
