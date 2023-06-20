# This Python file uses the following encoding: utf-8
import threading

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QIcon, QPixmap, QCursor
from python_qt_binding.QtWidgets import QTableWidgetItem, QHeaderView, QPushButton, QAbstractItemView, QHBoxLayout, \
    QWidget
from shared.base_widget.base_widget import BaseWidget
from ament_index_python import get_resource

from shared.enums import PackageNameEnum
import os

import paramiko


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
            self.cyclicallExecuteSshCommand()

        self.processPanel.closePanelSignal.connect(self.onDestroy)

    def cyclicallExecuteSshCommand(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.executeSshCommand)
        self.timer.start(10000)
        # self.timer = threading.Timer(5.0, self.cyclicallExecuteSshCommand)
        # self.timer.start()
        # self.executeSshCommand()

    def createSSHChannel(self):
        sshData = self.data.get('ssh', {})

        host = sshData.get('host')
        port = sshData.get('port')
        username = sshData.get('username')
        password = sshData.get('password')

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        command = "ps -aux"

        try:
            pass
            # self.ssh.connect(host, port, username, password, timeout=5)

        except BaseException as exception:
            print(exception)
            # self.commandOutputSignal.emit([command, "SSH ERROR", str(exception)])
        self.processTableWidgetUI.setEditTriggers(QAbstractItemView.NoEditTriggers)

    def executeSshCommand(self):
        try:
            # stdin, stdout, stderr = self.ssh.exec_command(command)
            # output = stdout.readlines()
            output = ["TEST"]
            self.processTableWidgetUI.setRowCount(len(output))

            index = 0
            for line in output:
                item = QTableWidgetItem(line)
                cellButton = self.createCellButton()

                self.processTableWidgetUI.setItem(index, 0, item)
                self.processTableWidgetUI.setCellWidget(index, 1, cellButton)
                index += 1
        except BaseException as exception:
            print(exception)

    def createCellButton(self):
        cellWidget = QWidget()
        layout = QHBoxLayout(cellWidget)
        layout.setContentsMargins(0, 0, 0, 0)

        # Create a QPushButton and add it to the layout
        button = QPushButton()
        button.setCursor(QCursor(Qt.PointingHandCursor))

        # Set the icon for the button
        iconPath = os.path.join(self.packagePath, 'share', 'commands_panel', 'resource', 'imgs', "stopIcon.png")
        icon = QPixmap(iconPath)
        button.setIcon(QIcon(icon))
        layout.addWidget(button)
        layout.setAlignment(button, Qt.AlignCenter)

        button.clicked.connect(self.test)

        return cellWidget

    def test(self):
        print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

    def initializeRobotSettings(self):
        print("initializeRobotSettings")
        pass

    def onDestroy(self):
        if self.timer:
            # self.timer.cancel()
            self.timer.stop()
