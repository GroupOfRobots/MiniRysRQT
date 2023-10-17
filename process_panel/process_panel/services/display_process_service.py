import os
import re

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QIcon, QPixmap, QCursor
from python_qt_binding.QtWidgets import QTableWidgetItem, QPushButton, QHBoxLayout, \
    QWidget

PID_PROCESS_PATTERN = r"\b(\d+)\b"


class DisplayProcessService:
    def __init__(self, processTableWidgetUI, packagePath, killProcess):
        self.processTableWidgetUI = processTableWidgetUI
        self.packagePath = packagePath
        self.killProcess = killProcess

    def displayProcesses(self, output):
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
