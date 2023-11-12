# This Python file uses the following encoding: utf-8
import json
import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox
from shared.inner_communication import innerCommunication

from .commands.commands import Commands
from .controls.controls import Controls
from .pid.pid_widget import PidWidget
from .ssh_data.ssh_data import SshData
from .video_recorder.video_recorder import VideoRecorder


class SetupWidget(QWidget):
    def __init__(self, stack=None, dataFilePath=None, addMode=False, node=None):
        super(SetupWidget, self).__init__()

        self.stack = stack
        self.addMode = addMode
        self.node = node

        _, self.sharedPath = get_resource('packages', 'shared')
        _, self.packagePath = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(self.packagePath, 'share', 'setup_panel', 'resource', 'setup.ui')
        loadUi(ui_file, self)

        self.defaultFilePath = os.path.join(self.sharedPath, 'share', 'shared', 'data', 'default.json')
        if dataFilePath:
            self.dataFilePath = dataFilePath
        else:
            self.dataFilePath = self.defaultFilePath

        self.initData()

        self.backButton.clicked.connect(self.goBack)
        self.saveButton.clicked.connect(self.saveClicked)

        self.restoreDefaultButton.clicked.connect(self.restoreDefault)

        self.sshData = SshData(self)
        self.pidWidget = PidWidget(self)
        self.controls = Controls(self)
        self.commands = Commands(self)
        self.videoRecorder = VideoRecorder(self)

    def initData(self):
        self.loadData(self.dataFilePath)

    def restoreDefault(self):
        reply = QMessageBox.question(self, 'Restore default settings', 'Are you sure to restore default setting?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.loadData(self.defaultFilePath)

            self.sshData.setData(self.data)
            self.pidWidget.setData(self.data)
            self.commands.setData(self.data)
            self.controls.setData(self.data)
            self.videoRecorder.setData(self.data)


    def goBack(self):
        self.stack.stack.setCurrentIndex(0)

    def saveClicked(self):
        self.commands.saveCommands()
        self.controls.controlsDataSaver.save()
        self.pidWidget.savePidData()
        self.sshData.saveSshData()
        self.videoRecorder.saveData()

        if self.addMode:
            self.addNewRobot()
            return
        self.updateRobotData()

    def addNewRobot(self):
        self.setupIdForNewRobot()
        self.saveToNewFile()

        itemData = {
            "fileName": self.fileName,
            "filePath": self.dataFilePath,
            "id": id,
        }

        innerCommunication.addRobotSignal.emit(itemData)
        self.goBack()

    def setupIdForNewRobot(self):
        self.dataPath = os.path.join(self.sharedPath, 'share', 'shared', 'data', 'robots')

        currentFiles = os.listdir(self.dataPath)
        for index in range(len(os.listdir(self.dataPath)) + 1):
            self.fileName = 'data' + str(index) + '.json'
            if self.fileName in currentFiles:
                continue
            else:
                self.dataFilePath = self.dataPath + '/' + self.fileName
                break

        id = self.findNewId()
        self.data['id'] = id

    def findNewId(self):
        currentIds = []
        currentFiles = os.listdir(self.dataPath)
        for file in currentFiles:
            dataFile = open(self.dataPath + '/' + file, 'r')
            data = json.loads(dataFile.read())
            dataFile.close()
            currentIds.append(data['id'])
        id = None
        for possibleId in range(0, len(currentIds) + 1):
            notFound = not possibleId in currentIds
            if notFound:
                id = possibleId
                break
        return id

    def updateRobotData(self):
        id = self.data.get('id', None)

        itemData = {
            "fileName": None,
            "filePath": self.dataFilePath,
            "id": id,
        }

        self.updateFile()

        innerCommunication.updateRobotSignal.emit(itemData)
        self.goBack()

    def updateFile(self):
        dataFile = open(self.dataFilePath, 'w')
        json.dump(self.data, dataFile)
        dataFile.close()

    def saveToNewFile(self):
        dataFile = open(self.dataFilePath, 'x')
        json.dump(self.data, dataFile)
        dataFile.close()

    def loadData(self, dataFilePath):
        dataFile = open(dataFilePath)
        self.data = json.loads(dataFile.read())
        dataFile.close()
