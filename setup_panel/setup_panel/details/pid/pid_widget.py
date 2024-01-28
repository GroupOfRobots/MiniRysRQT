# from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QThread
from python_qt_binding.QtWidgets import QWidget
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from shared.services.parameters_server_service import ParametersServerService


class PidWidget(QWidget):
    def __init__(self, widget):
        super(PidWidget, self).__init__()
        self.initialParams = None
        self.widget = widget
        self.setData(self.widget.data)
        self.parametersServerService = ParametersServerService(widget)
        pidData = self.data.get('pid', {})
        namespace = self.data.get("namespace")
        self.parametersServerService.setup(pidData, namespace)

        if self.parametersServerService.robotNode:
            # self.initialParams = self.getCurrentParams()
            self.getParamsThread = GetParams(self)
            self.getParamsThread.start()

            self.widget.testParamsButtonUI.clicked.connect(self.setParams)
            self.widget.testParamsButtonUI.setToolTip("Tests PID params for previously set namespace\n"
                                                      " GUI will make best effort to restore previous params on BACK click")

            self.widget.backButton.clicked.connect(self.restoreInitialParameters)
            self.widget.saveButton.clicked.connect(self.setParams)

        else:
           self.disableTestParamsButton()

    def disableTestParamsButton(self):
        self.widget.testParamsButtonUI.setEnabled(False)
        self.widget.testParamsButtonUI.setToolTip("Node which uses PID params is currently unavailable")

    def setData(self, data):
        self.data = data
        self.displayData()

    def setParams(self):
        pidData = self.getPidDataFromUI()
        self.parametersServerService.setPidData(pidData)
        self.parametersServerService.setParameters()
        self.getCurrentParams()

    def displayData(self):
        pidData = self.data.get('pid', {})

        self.widget.pidSpeedKpUI.setValue(pidData.get('pidSpeedKp', 0))
        self.widget.pidSpeedTiUI.setValue(pidData.get('pidSpeedTi', 0))
        self.widget.pidSpeedTdUI.setValue(pidData.get('pidSpeedTd', 0))

        self.widget.pidAngleKpUI.setValue(pidData.get('pidAngleKp', 0))
        self.widget.pidAngleTiUI.setValue(pidData.get('pidAngleTi', 0))
        self.widget.pidAngleTdUI.setValue(pidData.get('pidAngleTd', 0))

    def savePidData(self):
        self.widget.data['pid'] = self.getPidDataFromUI()

    def getPidDataFromUI(self):
        pidData = {}
        pidData["pidSpeedKp"] = self.widget.pidSpeedKpUI.value()
        pidData["pidSpeedTi"] = self.widget.pidSpeedTiUI.value()
        pidData["pidSpeedTd"] = self.widget.pidSpeedTdUI.value()

        pidData["pidAngleKp"] = self.widget.pidAngleKpUI.value()
        pidData["pidAngleTi"] = self.widget.pidAngleTiUI.value()
        pidData["pidAngleTd"] = self.widget.pidAngleTdUI.value()

        return pidData

    def getCurrentParams(self):
        names = ["pidSpeedKp", "pidSpeedTi", "pidSpeedTd", "pidAngleKp", "pidAngleTi", "pidAngleTd"]

        getParametersRequest = GetParameters.Request()
        getParametersRequest.names = names
        getParametersResponse = self.parametersServerService.callService(self.parametersServerService.getParamsClient,
                                                                         getParametersRequest)

        if getParametersResponse is None:
            self.disableTestParamsButton()
            return

        response = [
            Parameter.from_parameter_msg(ParameterMsg(name=name, value=value))
            for name, value in zip(names, getParametersResponse.values)
        ]

        if response is None or len(response) == 0:
            return

        for responseElement in response:
            name = responseElement.name
            value = str(responseElement.value)
            if name == "pidSpeedKp":
                self.widget.currentPidSpeedKpUI.setText(value)
            if name == "pidSpeedTi":
                self.widget.currentPidSpeedTiUI.setText(value)
            if name == "pidSpeedTd":
                self.widget.currentpidSpeedTdUI.setText(value)
            if name == "pidAngleKp":
                self.widget.currentPidAngleKpUI.setText(value)
            if name == "pidAngleTi":
                self.widget.currentpidAngleTiUI.setText(value)
            if name == "pidAngleTd":
                self.widget.currentpidAngleTdUI.setText(value)

        return response

    def restoreInitialParameters(self):
        if self.initialParams is None:
            return
        setParametersRequest = SetParameters.Request()
        setParametersRequest.parameters = [p.to_parameter_msg() for p in self.initialParams]
        response = self.parametersServerService.callService(self.parametersServerService.setParamsClient,
                                                            setParametersRequest)


class GetParams(QThread):
    pid=None
    def __init__(self, widget):
        super(QThread, self).__init__()
        self.widget=widget

    def run(self):
        self.widget.initialParams = self.widget.getCurrentParams()
