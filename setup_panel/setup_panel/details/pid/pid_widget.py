# from python_qt_binding.QtWidgets import QWidget
from threading import Event

from python_qt_binding.QtWidgets import QWidget
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from shared.alert.alert import Alert
from shared.services.parameters_server_service import ParametersServerService


class PidWidget(QWidget):
    def __init__(self, widget):
        super(PidWidget, self).__init__()
        self.widget = widget
        self.setData(self.widget.data)
        self.parametersServerService = ParametersServerService(widget)
        pidData = self.data.get('pid', {})
        namespace=self.data.get("namespace")
        self.parametersServerService.setup(pidData, namespace)

        # self.robotNode = self.findRobotNode()
        if self.parametersServerService.robotNode:
            # self.setParamsClients()
            self.initialParams = self.getCurrentParams()
            self.widget.testParamsButtonUI.clicked.connect(self.parametersServerService.setParameters)
            self.widget.testParamsButtonUI.setToolTip("Tests PID params for previously set namespace\n"
                                                      " GUI will make best effort to restore previous params on BACK click")

            self.widget.backButton.clicked.connect(self.restoreInitialParameters)
            self.widget.saveButton.clicked.connect(self.parametersServerService.setParameters)

        else:
            self.widget.testParamsButtonUI.setEnabled(False)
            self.widget.testParamsButtonUI.setToolTip("Node which uses PID params is currently unavailable")
        # self.widget.restoreDefaultButton.clicked.connect(self.restoreDefault)

    def setData(self, data):
        self.data = data
        self.displayData()

    # # it is important to notice that restoreDefault function is called after setData
    # # Hence curre
    # def restoreDefault(self):
    #     self.setParameters()

    def displayData(self):
        pidData = self.data.get('pid', {})

        self.widget.pidAngleKpUI.setValue(pidData.get('pidAngleKp', 0))
        self.widget.pidAngleTiUI.setValue(pidData.get('pidAngleTi', 0))
        self.widget.pidAngleTdUI.setValue(pidData.get('pidAngleTd', 0))

        self.widget.pidSpeedKpUI.setValue(pidData.get('pidSpeedKp', 0))
        self.widget.pidSpeedTiUI.setValue(pidData.get('pidSpeedTi', 0))
        self.widget.pidSpeedTdUI.setValue(pidData.get('pidSpeedTd', 0))

    def savePidData(self):
        self.widget.data['pid'] = {}

        self.widget.data['pid']["pidSpeedKp"] = self.widget.pidSpeedKpUI.value()
        self.widget.data['pid']["pidSpeedTi"] = self.widget.pidSpeedTiUI.value()
        self.widget.data['pid']["pidSpeedTd"] = self.widget.pidSpeedTdUI.value()

        self.widget.data['pid']["pidAngleKp"] = self.widget.pidAngleKpUI.value()
        self.widget.data['pid']["pidAngleTi"] = self.widget.pidAngleTiUI.value()
        self.widget.data['pid']["pidAngleTd"] = self.widget.pidAngleTdUI.value()

    # def findRobotNode(self):
    #     namesAndNamespaces = self.widget.node.get_node_names_and_namespaces()
    #     if len(namesAndNamespaces) == 0:
    #         return
    #
    #     robotNamespace = self.data.get("namespace")
    #
    #     correctNamespacesElement = [(name, namespace) for name, namespace in namesAndNamespaces if
    #                                 robotNamespace in namespace]
    #
    #     if len(correctNamespacesElement) == 0:
    #         return
    #
    #     nodes = [(name, namespace) for name, namespace in namesAndNamespaces if 'motors_controller_cs' in name]
    #     if len(nodes) == 0:
    #         return
    #     node = nodes[0]
    #     return node
    #
    # def setParamsClients(self):
    #     nodeName = self.robotNode[0]
    #     nodeNamespace = self.robotNode[1]
    #     if nodeNamespace == '/':
    #         self.getParamsClient = self.widget.node.create_client(
    #             GetParameters, f'{nodeName}/get_parameters'.format_map(locals())
    #         )
    #         self.setParamsClient = self.widget.node.create_client(
    #             SetParameters, f'{nodeName}/set_parameters'.format_map(locals())
    #         )
    #     else:
    #         self.getParamsClient = self.widget.node.create_client(
    #             GetParameters, f'{nodeNamespace}/{nodeName}/get_parameters'.format_map(locals())
    #         )
    #         self.setParamsClient = self.widget.node.create_client(
    #             SetParameters, f'{nodeNamespace}/{nodeName}/set_parameters'.format_map(locals())
    #         )

    def getCurrentParams(self):
        names = ["pidSpeedKp", "pidSpeedTi", "pidSpeedTd", "pidAngleKp", "pidAngleTi", "pidAngleTd"]

        getParametersRequest = GetParameters.Request()
        getParametersRequest.names = names
        getParametersResponse = self.parametersServerService.callService(self.parametersServerService.getParamsClient, getParametersRequest)

        response = [
            Parameter.from_parameter_msg(ParameterMsg(name=name, value=value))
            for name, value in zip(names, getParametersResponse.values)
        ]

        print("response", response)

        if response is None or len(response) == 0:
            return

        self.widget.currentPidSpeedKpUI.setText(str(response[0].value))
        self.widget.currentpidSpeedTiUI.setText(str(response[1].value))
        self.widget.currentpidSpeedTdUI.setText(str(response[2].value))

        self.widget.currentPidAngleKpUI.setText(str(response[3].value))
        self.widget.currentpidAngleTiUI.setText(str(response[4].value))
        self.widget.currentpidAngleTdUI.setText(str(response[5].value))
        return response

    def restoreInitialParameters(self):
        if self.initialParams is None:
            return
        setParametersRequest = SetParameters.Request()
        setParametersRequest.parameters = [p.to_parameter_msg() for p in self.initialParams]
        response = self.parametersServerService.callService(self.setParamsClient, setParametersRequest)
