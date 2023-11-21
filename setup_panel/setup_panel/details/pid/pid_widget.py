# from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QWidget

from rclpy.parameter import Parameter


class PidWidget(QWidget):
    def __init__(self, widget):
        super(PidWidget, self).__init__()
        self.widget = widget
        data = self.widget.data
        self.setData(data)
        self.widget.testParamsButtonUI.clicked.connect(self.testParams)

    def setData(self, data):
        pidData = data.get('pid', {})

        self.setPidAngle(pidData.get('pidAngle', {}))
        self.setPidSpeed(pidData.get('pidSpeed', {}))

    def setPidAngle(self, pidAngleData):
        self.widget.pidAngleKpUI.setValue(pidAngleData.get('Kp', 0))
        self.widget.pidAngleKiUI.setValue(pidAngleData.get('Ki', 0))
        self.widget.pidAngleKdUI.setValue(pidAngleData.get('Kd', 0))

    def setPidSpeed(self, pidSpeedData):
        self.widget.pidSpeedKpUI.setValue(pidSpeedData.get('Kp', 0))
        self.widget.pidSpeedKiUI.setValue(pidSpeedData.get('Ki', 0))
        self.widget.pidSpeedKdUI.setValue(pidSpeedData.get('Kd', 0))

    def savePidData(self):
        self.widget.data['pid'] = {
            "pidSpeed": {},
            "pidAngle": {}
        }

        self.widget.data['pid']["pidSpeed"]["Kp"] = self.widget.pidSpeedKpUI.value()
        self.widget.data['pid']["pidSpeed"]["Ki"] = self.widget.pidSpeedKiUI.value()
        self.widget.data['pid']["pidSpeed"]["Kd"] = self.widget.pidSpeedKdUI.value()

        self.widget.data['pid']["pidAngle"]["Kp"] = self.widget.pidAngleKpUI.value()
        self.widget.data['pid']["pidAngle"]["Ki"] = self.widget.pidAngleKiUI.value()
        self.widget.data['pid']["pidAngle"]["Kd"] = self.widget.pidAngleKdUI.value()

        namespace = self.widget.data.get("namespace")

        print(namespace)
        print(self.widget)
        print(self.widget.node)


        # self.widget.node.declare_parameter('my_str', Parameter.Type.STRING)
        # self.widget.node.declare_parameter('my_int', Parameter.Type.INTEGER)
        # self.widget.node.declare_parameter('my_double_array', Parameter.Type.DOUBLE_ARRAY)

        #
        # param_str = Parameter(namespace+'.my_str', Parameter.Type.STRING, 'Set from code')
        # param_int = Parameter(namespace+'.my_int', Parameter.Type.INTEGER, 12)
        # param_double_array = Parameter(namespace+'.my_double_array', Parameter.Type.DOUBLE_ARRAY, [1.1, 2.2])
        # self.widget.node.set_parameters([param_str, param_int, param_double_array])

        # self.widget.node.declare_parameters(
        #     namespace=namespace,
        #     parameters=[("bar", "default_value")]
        # )
        #
        # param_str = Parameter('bar', Parameter.Type.STRING, 'Set from code')
        # print(param_str)
        # self.widget.node.set_parameters([param_str])

    # def test(self, event):
    #     print("event12325")
    #     print("event")
    #     print(event)

    def testParams(self):
        # pidSpeedKp: 0.0013  # 0.001 0.05 to za duzo 0.01 git
        # pidSpeedKi: 0.05  # 0.00000005 0.0005 git 0.5 to zajebiscie duzy uchyb ustalony 5.5 ogromne oscylacje
        # pidSpeedKd: 0.0016  # 0.01
        # pidAngleKp: 43.3  # 100.0
        # pidAngleKi: 0.5  # 0.8   #1.61
        # pidAngleKd: 0.09  # 0
        pass