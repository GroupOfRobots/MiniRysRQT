# This Python file uses the following encoding: utf-8
import math
import os

# from PyQt5.QtWidgets import QGraphicsPixmapItem
# from PyQt5.QtGui import QImage
# from PyQt5.QtWidgets import QGraphicsScene
from python_qt_binding.QtCore import Qt, QPoint, QLineF, pyqtSignal
from python_qt_binding.QtGui import QPainter, QPen
from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.base_widget.base_widget import BaseWidget

from rclpy.node import Node
from minirys_msgs.msg import BatteryStatus, AngularPose
from sensor_msgs.msg import Range

from std_msgs.msg import Float32, String

import time
from collections import namedtuple

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QFileDialog
from python_qt_binding.QtGui import QImage, QPixmap, QTransform


# from python_qt_binding.Qt import AspectRatioMode


class DashboardDistanceSensorsWidget(BaseWidget):
    signal = pyqtSignal(object, name="updateFanValue111")

    def __init__(self, stack=None, node=Node):
        super(DashboardDistanceSensorsWidget, self).__init__()
        print(self.namespace)

        self.node = node
        self.predefineSubscribers()

        self.scene = QGraphicsScene()

        print('aaaaaaaaaaaaaaaaaaaaaaaaaaaa')

        _, packagePath = get_resource('packages', 'dashboard_distance_sensors')
        file_name = os.path.join(packagePath, 'share', 'dashboard_distance_sensors', 'resource', 'imgs', 'rys.png')

        self.image_qt = QImage(file_name)
        self.image_qt = self.image_qt.scaled(500, 500, Qt.IgnoreAspectRatio)
        self.pic = QGraphicsPixmapItem()
        self.pic.setPixmap(QPixmap.fromImage(self.image_qt))
        self.pic.setOpacity(0.5)
        self.scene.addItem(self.pic)

        self.text = self.scene.addText('')
        self.text.setPos(100, 200)
        self.text.setHtml('<div>FRONT </div><div>b</div><div>c</div>')

        self.scene.addLine(0, 0, 100, 100)

        for index, subscriberParam in enumerate(self.subscriberParams):
            line = self.scene.addLine(subscriberParam.QLineF)
            text = self.scene.addText(subscriberParam.textPlaceholder)
            subscriberParam = subscriberParam._replace(line=line)
            subscriberParam = subscriberParam._replace(text=text)

            subscriberParam.text.setPos(subscriberParam.QPoint)
            self.subscriberParams[index] = subscriberParam

        self.graphicsView.setScene(self.scene)
        self.graphicsView.show()

        # self.signal = pyqtSignal(object, name="updateFanValue111")
        self.signal.connect(self.test)

        self.setRobotOnScreen()

    def loadUI(self):
        _, packagePath = get_resource('packages', 'dashboard_distance_sensors')
        uiFile = os.path.join(packagePath, 'share', 'dashboard_distance_sensors', 'resource',
                              'dashboard_distance_sensors.ui')
        loadUi(uiFile, self)

    def predefineSubscribers(self):
        self.topSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_0',
                                                       self.topDistanceSensorCallback, QPoint(250, 0), None, 'TOP',
                                                       QLineF(10, 20, 30, 40), None)

        self.bottomSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_1',
                                                          self.bottomDistanceSensorCallback, QPoint(250, 400),
                                                          None, 'BOTTOM', QLineF(10, 20, 30, 40), None)

        self.rightSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_2',
                                                         self.rightDistanceSensorCallback, QPoint(400, 250),
                                                         None, 'RIGHT', QLineF(10, 20, 30, 40), None)

        self.backSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_3',
                                                        self.backDistanceSensorCallback,
                                                        QPoint(400, 200),
                                                        None, 'BACK', QLineF(10, 20, 30, 40), None)

        self.frontSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_4',
                                                         self.frontDistanceSensorCallback,
                                                         QPoint(200, 200),
                                                         None, 'FRONT', QLineF(10, 20, 30, 40), None)

        self.leftSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_5',
                                                        self.leftDistanceSensorCallback,
                                                        QPoint(0, 250), None, 'LEFT',
                                                        QLineF(10, 20, 30, 40), None)

        self.subscriberParams = [
            self.topSensorSubsciberParam,
            self.bottomSensorSubsciberParam,
            self.rightSensorSubsciberParam,
            self.backSensorSubsciberParam,
            self.frontSensorSubsciberParam,
            self.leftSensorSubsciberParam]

    def resetSubscribers(self):
        for subscriberParam in self.subscriberParams:
            subscriber = subscriberParam.subscriber
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)

    def initializeSubscribers(self):
        self.resetSubscribers()
        print(self.namespace)
        for index, subscriberParam in enumerate(self.subscriberParams):
            subscriberInstance = self.node.create_subscription(subscriberParam.messageType,
                                                               self.namespace + subscriberParam.topic,
                                                               subscriberParam.callback
                                                               , 10)
            #
            # subscriberInstance = self.node.create_subscription(subscriberParam.messageType,
            #                                                    self.namespace + subscriberParam.topic,
            #                                                    lambda event: subscriberParam.callback(event,
            #                                                                                           subscriberParam.text)
            #                                                    , 10)
            subscriberParam = subscriberParam._replace(subscriber=subscriberInstance)
            self.subscriberParams[index] = subscriberParam
        # print(self.subscriberParams[0].subscriber)

    def initializeRobotSettings(self):
        self.initializeSubscribers()

    def topDistanceSensorCallback(self, event):
        # print(event)
        # print("text")
        # # print("text")
        # print(self.subscriberParams[0].text)
        # # self.subscriberParams[0].text.setHtml('<div>FRONT </div><div>b</div><div>' + event.range + '</div>')
        # # self.subscriberParams[0].text.setHtml('<div>FRONT </div><div>b</div><div>' + 'aa' + '</div>')
        # self.lcdNumber.display(event.range)
        # self.topSensorSubsciberParam.setHtml('<div>FRONT </div><div>b</div><div>c</div>')
        # self.topDistanceLcdUI.display(event.range)
        # fanData = {
        #     "panelName": self.fanPanel.name,
        #     "value": self.value
        # }
        self.signal.emit(event)
        # self.test(event)

    def test(self, event):
        self.subscriberParams[0].text.setHtml('<div>FRONT </div><div>' + str(round(event.range, 2)) + '</div>')
        # self.text.setPlainText('aaaaa')
        self.scene.addLine(0, 100, 100, 100)

    def bottomDistanceSensorCallback(self, event):
        # self.bottomDistanceLcdUI.display(event.range)
        pass
        # print(event)

    def rightDistanceSensorCallback(self, event):
        # self.rightDistanceLcdUI.display(event.range)
        # print(event)
        pass

    def backDistanceSensorCallback(self, event):
        # self.backDistanceLcdUI.display(event.range)
        # print(event)
        pass

    def frontDistanceSensorCallback(self, event):
        # self.frontDistanceLcdUI.display(event.range)
        # print(event)
        pass

    def leftDistanceSensorCallback(self, event):
        # self.leftDistanceLcdUI.display(event.range)
        pass
        # print(event)

    # def topDistanceSensorCallback(self, event, text):
    #        # print(event)
    #        print("text")
    #        print("text")
    #        print(text)
    #        # text.setHtml('<div>FRONT </div><div>b</div><div>' + event.range + '</div>')
    #        # self.topSensorSubsciberParam.setHtml('<div>FRONT </div><div>b</div><div>c</div>')
    #        # self.topDistanceLcdUI.display(event.range)
    #
    #    def bottomDistanceSensorCallback(self, event, text):
    #        # self.bottomDistanceLcdUI.display(event.range)
    #        pass
    #        # print(event)
    #
    #    def rightDistanceSensorCallback(self, event, text):
    #        # self.rightDistanceLcdUI.display(event.range)
    #        # print(event)
    #        pass
    #
    #    def backDistanceSensorCallback(self, event, text):
    #        # self.backDistanceLcdUI.display(event.range)
    #        # print(event)
    #        pass
    #
    #    def frontDistanceSensorCallback(self, event, text):
    #        # self.frontDistanceLcdUI.display(event.range)
    #        print(event)
    #        # pass
    #
    #    def leftDistanceSensorCallback(self, event, text):
    #        # self.leftDistanceLcdUI.display(event.range)
    #        pass
    #        # print(event)

    # def backSensorCallback(self, event,text):
    #     # self.backDistanceLcdUI.display(event.range)
    #     print(event)

    def resizeEvent(self, event):
        # self.topSensorSubsciberParam.text.setHtml('<div>FRONT </div><div>b</div><div>c</div>')

        # print(self.topSensorSubsciberParam)
        # print(self.subscriberParams[0])
        graphicsViewSize = self.graphicsView.size()

        self.scene.setSceneRect(0, 0, graphicsViewSize.width(), graphicsViewSize.height())
        self.image_qt = self.image_qt.scaled(graphicsViewSize.width(), graphicsViewSize.height(),
                                             Qt.IgnoreAspectRatio)

        transform = QTransform()
        xScaleFactor = graphicsViewSize.width() / 500
        yScaleFactor = graphicsViewSize.height() / 500
        transform.scale(xScaleFactor, yScaleFactor)
        self.pic.setTransform(transform)

        for subscriberParam in self.subscriberParams:
            # line = subscriberParam.QLineF
            textPoint = subscriberParam.QPoint
            newX = int(textPoint.x() * xScaleFactor)
            newY = int(textPoint.y() * yScaleFactor)

            textNewPoint = QPoint(newX, newY)

            subscriberParam.text.setPos(textNewPoint)

            # point1 = QPoint(int(subscriberParam.QLineF.x1() * xScaleFactor),
            #                 int(subscriberParam.QLineF.y1() * yScaleFactor))
            # point2 = QPoint(int(subscriberParam.QLineF.x2() * xScaleFactor),
            #                 int(subscriberParam.QLineF.y2() * yScaleFactor))
            # subscriberParam.line.setPoints(point1, point2)
            subscriberParam.line.setTransform(transform)


SubscriberParam = namedtuple('SubscriberParam',
                             ["subscriber", "messageType", "topic", "callback", 'QPoint', 'text',
                              'textPlaceholder',
                              'QLineF', 'line'])
