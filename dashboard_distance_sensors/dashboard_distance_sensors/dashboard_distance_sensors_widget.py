# This Python file uses the following encoding: utf-8
import os
from collections import namedtuple

from ament_index_python import get_resource
from python_qt_binding.QtCore import Qt, QPoint, QLineF, pyqtSignal
from python_qt_binding.QtGui import QImage, QPixmap, QTransform
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsPixmapItem
from rclpy.node import Node
from sensor_msgs.msg import Range
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum


class DashboardDistanceSensorsWidget(BaseWidget):
    displayDistanceSensorSignal = pyqtSignal(object)

    def __init__(self, stack=None, node=Node):
        super(DashboardDistanceSensorsWidget, self).__init__(stack, PackageNameEnum.DashboardDistanceSensors)
        print(self.namespace)

        self.node = node
        self.predefineSubscribers()

        self.createCanvas()
        self.setBackgroundImage()

        self.initSubscriberParams()

        self.displayDistanceSensorSignal.connect(self.displayDistanceSensorCallback)

        self.setRobotOnScreen()

    def createCanvas(self):
        self.scene = QGraphicsScene()
        self.graphicsView.setScene(self.scene)
        self.graphicsView.show()

    def setBackgroundImage(self):
        _, packagePath = get_resource('packages', 'dashboard_distance_sensors')
        file_name = os.path.join(packagePath, 'share', 'dashboard_distance_sensors', 'resource', 'imgs', 'rys.png')

        self.image_qt = QImage(file_name)
        self.image_qt = self.image_qt.scaled(500, 500, Qt.IgnoreAspectRatio)
        self.pic = QGraphicsPixmapItem()
        self.pic.setPixmap(QPixmap.fromImage(self.image_qt))
        self.pic.setOpacity(0.5)

        self.scene.addItem(self.pic)

    def initSubscriberParams(self):
        for index, subscriberParam in enumerate(self.subscriberParams):
            line = self.scene.addLine(subscriberParam.QLineF)
            text = self.scene.addText(subscriberParam.textPlaceholder)
            subscriberParam = subscriberParam._replace(line=line)
            subscriberParam = subscriberParam._replace(text=text)

            subscriberParam.text.setPos(subscriberParam.QPoint)
            self.subscriberParams[index] = subscriberParam

    def predefineSubscribers(self):
        topSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_0',
                                                  self.topDistanceSensorCallback, QPoint(220, 0), None, 'TOP',
                                                  QLineF(250, 30, 250, 140), None)

        bottomSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_1',
                                                     self.bottomDistanceSensorCallback, QPoint(220, 460),
                                                     None, 'BOTTOM', QLineF(250, 410, 250, 460), None)

        rightSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_2',
                                                    self.rightDistanceSensorCallback, QPoint(450, 190),
                                                    None, 'RIGHT', QLineF(380, 200, 450, 200), None)

        backSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_3',
                                                   self.backDistanceSensorCallback,
                                                   QPoint(450, 50),
                                                   None, 'BACK', QLineF(450, 60, 340, 140), None)

        frontSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_4',
                                                    self.frontDistanceSensorCallback,
                                                    QPoint(50, 50),
                                                    None, 'FRONT', QLineF(90, 60, 200, 200), None)

        leftSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_5',
                                                   self.leftDistanceSensorCallback,
                                                   QPoint(20, 190), None, 'LEFT',
                                                   QLineF(70, 200, 100, 200), None)

        self.subscriberParams = [
            topSensorSubsciberParam,
            bottomSensorSubsciberParam,
            rightSensorSubsciberParam,
            backSensorSubsciberParam,
            frontSensorSubsciberParam,
            leftSensorSubsciberParam]

    def resetSubscribers(self):
        for subscriberParam in self.subscriberParams:
            subscriber = subscriberParam.subscriber
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)

    def initializeSubscribers(self):
        self.resetSubscribers()
        for index, subscriberParam in enumerate(self.subscriberParams):
            subscriberInstance = self.node.create_subscription(subscriberParam.messageType,
                                                               self.namespace + subscriberParam.topic,
                                                               subscriberParam.callback
                                                               , 10)

            subscriberParam = subscriberParam._replace(subscriber=subscriberInstance)
            self.subscriberParams[index] = subscriberParam

    def initializeRobotSettings(self):
        self.initializeSubscribers()

    def emitLabel(self, subscriberParam, event):
        self.displayDistanceSensorSignal.emit(
            {"text": subscriberParam.text, "range": event.range,
             "textPlaceholder": subscriberParam.textPlaceholder})

    def displayDistanceSensorCallback(self, event):
        range = str(round(event["range"], 2))
        textPlaceholder = event.get("textPlaceholder", "")
        event["text"].setHtml('<div>' + textPlaceholder + '</div><div>' + range + '</div>')

    def topDistanceSensorCallback(self, event):
        subscriberParam = self.subscriberParams[0]
        self.emitLabel(subscriberParam, event)

    def bottomDistanceSensorCallback(self, event):
        subscriberParam = self.subscriberParams[1]
        self.emitLabel(subscriberParam, event)

    def rightDistanceSensorCallback(self, event):
        subscriberParam = self.subscriberParams[2]
        self.emitLabel(subscriberParam, event)

    def backDistanceSensorCallback(self, event):
        subscriberParam = self.subscriberParams[3]
        self.emitLabel(subscriberParam, event)

    def frontDistanceSensorCallback(self, event):
        subscriberParam = self.subscriberParams[4]
        self.emitLabel(subscriberParam, event)

    def leftDistanceSensorCallback(self, event):
        subscriberParam = self.subscriberParams[5]
        self.emitLabel(subscriberParam, event)

    def resizeEvent(self, event):
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
            textPoint = subscriberParam.QPoint
            newX = int(textPoint.x() * xScaleFactor)
            newY = int(textPoint.y() * yScaleFactor)

            textNewPoint = QPoint(newX, newY)
            subscriberParam.text.setPos(textNewPoint)
            subscriberParam.line.setTransform(transform)


SubscriberParam = namedtuple('SubscriberParam',
                             ["subscriber", "messageType", "topic", "callback", 'QPoint', 'text',
                              'textPlaceholder',
                              'QLineF', 'line'])
