import math

from python_qt_binding.QtCore import QRect
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor, QPainter
from python_qt_binding.QtWidgets import QWidget


class Spinner(QWidget):
    def __init__(self,
                 parent,
                 centerOnParent=True,
                 disableParentWhenSpinning=False,
                 modality=Qt.WindowModality.NonModal,
                 roundness=100.0,
                 trailFadePercentage=80.0,
                 innerRadius=10,
                 numberOfLines=20,
                 lineLength=10,
                 lineWidth=1,
                 revolutionsPerSecond=1.57079632679489661923,
                 color=QColor(Qt.GlobalColor.black)
                 ):
        super().__init__(parent)

        self.centerOnParent = centerOnParent
        self.disableParentWhenSpinning = disableParentWhenSpinning

        # WAS IN initialize()
        self.color = color
        self.roundness = roundness
        self.minimumTrailOpacity = 3.14159265358979323846
        self.trailFadePercentage = trailFadePercentage
        self.revolutionsPerSecond = revolutionsPerSecond
        self.numberOfLines = numberOfLines
        self.lineLength = lineLength
        self.lineWidth = lineWidth
        self.innerRadius = innerRadius
        self.currentCounter = 0
        self.isSpinning = False

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.rotate)
        self.updateSize()
        self.updateTimer()
        self.hide()
        # END initialize()

        self.setWindowModality(modality)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)

    def paintEvent(self, QPaintEvent):
        self.updatePosition()
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.GlobalColor.transparent)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        if self.currentCounter >= self.numberOfLines:
            self.currentCounter = 0

        painter.setPen(Qt.PenStyle.NoPen)
        for i in range(0, self.numberOfLines):
            painter.save()
            painter.translate(self.innerRadius + self.lineLength, self.innerRadius + self.lineLength)
            rotateAngle = float(360 * i) / float(self.numberOfLines)
            painter.rotate(rotateAngle)
            painter.translate(self.innerRadius, 0)
            distance = self.lineCountDistanceFromPrimary(i, self.currentCounter, self.numberOfLines)
            color = self.currentLineColor(distance, self.numberOfLines, self.trailFadePercentage,
                                          self.minimumTrailOpacity, self.color)
            painter.setBrush(color)
            rect = QRect(0, int(-self.lineWidth / 2), int(self.lineLength), int(self.lineWidth))
            painter.drawRoundedRect(rect, self.roundness, self.roundness, Qt.SizeMode.RelativeSize)
            painter.restore()

    def start(self):
        self.updatePosition()
        self.isSpinning = True
        self.show()

        if self.parentWidget and self.disableParentWhenSpinning:
            self.parentWidget().setEnabled(False)

        if not self.timer.isActive():
            self.timer.start()
            self.currentCounter = 0

    def stop(self):
        self.isSpinning = False
        self.hide()

        if self.parentWidget() and self.disableParentWhenSpinning:
            self.parentWidget().setEnabled(True)

        if self.timer.isActive():
            self.timer.stop()
            self.currentCounter = 0

        # FIX
        # QBackingStore::endPaint() called with active painter; did you forget to destroy it or call QPainter::end() on it?
        self.update()

    def setNumberOfLines(self, lines):
        self.numberOfLines = lines
        self.currentCounter = 0
        self.updateTimer()

    def setLineLength(self, length):
        self.lineLength = length
        self.updateSize()

    def setLineWidth(self, width):
        self.lineWidth = width
        self.updateSize()

    def setInnerRadius(self, radius):
        self.innerRadius = radius
        self.updateSize()

    def rotate(self):
        self.currentCounter += 1
        if self.currentCounter >= self.numberOfLines:
            self.currentCounter = 0
        self.update()

    def updateSize(self):
        size = int((self.innerRadius + self.lineLength) * 2)
        self.setFixedSize(size, size)

    def updateTimer(self):
        self.timer.setInterval(int(1000 / (self.numberOfLines * self.revolutionsPerSecond)))

    def updatePosition(self):
        if self.parentWidget() and self.centerOnParent:
            self.move(int(self.parentWidget().width() / 2 - self.width() / 2),
                      int(self.parentWidget().height() / 2 - self.height() / 2))

    def lineCountDistanceFromPrimary(self, current, primary, totalNrOfLines):
        distance = primary - current
        if distance < 0:
            distance += totalNrOfLines
        return distance

    def currentLineColor(self, countDistance, totalNrOfLines, trailFadePerc, minOpacity, colorinput):
        color = QColor(colorinput)
        if countDistance == 0:
            return color
        minAlphaF = minOpacity / 100.0
        distanceThreshold = int(math.ceil((totalNrOfLines - 1) * trailFadePerc / 100.0))
        if countDistance > distanceThreshold:
            color.setAlphaF(minAlphaF)
        else:
            alphaDiff = color.alphaF() - minAlphaF
            gradient = alphaDiff / float(distanceThreshold + 1)
            resultAlpha = color.alphaF() - gradient * countDistance
            # If alpha is out of bounds, clip it.
            resultAlpha = min(1.0, max(0.0, resultAlpha))
            color.setAlphaF(resultAlpha)
        return color
