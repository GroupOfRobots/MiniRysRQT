import math


class EnginesValueService:
    def __init__(self, joystickData):
        self.setJoystickEngineData(joystickData)

    def setJoystickEngineData(self, joystickData):
        self.joystickForwardData = joystickData.get("forward", {})
        self.joystickRightData = joystickData.get("right", {})
        self.joystickBackwartdData = joystickData.get("backward", {})
        self.joystickLeftData = joystickData.get("right", {})

    def getValue(self, data, fieldName):
        return int(data.get(fieldName, 0))

    def calculateEngineValue(self, data1, data2, engine_key, angleFactor):
        return self.getValue(data1, engine_key) * (1 - angleFactor) + self.getValue(data2, engine_key) * angleFactor

    def calculateEnginesValueInQuarter(self, data1, data2, angleFactor):
        leftEngine = self.calculateEngineValue(data1, data2, "leftEngine", angleFactor)

        rightEngine = self.calculateEngineValue(data1, data2, "rightEngine", angleFactor)

        return leftEngine, rightEngine

    def checkIfInUpperRightQuarter(self, angle):
        return angle >= 0 and (angle <= math.pi * 0.5)

    def checkIfInUpperLeftQuarter(self, angle):
        return angle > math.pi * 0.5

    def checkIfInBottomRightQuarter(self, angle):
        return angle < 0 and (angle >= -math.pi * 0.5)

    def checkIfInBottomLeftQuarter(self, angle):
        return angle < (-math.pi * 0.5)

    def calculateEnginesValue(self, angle, joystickR, elipseR):
        joystickDeflection = joystickR / elipseR

        leftEngine = 0
        rightEngine = 0

        if self.checkIfInUpperRightQuarter(angle):
            angleFactor = angle / (math.pi * 0.5)

            leftEngine, rightEngine = self.calculateEnginesValueInQuarter(self.joystickRightData,
                                                                          self.joystickForwardData,
                                                                          angleFactor)
        elif self.checkIfInUpperLeftQuarter(angle):
            angleFactor = (angle - (math.pi * 0.5)) / (math.pi * 0.5)

            leftEngine, rightEngine = self.calculateEnginesValueInQuarter(self.joystickForwardData,
                                                                          self.joystickLeftData,
                                                                          angleFactor)

        elif self.checkIfInBottomRightQuarter(angle):
            angleFactor = abs(angle / (math.pi * 0.5))

            leftEngine, rightEngine = self.calculateEnginesValueInQuarter(self.joystickRightData,
                                                                          self.joystickBackwartdData,
                                                                          angleFactor)

        elif self.checkIfInBottomLeftQuarter(angle):
            angleFactor = abs((angle + (math.pi * 0.5)) / (math.pi * 0.5))

            leftEngine, rightEngine = self.calculateEnginesValueInQuarter(self.joystickBackwartdData,
                                                                          self.joystickLeftData,
                                                                          angleFactor)

        return leftEngine, rightEngine
