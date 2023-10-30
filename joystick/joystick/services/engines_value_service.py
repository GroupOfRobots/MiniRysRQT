import math


class EnginesValueService:
    def __init__(self, joystickData):
        self.setJoystickEngineData(joystickData)

    def setJoystickEngineData(self, joystickData):

        joystickMotorCommandData = joystickData.get('motorCommand', {})
        self.joystickTwistData = joystickData.get('twist', {})

        self.joystickForwardData = joystickMotorCommandData.get("forward", {})
        self.joystickRightData = joystickMotorCommandData.get("right", {})
        self.joystickBackwartdData = joystickMotorCommandData.get("backward", {})
        self.joystickLeftData = joystickMotorCommandData.get("right", {})

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

    def calculateMotorCommandEnginesValue(self, angle, joystickR, elipseR):
        joystickDeflection = joystickR / elipseR

        #      pi/2
        #        |
        # pi     |      0
        # ---------------
        # -pi    |      0
        #        |
        #      -pi/2

        if self.checkIfInUpperRightQuarter(angle):
            angleFactor = angle / (math.pi * 0.5)
            data1 = self.joystickRightData
            data2 = self.joystickForwardData

        elif self.checkIfInUpperLeftQuarter(angle):
            angleFactor = (angle - (math.pi * 0.5)) / (math.pi * 0.5)
            data1 = self.joystickForwardData
            data2 = self.joystickLeftData

        elif self.checkIfInBottomRightQuarter(angle):
            angleFactor = abs(angle / (math.pi * 0.5))
            data1 = self.joystickRightData
            data2 = self.joystickBackwartdData

        elif self.checkIfInBottomLeftQuarter(angle):
            angleFactor = abs((angle + (math.pi * 0.5)) / (math.pi * 0.5))
            data1 = self.joystickBackwartdData
            data2 = self.joystickLeftData

        leftEngine, rightEngine = self.calculateEnginesValueInQuarter(data1, data2, angleFactor)
        return leftEngine * joystickDeflection, rightEngine * joystickDeflection

    def calculateTwistEnginesValue(self, angle, joystickR, elipseR):
        joystickDeflection = joystickR / elipseR

        baseLinear = float(self.joystickTwistData.get('linear', 0))
        baseAngle = float(self.joystickTwistData.get('angular', 0))
        sign = math.copysign(1, angle)

        angleVelocity = sign * baseAngle * (1 - abs(angle) / (math.pi / 2)) * joystickDeflection
        linearVelocity = sign * baseLinear * (1 - abs(abs(angle) - math.pi / 2) / (math.pi / 2)) * joystickDeflection
        # print(angle, linearVelocity, angleVelocity)
        return linearVelocity, angleVelocity
