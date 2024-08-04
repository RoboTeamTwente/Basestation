import math

class BBPosVel:
    def __init__(self, pos, vel):
        self.pos = pos
        self.vel = vel

class BBTrajectoryPart:
    def __init__(self, acc, startPos, startVel, tEnd):
        self.acc = acc
        self.startPos = startPos
        self.startVel = startVel
        self.tEnd = tEnd

class BBTrajectory1D:
    def __init__(self, startPos, startVel, endPos, maximumVel, maximumAcc):
        self.maxVel = maximumVel
        self.parts = []
        self.finalPos = endPos
        self.generateTrajectory(startPos, startVel, endPos, maximumVel, maximumAcc)

    def fullBrakePos(self, pos, vel, accMax):
        acc = accMax if vel <= 0 else -accMax
        t = -vel / acc
        return pos + 0.5 * vel * t

    def accelerateBrakePos(self, pos0, vel0, vel1, accMax):
        acc1 = accMax if vel1 >= vel0 else -accMax
        acc2 = -accMax if vel1 >= vel0 else accMax
        t1 = (vel1 - vel0) / acc1
        pos1 = pos0 + 0.5 * (vel0 + vel1) * t1
        t2 = -vel1 / acc2
        return pos1 + 0.5 * vel1 * t2

    def triangularProfile(self, startPos, startVel, endPos, maximumAcc, invertedSign):
        acc = maximumAcc if invertedSign else -maximumAcc
        sq = (acc * (endPos - startPos) + 0.5 * startVel ** 2) / (acc ** 2)
        brakeTime = math.sqrt(sq) if sq > 0 else 0
        topVel = acc * brakeTime
        switchTime = (topVel - startVel) / acc
        switchPos = startPos + (startVel + topVel) * 0.5 * switchTime
        self.updatePart(0, switchTime, acc, startVel, startPos)
        self.updatePart(1, switchTime + brakeTime, -acc, topVel, switchPos)

    def trapezoidalProfile(self, startPos, startVel, maximumVel, endPos, maximumAcc):
        acc1 = maximumAcc if startVel < maximumVel else -maximumAcc
        acc3 = -maximumAcc if maximumVel > 0 else maximumAcc
        t1 = (maximumVel - startVel) / acc1
        t3 = -maximumVel / acc3
        startCoastPos = startPos + 0.5 * (startVel + maximumVel) * t1
        endCoastPos = endPos - 0.5 * maximumVel * t3
        t2 = (endCoastPos - startCoastPos) / maximumVel if maximumVel != 0 else 0
        self.updatePart(0, t1, acc1, startVel, startPos)
        self.updatePart(1, t1 + t2, 0, maximumVel, startCoastPos)
        self.updatePart(2, t1 + t2 + t3, acc3, maximumVel, endCoastPos)

    def updatePart(self, index, tEnd, acc, vel, pos):
        if index < len(self.parts):
            self.parts[index] = BBTrajectoryPart(acc, pos, vel, tEnd)
        else:
            self.parts.append(BBTrajectoryPart(acc, pos, vel, tEnd))

    def generateTrajectory(self, startPos, startVel, endPos, maximumVel, maximumAcc):
        brakePos = self.fullBrakePos(startPos, startVel, maximumAcc)
        if brakePos <= endPos:
            accBrakePos = self.accelerateBrakePos(startPos, startVel, maximumVel, maximumAcc)
            if accBrakePos >= endPos:
                self.triangularProfile(startPos, startVel, endPos, maximumAcc, True)
            else:
                self.trapezoidalProfile(startPos, startVel, maximumVel, endPos, maximumAcc)
        else:
            accBrakePos = self.accelerateBrakePos(startPos, startVel, -maximumVel, maximumAcc)
            if accBrakePos <= endPos:
                self.triangularProfile(startPos, startVel, endPos, maximumAcc, False)
            else:
                self.trapezoidalProfile(startPos, startVel, -maximumVel, endPos, maximumAcc)

    def getValues(self, t):
        trajTime = max(0, t)
        if trajTime >= self.getTotalTime():
            return BBPosVel(self.finalPos, 0)
        tPieceStart = 0
        for part in self.parts:
            if trajTime <= part.tEnd:
                break
            tPieceStart = part.tEnd
        tPiece = trajTime - tPieceStart
        return BBPosVel(part.startPos + part.startVel * tPiece + 0.5 * part.acc * tPiece ** 2, part.startVel + part.acc * tPiece)

    def getTotalTime(self):
        return self.parts[-1].tEnd if self.parts else 0

    def getAcceleration(self, t):
        trajTime = max(0, t)
        if trajTime >= self.getTotalTime():
            return 0
        for part in self.parts:
            if trajTime <= part.tEnd:
                return part.acc
        return 0

    def getVelocity(self, t):
        trajTime = max(0, t)
        if trajTime >= self.getTotalTime():
            return 0
        tPieceStart = 0
        for part in self.parts:
            if trajTime <= part.tEnd:
                break
            tPieceStart = part.tEnd
        tPiece = trajTime - tPieceStart
        return part.startVel + part.acc * tPiece

    def getPosition(self, t):
        trajTime = max(0, t)
        if trajTime >= self.getTotalTime():
            return self.finalPos
        tPieceStart = 0
        for part in self.parts:
            if trajTime <= part.tEnd:
                break
            tPieceStart = part.tEnd
        tPiece = trajTime - tPieceStart
        return part.startPos + part.startVel * tPiece + 0.5 * part.acc * tPiece ** 2

    def inLastPart(self, t):
        return t >= self.parts[-2].tEnd if len(self.parts) > 1 else True

    def getParts(self):
        return self.parts