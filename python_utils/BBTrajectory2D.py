import math
from BBTrajectory1D import BBTrajectory1D

class BBTrajectory2D:
    def __init__(self, initialPosX, initialPosY, initialVelX, initialVelY, finalPosX, finalPosY, maxVel, maxAcc):
        self.generateSyncedTrajectory(initialPosX, initialPosY, initialVelX, initialVelY, finalPosX, finalPosY, maxVel, maxAcc)

    def generateTrajectory(self, initialPosX, initialPosY, initialVelX, initialVelY, finalPosX, finalPosY, maxVel, maxAcc, alpha):
        self.x = BBTrajectory1D(initialPosX, initialVelX, finalPosX, maxVel * math.cos(alpha), maxAcc * math.cos(alpha))
        self.y = BBTrajectory1D(initialPosY, initialVelY, finalPosY, maxVel * math.sin(alpha), maxAcc * math.sin(alpha))

    def generateSyncedTrajectory(self, initialPosX, initialPosY, initialVelX, initialVelY, finalPosX, finalPosY, maxVel, maxAcc):
        inc = math.pi / 8
        alpha = math.pi / 4
        iterationLimit = 1e-7
        timeDiffLimit = 1e-3
        while inc > iterationLimit:
            self.generateTrajectory(initialPosX, initialPosY, initialVelX, initialVelY, finalPosX, finalPosY, maxVel, maxAcc, alpha)
            diff = abs(self.x.getTotalTime() - self.y.getTotalTime())
            if diff < timeDiffLimit:
                return
            if self.x.getTotalTime() > self.y.getTotalTime():
                alpha -= inc
            else:
                alpha += inc
            inc *= 0.5

    def getPosition(self, t):
        return self.x.getPosition(t), self.y.getPosition(t)

    def getVelocity(self, t):
        return self.x.getVelocity(t), self.y.getVelocity(t)

    def getAcceleration(self, t):
        return self.x.getAcceleration(t), self.y.getAcceleration(t)

    def getTotalTime(self):
        return max(self.x.getTotalTime(), self.y.getTotalTime())

    def getParts(self):
        return (self.x.getParts(), self.y.getParts())