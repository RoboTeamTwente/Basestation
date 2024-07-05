import math
from ruckig import InputParameter, Ruckig, Trajectory, Result

class BBTrajectory2D:
    def __init__(self, initialPosX, initialPosY, initialVelX, initialVelY, initialAccX, initialAccY, finalPosX, finalPosY, maxVel, maxAcc, maxJerk):
        self.generate_synced_trajectory(initialPosX, initialPosY, initialVelX, initialVelY, initialAccX, initialAccY, finalPosX, finalPosY, maxVel, maxAcc, maxJerk)

    def generate_trajectory(self, initialPos, initialVel, initialAcc, finalPos, maxVel, maxAcc, maxJerk, angle):
        inp = InputParameter(1)
        inp.current_position = [initialPos]
        inp.current_velocity = [initialVel]
        inp.current_acceleration = [initialAcc]

        inp.target_position = [finalPos]
        inp.target_velocity = [0]
        inp.target_acceleration = [0]

        inp.max_velocity = [maxVel * angle]
        inp.max_acceleration = [maxAcc * angle]
        inp.max_jerk = [maxJerk * angle]

        inp.min_velocity = [-maxVel * angle]
        inp.min_acceleration = [-maxAcc * angle]

        otg = Ruckig(1)
        trajectory = Trajectory(1)
        otg.calculate(inp, trajectory)
        return trajectory

    def generate_synced_trajectory(self, initialPosX, initialPosY, initialVelX, initialVelY, initialAccX, initialAccY, finalPosX, finalPosY, maxVel, maxAcc, maxJerk):
        inc = math.pi / 8
        alpha = math.pi / 4
        iteration_limit = 1e-7
        time_diff_limit = 1e-3
        while inc > iteration_limit:
            self.x = self.generate_trajectory(initialPosX, initialVelX, initialAccX, finalPosX, maxVel, maxAcc, maxJerk, math.cos(alpha))
            self.y = self.generate_trajectory(initialPosY, initialVelY, initialAccY, finalPosY, maxVel, maxAcc, maxJerk, math.sin(alpha))

            diff = abs(self.x.duration - self.y.duration)
            if diff < time_diff_limit:
                return
            if self.x.duration > self.y.duration:
                alpha -= inc
            else:
                alpha += inc
            inc *= 0.5

    def getPosition(self, t):
        return self.x.at_time(t)[0][0], self.y.at_time(t)[0][0]

    def getVelocity(self, t):
        return self.x.at_time(t)[1][0], self.y.at_time(t)[1][0]

    def getAcceleration(self, t):
        return self.x.at_time(t)[2][0], self.y.at_time(t)[2][0]

    def get_total_time(self):
        return max(self.x.duration, self.y.duration)
