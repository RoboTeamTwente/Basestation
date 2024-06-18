import os
import sys
import time

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import utils

basestation = utils.open_continuous(timeout=0.01)
robot_kill_command = utils.generate_empty_robot_kill_command()

for i in range(50):
	for robot_id in range(16):
		if robot_id % 2 == i % 2:
			robot_kill_command.toRobotId = robot_id
			basestation.write(robot_kill_command)
	time.sleep(0.1)
print("All robots killed, enjoy your day")