import os
import sys
import time
import utils

# Add parent directory to path to allow importing from Core.Inc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Core.Inc.roboteam_embedded_messages.python import REM_BaseTypes as BaseTypes
from Core.Inc.roboteam_embedded_messages.python.REM_RobotKillCommand import REM_RobotKillCommand

basestation = utils.open_continuous(timeout=0.01)

for i in range(50):
	for is_yellow_team in [True, False]:
		# basestation_config_command = utils.generate_basestation_config_command(is_yellow_team)
		# basestation.write(basestation_config_command.encode())
		for robot_id in range(5,7):
			if robot_id % 2 == i % 2:
				robot_kill_command = utils.generate_empty_robot_kill_command()
				robot_kill_command.toRobotId = robot_id
				basestation.write(robot_kill_command.encode())
		time.sleep(0.1)